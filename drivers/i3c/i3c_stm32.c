#include <zephyr/device.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <stm32_ll_i3c.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <stm32h5xx_ll_system.h>
#include <stm32h5xx_ll_cortex.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i3c_stm32, CONFIG_I3C_LOG_LEVEL);

#define DT_DRV_COMPAT st_stm32_i3c

#define CONFIG_I3C_INIT_PRIORITY 50

#define STM32_I3C_SCLH_I2C_MIN_FM_NS  600ull
#define STM32_I3C_SCLH_I2C_MIN_FMP_NS 260ull
#define STM32_I3C_SCLL_OD_MIN_FM_NS   1320ull
#define STM32_I3C_SCLL_OD_MIN_FMP_NS  500ull
#define STM32_I3C_SCLL_OD_MIN_I3C_NS  200ull

#define STM32_I3C_SCLL_PP_MIN_NS  32ull
#define STM32_I3C_SCLH_I3C_MIN_NS 32ull

#define STM32_I3C_TBUF_FMP_MIN_NS 500.0
#define STM32_I3C_TBUF_FM_MIN_NS  1300.0
#define STM32_I3C_TCAS_MIN_NS     38.4

#define STM32_I3C_TRANSFER_TIMEOUT K_MSEC(100)

/* Integer divison with ceiling */
#define INT_DIV_CEIL(a, b) (((a) + (b - 1)) / b)

typedef void (*irq_config_func_t)(const struct device *port);

enum i3c_stm32_msg_state {
	STM32_I3C_MSG_DAA,    /* Dynamic addressing state */
	STM32_I3C_MSG_CCC,    /* First part of CCC command state*/
	STM32_I3C_MSG_CCC_P2, /* Second part of CCC command state (used for direct commands)*/
	STM32_I3C_MSG,        /* Private msg state */
	STM32_I2C_MSG,        /* I2C legacy msg state */
	STM32_I3C_MSG_IDLE,   /* Idle bus state */
	STM32_I3C_MSG_ERR,    /* Error state */
	STM32_I3C_MSG_INVAL,  /* Invalid state */
};

struct i3c_stm32_config {
	struct i3c_driver_config drv_cfg;      /* I3C driver config */
	I3C_TypeDef *i3c;                      /* Pointer to I3C module base addr */
	irq_config_func_t irq_config_func;     /* IRQ config function */
	const struct stm32_pclken *pclken;     /* Pointer to peripheral clock configuration */
	const struct pinctrl_dev_config *pcfg; /* Pointer to pin control configuration */
};

struct i3c_stm32_data {
	struct i3c_driver_data drv_data;     /* I3C driver data */
	enum i3c_stm32_msg_state msg_state;  /* Current I3C bus state */
	struct i3c_ccc_payload *ccc_payload; /* Current CCC message payload */
	struct i3c_ccc_target_payload *
		ccc_target_payload; /* Current target addressed by 2nd part of direct CCC command */
	size_t ccc_target_idx;      /* Current target index, used for filling C-FIFO */
	struct k_sem device_sync_sem; /* Sync between device communication messages */
	struct k_sem bus_mutex;       /* Sync between transfers */
	uint8_t target_addr;          /* Current target xfer address */
	struct i3c_msg
		*current_i3c_msg; /* Pointer to the current private message to send on the bus */
	struct i3c_msg *current_i3c_msg_cf; /* Pointer to the private messagethat will be used by
						   the control FIFO */
	struct i2c_msg
		*current_i2c_msg; /* Pointer to the current legacy message to send on the bus */
	struct i2c_msg *current_i2c_msg_cf; /* Pointer to the I2C legavy message that will be used
						   by the control FIFO */
	size_t current_i2c_msg_idx;         /* Current I2C legacy message transfer index */
	uint8_t num_msgs;                   /* Number of messages to send on bus */
	uint64_t pid;                       /* Current DAA target PID */
	size_t daa_rx_rcv;                  /* Number of RX bytes received during DAA */
	uint8_t target_id;                  /* Target id */
#ifdef CONFIG_I3C_USE_IBI
	uint32_t ibi_payload;      /* Received ibi payload */
	uint32_t ibi_payload_size; /* Received payload size */
	uint32_t ibi_target_addr;  /* Received target dynamic address */
	struct {
		uint8_t addr[4];       /* List of target addresses */
		uint8_t num_addr;      /* Number of valid addresses */
	} ibi;
	struct k_sem ibi_lock_sem; /* Semaphore used for ibi requests */
#endif
};

/* Activates the device I3C pinctrl and CLK */
static int i3c_stm32_activate(const struct device *dev)
{
	int ret;
	struct i3c_stm32_config *config = (struct i3c_stm32_config *)dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (clock_control_on(clk, (clock_control_subsys_t)&config->pclken[0]) != 0) {
		return -EIO;
	}
	return 0;
}

static void i3c_stm32_flush_all_fifo(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	LL_I3C_RequestControlFIFOFlush(i3c);
	LL_I3C_RequestRxFIFOFlush(i3c);
	LL_I3C_RequestTxFIFOFlush(i3c);
}

static void i3c_stm32_clear_err(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;

	i3c_stm32_flush_all_fifo(dev);

	data->msg_state = STM32_I3C_MSG_IDLE;
}

static int i3c_stm32_calc_scll_od_sclh_i2c(uint32_t i2c_bus_freq, uint32_t i3c_clock,
					   uint8_t *scll_od, uint8_t *sclh_i2c)
{
	if (i2c_bus_freq > 400000) {
		/* I2C bus is FM+ */
		*scll_od =
			INT_DIV_CEIL(STM32_I3C_SCLL_OD_MIN_FMP_NS * i3c_clock, 1000000000ull) - 1;
	} else if (i2c_bus_freq > 0) {
		/* I2C bus is FM */
		*scll_od = INT_DIV_CEIL(STM32_I3C_SCLL_OD_MIN_FM_NS * i3c_clock, 1000000000ull) - 1;
	} else {
		/* Assume no I2C devices on the bus */
		*scll_od =
			INT_DIV_CEIL(STM32_I3C_SCLL_OD_MIN_I3C_NS * i3c_clock, 1000000000ull) - 1;
	}

	if (i2c_bus_freq > 0) {
		*sclh_i2c = INT_DIV_CEIL(i3c_clock, i2c_bus_freq) - *scll_od - 2;
	} else {
		*sclh_i2c = 0;
	}

	if (i2c_bus_freq > 400000 &&
		*sclh_i2c <
			INT_DIV_CEIL(STM32_I3C_SCLH_I2C_MIN_FMP_NS * i3c_clock, 1000000000ull) - 1) {
		LOG_ERR("Cannot find a combination of SCLL_OD and SCLH_I2C at current I3C clock "
			"frequency for FM+ I2C bus");
		return -EINVAL;
	}

	if (i2c_bus_freq > 0 &&
		*sclh_i2c < INT_DIV_CEIL(STM32_I3C_SCLH_I2C_MIN_FM_NS * i3c_clock, 1000000000ull) - 1) {
		LOG_ERR("Cannot find a combination of SCLL_OD and SCLH_I2C at current I3C clock "
			"frequency for FM I2C bus");
		return -EINVAL;
	}

	LOG_DBG("TimingReg0: SCLL_OD = %d, SCLH_I2C = %d", *scll_od, *sclh_i2c);
	return 0;
}

static int i3c_stm32_calc_scll_pp_sclh_i3c(uint32_t i3c_bus_freq, uint32_t i3c_clock,
					   uint8_t *scll_pp, uint8_t *sclh_i3c)
{
	*sclh_i3c = INT_DIV_CEIL(STM32_I3C_SCLH_I3C_MIN_NS * i3c_clock, 1000000000ull) - 1;
	*scll_pp = INT_DIV_CEIL(i3c_clock, i3c_bus_freq) - *sclh_i3c - 2;

	if (*scll_pp < INT_DIV_CEIL(STM32_I3C_SCLL_PP_MIN_NS * i3c_clock, 1000000000ull) - 1) {
		LOG_ERR("Cannot find a combination of SCLL_PP and SCLH_I3C at current I3C clock "
			"frequency for specified I3C bus speed");
		return -EINVAL;
	}

	LOG_DBG("TimingReg0: SCLL_PP = %d, SCLH_I3C = %d", *scll_pp, *sclh_i3c);
	return 0;
}

static int i3c_stm32_config_clk_wave(const struct device *dev)
{
	const struct i3c_stm32_config *cfg = dev->config;
	struct i3c_stm32_data *data = dev->data;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	I3C_TypeDef *i3c = cfg->i3c;
	uint32_t i3c_clock = 0;
	uint32_t i2c_bus_freq = data->drv_data.ctrl_config.scl.i2c;
	uint32_t i3c_bus_freq = data->drv_data.ctrl_config.scl.i3c;

	if (clock_control_get_rate(clk, (clock_control_subsys_t)&cfg->pclken[0], &i3c_clock) < 0) {
		LOG_ERR("Failed call clock_control_get_rate(pclken[0])");
		return -EIO;
	}

	uint8_t scll_od = 0;
	uint8_t sclh_i2c = 0;
	uint8_t scll_pp = 0;
	uint8_t sclh_i3c = 0;
	uint32_t clk_wave = 0;
	int ret;

	LOG_DBG("I3C Clock = %u, I2C Bus Freq = %u, I3C Bus Freq = %u", i3c_clock, i2c_bus_freq,
		i3c_bus_freq);

	ret = i3c_stm32_calc_scll_od_sclh_i2c(i2c_bus_freq, i3c_clock, &scll_od, &sclh_i2c);
	if (ret != 0) {
		LOG_ERR("Cannot calculate the timing for TimingReg0, err=%d", ret);
		return ret;
	}

	ret = i3c_stm32_calc_scll_pp_sclh_i3c(i3c_bus_freq, i3c_clock, &scll_pp, &sclh_i3c);
	if (ret != 0) {
		LOG_ERR("Cannot calculate the timing for TimingReg0, err=%d", ret);
		return ret;
	}

	clk_wave = ((uint32_t)sclh_i2c << 24) | ((uint32_t)scll_od << 16) |
		   ((uint32_t)sclh_i3c << 8) | (scll_pp);

	LOG_DBG("TimigReg0 = 0x%08x", clk_wave);

	LL_I3C_ConfigClockWaveForm(i3c, clk_wave);

	return 0;
}
/**
 * @brief Get current configuration of the I3C hardware.
 *
 * @param[in] dev Pointer to controller device driver instance.
 * @param[in] type Type of configuration.
 * @param[in,out] config Pointer to the configuration parameters.
 *
 * @retval 0 If successful.
 * @retval -EIO General Input/Output errors.
 * @retval -ENOSYS If not implemented.
 */
static int i3c_stm32_config_get(const struct device *dev, enum i3c_config_type type, void *config)
{
	struct i3c_stm32_data *data = dev->data;

	if ((type != I3C_CONFIG_CONTROLLER) || (config == NULL)) {
		return -EINVAL;
	}

	(void)memcpy(config, &data->drv_data.ctrl_config, sizeof(data->drv_data.ctrl_config));

	return 0;
}

static int i3c_stm32_config_ctrl_bus_char(const struct device *dev)
{
	const struct i3c_stm32_config *cfg = dev->config;
	struct i3c_stm32_data *data = dev->data;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	I3C_TypeDef *i3c = cfg->i3c;
	uint32_t i3c_clock = 0;
	uint32_t i2c_bus_freq = data->drv_data.ctrl_config.scl.i2c;

	uint8_t free = 0;
	uint8_t aval = 0;

	if (clock_control_get_rate(clk, (clock_control_subsys_t)&cfg->pclken[0], &i3c_clock) < 0) {
		LOG_ERR("Failed call clock_control_get_rate(pclken[0])");
		return -EIO;
	}

	/* Satisfying I3C start timing min timing will statisfy the rest of the conditions */
	if (i2c_bus_freq > 400000) {
		/* Mixed bus with I2C FM+ device */
		free = (uint8_t)ceil((STM32_I3C_TBUF_FMP_MIN_NS * i3c_clock / 1e9 - 0.5) / 2);
	} else if (i2c_bus_freq > 0) {
		/* Mixed bus with I2C FM device */
		free = (uint8_t)ceil((STM32_I3C_TBUF_FM_MIN_NS * i3c_clock / 1e9 - 0.5) / 2);
	} else {
		/* Pure I3C bus */
		free = (uint8_t)ceil((STM32_I3C_TCAS_MIN_NS * i3c_clock / 1e9 - 0.5) / 2);
	}

	aval = INT_DIV_CEIL(1000ull * i3c_clock, 1000000000ull) - 1;

	LL_I3C_SetDataHoldTime(i3c, LL_I3C_SDA_HOLD_TIME_1_5);
	LL_I3C_SetFreeTiming(i3c, free);
	LL_I3C_SetControllerActivityState(i3c, LL_I3C_OWN_ACTIVITY_STATE_0);
	LL_I3C_SetAvalTiming(i3c, aval);

	LOG_DBG("TimingReg1 = 0x%08x", LL_I3C_GetCtrlBusCharacteristic(i3c));

	return 0;
}

/* Configures the I3C module in controller mode */
static int i3c_stm32_configure(const struct device *dev, enum i3c_config_type type, void *cfg)
{
	int ret;

	if (type == I3C_CONFIG_TARGET || type == I3C_CONFIG_CUSTOM) {
		return -EOPNOTSUPP;
	}

	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;

	I3C_TypeDef *i3c = config->i3c;

	ret = i3c_stm32_activate(dev);
	if (ret != 0) {
		LOG_ERR("Clock and GPIO could not be initialized for the I3C module, err=%d", ret);
		return ret;
	}

	/* I3C Initialization */
	LL_I3C_SetMode(i3c, LL_I3C_MODE_CONTROLLER);
	LL_I3C_SetDataHoldTime(i3c, LL_I3C_SDA_HOLD_TIME_1_5);
	LL_I3C_SetControllerActivityState(i3c, LL_I3C_OWN_ACTIVITY_STATE_0);

	/* Timing currently works on 120 MHz on ABP1 */

	ret = i3c_stm32_config_clk_wave(dev);
	if (ret != 0) {
		LOG_ERR("TimigReg0 timing could not be calculated, err=%d", ret);
		return ret;
	}

	ret = i3c_stm32_config_ctrl_bus_char(dev);
	if (ret != 0) {
		LOG_ERR("TimingReg1 timing could not be calculated, err=%d", ret);
		return ret;
	}

#ifdef CONFIG_I3C_USE_IBI
	LL_I3C_EnableHJAck(i3c);
#endif

	/* Configure FIFO */
	LL_I3C_SetRxFIFOThreshold(i3c, LL_I3C_RXFIFO_THRESHOLD_1_4);
	LL_I3C_SetTxFIFOThreshold(i3c, LL_I3C_TXFIFO_THRESHOLD_1_4);
	LL_I3C_EnableControlFIFO(i3c);
	LL_I3C_EnableStatusFIFO(i3c);

	/* Configure Controller */
	LL_I3C_SetOwnDynamicAddress(i3c, 0);
	LL_I3C_EnableOwnDynAddress(i3c);
	LL_I3C_SetStallTime(i3c, 0x00);
	LL_I3C_DisableStallACK(i3c);
	LL_I3C_DisableStallParityCCC(i3c);
	LL_I3C_DisableStallParityData(i3c);
	LL_I3C_DisableStallTbit(i3c);
	LL_I3C_DisableHighKeeperSDA(i3c);

	LL_I3C_Enable(i3c);

	LL_I3C_EnableIT_FC(i3c);
	LL_I3C_EnableIT_CFNF(i3c);
	LL_I3C_EnableIT_SFNE(i3c);
	LL_I3C_EnableIT_RXFNE(i3c);
	LL_I3C_EnableIT_TXFNF(i3c);
	LL_I3C_EnableIT_ERR(i3c);
	LL_I3C_EnableIT_WKP(i3c);
#ifdef CONFIG_I3C_USE_IBI
	LL_I3C_EnableIT_IBI(i3c);
	LL_I3C_EnableIT_HJ(i3c);
#endif

	/* Bus will be idle initially */
	data->msg_state = STM32_I3C_MSG_IDLE;
	data->target_id = 0;
	data->ibi_payload = 0;
	data->ibi_payload_size = 0;
	data->ibi_target_addr = 0;

	return 0;
}

static int i3c_stm32_i2c_configure(const struct device *dev, uint32_t config)
{
	switch (I2C_SPEED_GET(config)) {
	case I2C_SPEED_FAST:
	case I2C_SPEED_FAST_PLUS:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Find a registered I3C target device.
 *
 * This returns the I3C device descriptor of the I3C device
 * matching the incoming @p id.
 *
 * @param dev Pointer to controller device driver instance.
 * @param id Pointer to I3C device ID.
 *
 * @return @see i3c_device_find.
 */
static struct i3c_device_desc *i3c_stm32_device_find(const struct device *dev,
						    const struct i3c_device_id *id)
{
	const struct i3c_stm32_config *config = dev->config;

	return i3c_dev_list_find(&config->drv_cfg.dev_list, id);

}

/* Handles broadcast/direct CCCs except for ENTDAA */
static int i3c_stm32_do_ccc(const struct device *dev, struct i3c_ccc_payload *payload)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

	if (dev == NULL || payload == NULL) {
		return -EINVAL;
	}

	if (payload->ccc.id == I3C_CCC_ENTDAA) {
		return -EINVAL;
	}

	/* Check if payload has targets when sending a direct CCC */
	if (!i3c_ccc_is_payload_broadcast(payload) &&
		(payload->targets.payloads == NULL || payload->targets.num_targets == 0)) {
		return -EINVAL;
	}

	if (payload->ccc.data_len > 0 && payload->ccc.data == NULL) {
		return -EINVAL;
	}

#ifdef CONFIG_PM_DEVICE_RUNTIME
	(void)pm_device_runtime_get(dev);
#endif
	/* Prevent the clocks to be stopped during the transaction */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	/* Mark current transfer as CCC */
	data->msg_state = STM32_I3C_MSG_CCC;
	data->ccc_payload = payload;
	data->ccc_target_idx = 0;
	data->ccc_target_payload = payload->targets.payloads;

	payload->ccc.num_xfer = 0;

	for (size_t i = 0; i < payload->targets.num_targets; i++) {
		payload->targets.payloads->num_xfer = 0;
	}

	/* Start CCC transfer */
	LL_I3C_ControllerHandleCCC(i3c, payload->ccc.id, payload->ccc.data_len,
				   (i3c_ccc_is_payload_broadcast(payload)
						? LL_I3C_GENERATE_STOP
						: LL_I3C_GENERATE_RESTART));

	/* Wait for CCC to complete */
	if (k_sem_take(&data->device_sync_sem, STM32_I3C_TRANSFER_TIMEOUT) != 0) {
		i3c_stm32_clear_err(dev);
		return -ETIMEDOUT;
	}

	if (data->msg_state == STM32_I3C_MSG_ERR) {
		i3c_stm32_clear_err(dev);
		return -EIO;
	}

	return 0;
}

/* Handles the ENTDAA CCC */
static int i3c_stm32_do_daa(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

#ifdef CONFIG_PM_DEVICE_RUNTIME
	(void)pm_device_runtime_get(dev);
#endif
	/* Prevent the clocks to be stopped during the transaction */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	/* Mark current transfer as DAA */
	data->msg_state = STM32_I3C_MSG_DAA;

	/* Disable TXFNF interrupt, the RXFNE interrupt will enable it once all PID bytes are
	 * received
	 */
	LL_I3C_DisableIT_TXFNF(i3c);

	/* Start DAA */
	LL_I3C_ControllerHandleCCC(i3c, I3C_CCC_ENTDAA, 0, LL_I3C_GENERATE_STOP);

	/* Wait for DAA to finish */
	if (k_sem_take(&data->device_sync_sem, STM32_I3C_TRANSFER_TIMEOUT) != 0) {
		return -ETIMEDOUT;
	}

	if (data->msg_state == STM32_I3C_MSG_ERR) {
		i3c_stm32_clear_err(dev);
		/* Enable TXFNF interrupt in case an error occured before it was enabled by RXFNE */
		LL_I3C_EnableIT_TXFNF(i3c);
		return -EIO;
	}

	return 0;
}

/* Handles the controller private read/write transfers */
static int i3c_stm32_transfer(const struct device *dev, struct i3c_device_desc *target,
				  struct i3c_msg *msgs, uint8_t num_msgs)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	/* Verify all messages */
	for (size_t i = 0; i < num_msgs; i++) {
		if (msgs[i].buf == NULL) {
			return -EINVAL;
		}
	}

	k_sem_take(&data->bus_mutex, K_FOREVER);

	data->msg_state = STM32_I3C_MSG;
	data->target_addr = target->dynamic_addr;

#ifdef CONFIG_PM_DEVICE_RUNTIME
	(void)pm_device_runtime_get(dev);
#endif

	/* Prevent the clocks to be stopped during the transaction */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	data->current_i3c_msg_cf = &msgs[0];
	data->current_i3c_msg = &msgs[0];
	data->num_msgs = num_msgs;

	LL_I3C_RequestTransfer(i3c);

	/* Wait for whole transfer to complete */
	if (k_sem_take(&data->device_sync_sem, STM32_I3C_TRANSFER_TIMEOUT) != 0) {
		return -ETIMEDOUT;
	}

	if (data->msg_state == STM32_I3C_MSG_ERR) {
		i3c_stm32_clear_err(dev);
		k_sem_give(&data->bus_mutex);
		return -EIO;
	}

	k_sem_give(&data->bus_mutex);

	return 0;
}

static int i3c_stm32_i2c_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				  uint16_t addr)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	if (addr > 0xff) {
		LOG_ERR("This device only supports 7-bit addressing mode");
		return -EINVAL;
	}

	/* Verify all messages */
	for (size_t i = 0; i < num_msgs; i++) {
		if (msgs[i].buf == NULL) {
			return -EINVAL;
		}
	}

	k_sem_take(&data->bus_mutex, K_FOREVER);

	data->msg_state = STM32_I2C_MSG;
	data->target_addr = addr;

	/* Disable arbitration header for all I2C messages in case no I3C devices exist on bus */
	LL_I3C_DisableArbitrationHeader(i3c);

#ifdef CONFIG_PM_DEVICE_RUNTIME
	(void)pm_device_runtime_get(dev);
#endif

	/* Prevent the clocks to be stopped during the transaction */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	data->current_i2c_msg_cf = &msgs[0];
	data->current_i2c_msg = &msgs[0];
	data->current_i2c_msg_idx = 0;
	data->num_msgs = num_msgs;

	LL_I3C_RequestTransfer(i3c);

	/* Wait for current transfer to complete */
	if (k_sem_take(&data->device_sync_sem, STM32_I3C_TRANSFER_TIMEOUT) != 0) {
		LL_I3C_EnableArbitrationHeader(i3c);
		return -ETIMEDOUT;
	}

	if (data->msg_state == STM32_I3C_MSG_ERR) {
		i3c_stm32_clear_err(dev);
		LL_I3C_EnableArbitrationHeader(i3c);
		k_sem_give(&data->bus_mutex);
		return -EIO;
	}

	LL_I3C_EnableArbitrationHeader(i3c);

	k_sem_give(&data->bus_mutex);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int i3c_stm32_suspend(const struct device *dev)
{
	int ret;
	const struct i3c_stm32_config *cfg = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	/* Disable device clock. */
	ret = clock_control_off(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (ret < 0) {
		LOG_ERR("failure disabling I3C clock");
		return ret;
	}

	/* Move pins to sleep state */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
	if (ret == -ENOENT) {
		/* Warn but don't block suspend */
		LOG_WRN("I3C pinctrl sleep state not available");
	} else if (ret < 0) {
		return ret;
	}

	return 0;
}

static int i3c_stm32_pm_action(const struct device *dev, enum pm_device_action action)
{
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		err = i3c_stm32_activate(dev);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		err = i3c_stm32_suspend(dev);
		break;
	default:
		return -ENOTSUP;
	}

	return err;
}
#endif

/* Initializes the I3C device and I3C bus */
static int i3c_stm32_init(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	int ret;

	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);

	/* initialize mutex used when multiple transfers
	 * are taking place to guarantee that each one is
	 * atomic and has exclusive access to the I3C bus.
	 */
	k_sem_init(&data->bus_mutex, 1, 1);

	/* initialize semaphore used when multiple ibi requests are taking place */
	k_sem_init(&data->ibi_lock_sem, 1, 1);

	ret = i3c_addr_slots_init(dev);
	if (ret != 0) {
		LOG_ERR("Addr slots init fail, err=%d", ret);
		return ret;
	}

	config->irq_config_func(dev);

	i3c_stm32_configure(dev, I3C_CONFIG_CONTROLLER, NULL);

#ifdef CONFIG_PM_DEVICE_RUNTIME
	(void)pm_device_runtime_enable(dev);
#endif

	// ret = i3c_bus_init(dev, &config->drv_cfg.dev_list);
	// if (ret != 0) {
	// 	LOG_ERR("Failed to do i3c bus init, err=%d", ret);
	// 	return ret;
	// }

	return 0;
}

static void i3c_stm32_event_isr_tx(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

	switch (data->msg_state) {
	case STM32_I3C_MSG: {
		struct i3c_msg *msg = data->current_i3c_msg;

		if (LL_I3C_IsActiveFlag_TXLAST(i3c)) {
			data->current_i3c_msg++;
		}

		if (msg->num_xfer < msg->len) {
			LL_I3C_TransmitData8(i3c, msg->buf[msg->num_xfer++]);
		}

		break;
	}
	case STM32_I2C_MSG: {
		struct i2c_msg *msg = data->current_i2c_msg;

		if (LL_I3C_IsActiveFlag_TXLAST(i3c)) {
			data->current_i2c_msg++;
		}

		if (data->current_i2c_msg_idx < msg->len) {
			LL_I3C_TransmitData8(i3c, msg->buf[data->current_i2c_msg_idx++]);
		}
		break;
	}
	case STM32_I3C_MSG_DAA: {
		struct i3c_device_desc *target;
		uint8_t bcr;
		uint8_t dcr;
		uint8_t dyn_addr = 0;
		int ret;

		bcr = (data->pid >> 8) & 0xFF;
		dcr = data->pid & 0xFF;
		data->pid >>= 16;

		/* Find the device in the device list */
		ret = i3c_dev_list_daa_addr_helper(&data->drv_data.attached_dev.addr_slots,
						   &config->drv_cfg.dev_list, data->pid, false,
						   false, &target, &dyn_addr);
		if (ret != 0) {
			/* TODO: figure out what is the correct sequence to exit form this error
			 * It is expected that a TX overrun error to occur which triggers err isr
			 */
			LOG_ERR("No dynamic address could be assigned to target");

			return;
		}

		/* Put the new dynamic address in TX FIFO for transmission */
		LL_I3C_TransmitData8(i3c, dyn_addr);

		if (target != NULL) {
			/* Update target descriptor */
			target->dynamic_addr = dyn_addr;
			target->bcr = bcr;
			target->dcr = dcr;
		}

		/* Mark the address as used */
		i3c_addr_slots_mark_i3c(&data->drv_data.attached_dev.addr_slots, dyn_addr);

		/* Mark the static address as free */
		if ((target != NULL) && (target->static_addr != 0) &&
			(dyn_addr != target->static_addr)) {
			i3c_addr_slots_mark_free(&data->drv_data.attached_dev.addr_slots, dyn_addr);
		}

		break;
	}
	case STM32_I3C_MSG_CCC: {
		struct i3c_ccc_payload *payload = data->ccc_payload;

		if (payload->ccc.num_xfer < payload->ccc.data_len) {
			LL_I3C_TransmitData8(i3c, payload->ccc.data[payload->ccc.num_xfer++]);
		}
		break;
	}
	case STM32_I3C_MSG_CCC_P2: {
		struct i3c_ccc_target_payload *target = data->ccc_target_payload;

		if (target->num_xfer < target->data_len) {
			LL_I3C_TransmitData8(i3c, target->data[target->num_xfer++]);

			/* After sending all bytes for current target, move on to the next target */
			if (target->num_xfer == target->data_len) {
				data->ccc_target_payload++;
			}
		}
		break;
	}
	default:
		break;
	}
}

static void i3c_stm32_event_isr_rx(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

	switch (data->msg_state) {
	case STM32_I3C_MSG: {
		struct i3c_msg *msg = data->current_i3c_msg;

		if (LL_I3C_IsActiveFlag_RXLAST(i3c)) {
			data->current_i3c_msg++;
		}

		if (msg->num_xfer < msg->len) {
			msg->buf[msg->num_xfer++] = LL_I3C_ReceiveData8(i3c);
		}

		break;
	}
	case STM32_I2C_MSG: {
		struct i2c_msg *msg = data->current_i2c_msg;

		if (LL_I3C_IsActiveFlag_RXLAST(i3c)) {
			data->current_i2c_msg++;
		}

		if (data->current_i2c_msg_idx < msg->len) {
			msg->buf[data->current_i2c_msg_idx++] = LL_I3C_ReceiveData8(i3c);
		}
		break;
	}
	case STM32_I3C_MSG_DAA: {
		data->pid <<= 8;
		data->pid |= LL_I3C_ReceiveData8(i3c);

		data->daa_rx_rcv++;

		/* After receiving 8 PID bytes from DAA, enable TXFNF interrupt to send the dynamic
		 * address
		 */
		if (data->daa_rx_rcv == 8) {
			LL_I3C_EnableIT_TXFNF(i3c);
			data->daa_rx_rcv = 0;
		}
		break;
	}
	case STM32_I3C_MSG_CCC_P2: {
		struct i3c_ccc_target_payload *target = data->ccc_target_payload;

		if (target->num_xfer < target->data_len) {
			target->data[target->num_xfer++] = LL_I3C_ReceiveData8(i3c);

			/* After receiving all bytes for current target, move on to the next target
			 */
			if (target->num_xfer == target->data_len) {
				data->ccc_target_payload++;
			}
		}
		break;
	}
	default:
		break;
	}
}

static void i3c_stm32_event_isr_cf(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

	switch (data->msg_state) {
	case STM32_I3C_MSG: {
		LL_I3C_ControllerHandleMessage(
			i3c, data->target_addr, data->current_i3c_msg_cf->len,
			((data->current_i3c_msg_cf->flags & I3C_MSG_RW_MASK) == I3C_MSG_READ)
				? LL_I3C_DIRECTION_READ
				: LL_I3C_DIRECTION_WRITE,
			LL_I3C_CONTROLLER_MTYPE_PRIVATE,
			((data->num_msgs > 1) ? LL_I3C_GENERATE_RESTART : LL_I3C_GENERATE_STOP));

		data->current_i3c_msg_cf++;
		data->num_msgs--;
		break;
	}
	case STM32_I2C_MSG: {
		LL_I3C_ControllerHandleMessage(
			i3c, data->target_addr, data->current_i2c_msg->len,
			((data->current_i2c_msg_cf->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ)
				? LL_I3C_DIRECTION_READ
				: LL_I3C_DIRECTION_WRITE,
			LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C,
			((data->num_msgs > 1) ? LL_I3C_GENERATE_RESTART : LL_I3C_GENERATE_STOP));

		data->current_i2c_msg_cf++;
		data->num_msgs--;
		break;
	}
	case STM32_I3C_MSG_CCC:
	case STM32_I3C_MSG_CCC_P2: {
		struct i3c_ccc_payload *payload = data->ccc_payload;
		struct i3c_ccc_target_payload *target;

		if (data->ccc_target_idx < payload->targets.num_targets) {
			target = &payload->targets.payloads[data->ccc_target_idx++];

			LL_I3C_ControllerHandleMessage(
				i3c, target->addr, target->data_len,
				target->rnw ? LL_I3C_DIRECTION_READ : LL_I3C_DIRECTION_WRITE,
				LL_I3C_CONTROLLER_MTYPE_DIRECT,
				(data->ccc_target_idx == payload->targets.num_targets)
					? LL_I3C_GENERATE_STOP
					: LL_I3C_GENERATE_RESTART);

			/* Change state to second part of CCC communication */
			if (data->msg_state == STM32_I3C_MSG_CCC) {
				data->msg_state = STM32_I3C_MSG_CCC_P2;
			}
		}
		break;
	}
	default:
		break;
	}
}

/* Handles the I3C event ISR */
static void i3c_stm32_event_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;

	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

	/* TX FIFO not full handler */
	if (LL_I3C_IsActiveFlag_TXFNF(i3c) && LL_I3C_IsEnabledIT_TXFNF(i3c)) {
		i3c_stm32_event_isr_tx(dev);
	}

	/* RX FIFO not empty handler */
	if (LL_I3C_IsActiveFlag_RXFNE(i3c) && LL_I3C_IsEnabledIT_RXFNE(i3c)) {
		i3c_stm32_event_isr_rx(dev);
	}

	/* Control FIFO not full handler */
	if (LL_I3C_IsActiveFlag_CFNF(i3c) && LL_I3C_IsEnabledIT_CFNF(i3c)) {
		i3c_stm32_event_isr_cf(dev);
	}

	if (LL_I3C_IsActiveFlag_SFNE(i3c) && LL_I3C_IsEnabledIT_SFNE(i3c)) {
		/* Will be read and discarded since there's no real use for it */
		uint32_t status_reg = i3c->SR;

		ARG_UNUSED(status_reg);
	}

	/* Frame complete handler */
	if (LL_I3C_IsActiveFlag_FC(i3c) && LL_I3C_IsEnabledIT_FC(i3c)) {
		LL_I3C_ClearFlag_FC(i3c);
		k_sem_give(&data->device_sync_sem);

#ifdef CONFIG_PM_DEVICE_RUNTIME
		(void)pm_device_runtime_put(dev);
#endif
		pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

		/* Mark bus as idle after each frame complete */
		data->msg_state = STM32_I3C_MSG_IDLE;
	}

#ifdef CONFIG_I3C_USE_IBI

	k_sem_take(&data->ibi_lock_sem, K_FOREVER);

	if (LL_I3C_IsActiveFlag_IBI(i3c)) {
		/* Clear frame complete flag */
		LL_I3C_ClearFlag_IBI(i3c);
		data->ibi_payload = LL_I3C_GetIBIPayload(i3c);
		data->ibi_payload_size = LL_I3C_GetNbIBIAddData(i3c);
		data->ibi_target_addr = LL_I3C_GetIBITargetAddr(i3c);
		if ((data->ibi_payload == 0) && (data->ibi_payload_size == 0) &&
			(data->ibi_target_addr == 0)) {
			LOG_ERR("Invalid Payload\n");
		} else {
			LOG_INF("IBI done, payload received :%d,%d,%d\n", data->ibi_payload,
				data->ibi_payload_size, data->ibi_target_addr);
			if ((data->ibi_payload != 0) && (data->ibi_payload_size != 0)) {
				struct i3c_device_desc *target;
				target = i3c_dev_list_i3c_addr_find(&data->drv_data.attached_dev,
									data->ibi_target_addr);

				if (target != NULL) {
					if (i3c_ibi_work_enqueue_target_irq(target,
									(uint8_t *)&data->ibi_payload,
									data->ibi_payload_size) != 0) {
						LOG_ERR("Error enqueue IBI IRQ work");
					}
				} else {
					LOG_ERR("IBI from unknown device addr 0x%x",
						data->ibi_target_addr);
				}
			}
		}
	}

	if (LL_I3C_IsActiveFlag_HJ(i3c)) {
		int ret;

		LL_I3C_ClearFlag_HJ(i3c);

		ret = i3c_ibi_work_enqueue_hotjoin(dev);
		if (ret != 0) {
			LOG_ERR("IBI Failed to enqueue hotjoin work");
		}
	}

	k_sem_give(&data->ibi_lock_sem);

#endif

	if (LL_I3C_IsActiveFlag_WKP(I3C1)) {
		LL_I3C_ClearFlag_WKP(I3C1);
	}
}

/* Handles the I3C error ISR */
static void i3c_stm32_error_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;

	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

	if (LL_I3C_IsActiveFlag_ANACK(i3c)) {
		LOG_ERR("Address NACK");
	}

	if (LL_I3C_IsActiveFlag_COVR(i3c)) {
		LOG_ERR("Control/Status FIFO underrun/overrun");
	}

	if (LL_I3C_IsActiveFlag_DOVR(i3c)) {
		LOG_ERR("TX/RX FIFO underrun/overrun");
	}

	if (LL_I3C_IsActiveFlag_DNACK(i3c)) {
		LOG_ERR("Data NACK by target");
	}

	if (LL_I3C_IsActiveFlag_PERR(i3c)) {
		switch (LL_I3C_GetMessageErrorCode(i3c)) {
		case LL_I3C_CONTROLLER_ERROR_CE0:
			LOG_ERR("Illegally formatted CCC detected");
			break;
		case LL_I3C_CONTROLLER_ERROR_CE1:
			LOG_ERR("Data on bus is not as expected");
			break;
		case LL_I3C_CONTROLLER_ERROR_CE2:
			LOG_ERR("No response to broadcast address");
			break;
		default:
			LOG_ERR("Unsupported error detected");
			break;
		}
	}

	LL_I3C_ClearFlag_ERR(i3c);

	data->msg_state = STM32_I3C_MSG_ERR;

	k_sem_give(&data->device_sync_sem);

#ifdef CONFIG_PM_DEVICE_RUNTIME
	(void)pm_device_runtime_put(dev);
#endif
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
}

#ifdef CONFIG_I3C_USE_IBI
int i3c_stm32_ibi_enable(const struct device *dev,
			struct i3c_device_desc *target)
{
	int ret = 0;
	uint8_t idx;
	I3C_TypeDef *i3c;
	struct i3c_ccc_events i3c_events;
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;

	i3c = config->i3c;
	if (!i3c_device_is_ibi_capable(target)) {
		return -EINVAL;
	}

	if (data->ibi.num_addr >= ARRAY_SIZE(data->ibi.addr)) {
		/* No more free entries in the IBI table */
		LOG_ERR("%s: no more free space in the IBI table", __func__);
		return -ENOMEM;
	}

	for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
		if (data->ibi.addr[idx] == target->dynamic_addr) {
			LOG_ERR("%s: selected target is already in the list", __func__);
			return -EINVAL;
		}
	}

	if (data->ibi.num_addr > 0) {
		for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
			if (data->ibi.addr[idx] == 0U) {
				break;
			}
		}

		if (idx >= ARRAY_SIZE(data->ibi.addr)) {
			LOG_ERR("Cannot support more IBIs");
			return -ENOTSUP;
		}

	} else {
		idx = 0;
	}

	data->ibi.addr[idx] = target->dynamic_addr;
	data->ibi.num_addr += 1U;

	/* Tell target to enable IBI */
	i3c_events.events = I3C_CCC_EVT_INTR;
	ret = i3c_ccc_do_events_set(target, true, &i3c_events);
	if (ret != 0) {
		LOG_ERR("Error sending IBI ENEC for 0x%02x (%d)",
			target->dynamic_addr, ret);
	}

	if (((target != NULL) && i3c_device_is_ibi_capable(target)) ||
		(((target->bcr & I3C_BCR_IBI_REQUEST_CAPABLE) == I3C_BCR_IBI_REQUEST_CAPABLE))) {
		/* Set I3C bus devices configuration */
		LL_I3C_ConfigDeviceCapabilities(
			i3c, (idx+1), target->dynamic_addr, LL_I3C_IBI_CAPABILITY,
			LL_I3C_IBI_DATA_ENABLE, LL_I3C_CR_NO_CAPABILITY);
	}

	return ret;
}

int i3c_stm32_ibi_disable(const struct device *dev,
			 struct i3c_device_desc *target)
{
	int ret = 0;
	uint8_t idx;
	I3C_TypeDef *i3c;
	struct i3c_ccc_events i3c_events;
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;

	i3c = config->i3c;
	if (!i3c_device_is_ibi_capable(target)) {
		return -EINVAL;
	}

	for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
		if (target->dynamic_addr == data->ibi.addr[idx]) {
			break;
		}
	}

	if (idx == ARRAY_SIZE(data->ibi.addr)) {
		LOG_ERR("%s: target is not in list of registered addresses", __func__);
		return -ENODEV;
	}

	data->ibi.addr[idx] = 0U;
	data->ibi.num_addr -= 1U;

	/* Tell target to disable IBI */
	i3c_events.events = I3C_CCC_EVT_INTR;
	ret = i3c_ccc_do_events_set(target, false, &i3c_events);
	if (ret != 0) {
		LOG_ERR("Error sending IBI DISEC for 0x%02x (%d)",
			target->dynamic_addr, ret);
	}

	if (((target != NULL) && i3c_device_is_ibi_capable(target)) ||
		(((target->bcr & I3C_BCR_IBI_REQUEST_CAPABLE) == I3C_BCR_IBI_REQUEST_CAPABLE))) {
		/* Set I3C bus devices configuration */
		LL_I3C_ConfigDeviceCapabilities(
			i3c, (idx + 1), target->dynamic_addr, LL_I3C_IBI_NO_CAPABILITY,
			LL_I3C_IBI_DATA_DISABLE, LL_I3C_CR_NO_CAPABILITY);
	}

	return ret;
}

#endif /* CONFIG_I3C_USE_IBI */

static const struct i3c_driver_api i3c_stm32_driver_api = {
	.i2c_api.configure = i3c_stm32_i2c_configure,
	.i2c_api.transfer = i3c_stm32_i2c_transfer,
	.configure = i3c_stm32_configure,
	.config_get = i3c_stm32_config_get,
	.i3c_device_find = i3c_stm32_device_find,
	.i3c_xfers = i3c_stm32_transfer,
	.do_daa = i3c_stm32_do_daa,
	.do_ccc = i3c_stm32_do_ccc,
#ifdef CONFIG_I3C_USE_IBI
	.ibi_enable = i3c_stm32_ibi_enable,
	.ibi_disable = i3c_stm32_ibi_disable,
#endif
};

#define STM32_I3C_IRQ_CONNECT_AND_ENABLE(index)                                                    \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, event, irq),                                \
				DT_INST_IRQ_BY_NAME(index, event, priority), i3c_stm32_event_isr,      \
				DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQ_BY_NAME(index, event, irq));                                \
																								   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, error, irq),                                \
				DT_INST_IRQ_BY_NAME(index, error, priority), i3c_stm32_error_isr,      \
				DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQ_BY_NAME(index, error, irq));                                \
	} while (false)

#define STM32_I3C_IRQ_HANDLER_DECL(index)                                                          \
	static void i3c_stm32_irq_config_func_##index(const struct device *dev)

#define STM32_I3C_IRQ_HANDLER_FUNCTION(index) .irq_config_func = i3c_stm32_irq_config_func_##index,

#define STM32_I3C_IRQ_HANDLER(index)                                                               \
	static void i3c_stm32_irq_config_func_##index(const struct device *dev)                    \
	{                                                                                          \
		STM32_I3C_IRQ_CONNECT_AND_ENABLE(index);                                           \
	}

#define I3C_STM32_INIT(index)                                                                      \
	STM32_I3C_IRQ_HANDLER_DECL(index);                                                         \
																								   \
	static const struct stm32_pclken pclken_##index[] = STM32_DT_INST_CLOCKS(index);           \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	static struct i3c_device_desc i3c_stm32_dev_arr_##index[] =                                \
		I3C_DEVICE_ARRAY_DT_INST(index);                                                   \
	static struct i3c_i2c_device_desc i3c_i2c_stm32_dev_arr_##index[] =                        \
		I3C_I2C_DEVICE_ARRAY_DT_INST(index);                                               \
																								   \
	static const struct i3c_stm32_config i3c_stm32_cfg_##index = {                             \
		.i3c = (I3C_TypeDef *)DT_INST_REG_ADDR(index),                                     \
		STM32_I3C_IRQ_HANDLER_FUNCTION(index).pclken = pclken_##index,                     \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.drv_cfg.dev_list.i3c = i3c_stm32_dev_arr_##index,                                 \
		.drv_cfg.dev_list.num_i3c = ARRAY_SIZE(i3c_stm32_dev_arr_##index),                 \
		.drv_cfg.dev_list.i2c = i3c_i2c_stm32_dev_arr_##index,                             \
		.drv_cfg.dev_list.num_i2c = ARRAY_SIZE(i3c_i2c_stm32_dev_arr_##index),             \
	};                                                                                         \
																								   \
	static struct i3c_stm32_data i3c_stm32_data_##index = {                                    \
		.drv_data.ctrl_config.scl.i2c = DT_INST_PROP_OR(index, i2c_scl_hz, 0),             \
		.drv_data.ctrl_config.scl.i3c = DT_INST_PROP_OR(index, i3c_scl_hz, 0),             \
	};                                                                                         \
																								   \
	PM_DEVICE_DT_INST_DEFINE(index, i3c_stm32_pm_action);                                      \
																								   \
	DEVICE_DT_INST_DEFINE(index, &i3c_stm32_init, PM_DEVICE_DT_INST_GET(index),                \
				  &i3c_stm32_data_##index, &i3c_stm32_cfg_##index, POST_KERNEL,        \
				  CONFIG_I3C_INIT_PRIORITY, &i3c_stm32_driver_api);                    \
																								   \
	STM32_I3C_IRQ_HANDLER(index)
DT_INST_FOREACH_STATUS_OKAY(I3C_STM32_INIT)

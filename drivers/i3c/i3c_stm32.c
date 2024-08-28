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
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i3c_stm32, CONFIG_I3C_LOG_LEVEL);

#define DT_DRV_COMPAT st_stm32_i3c

#define CONFIG_I3C_INIT_PRIORITY 50

#define STM32_I3C_TRANSFER_TIMEOUT K_MSEC(100)

typedef void (*irq_config_func_t)(const struct device *port);

enum i3c_stm32_msg_state {
    STM32_I3C_MSG_DAA,    /* Dynamic addressing state */
    STM32_I3C_MSG_CCC,    /* First part of CCC command state*/
    STM32_I3C_MSG_CCC_P2, /* Second part of CCC command state (used for direct commands)*/
    STM32_I3C_MSG_WRITE,  /* Private write state */
    STM32_I3C_MSG_READ,   /* Private read state */
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
    struct k_sem bus_mutex;     /* Sync between device communication messages */
    struct i3c_msg *msg;        /* Current private message */
    uint8_t target_dynamic_addr; /* Current private xfer dynamic address */
    uint64_t pid;                /* Current DAA target PID */
    size_t daa_rx_rcv;           /* Number of RX bytes received during DAA */
    uint8_t target_id;           /* Traget id*/
    uint32_t ibi_payload;        /* Received ibi payload*/
    uint32_t ibi_payload_size;   /* Received payload size*/
    uint32_t ibi_target_addr;    /* Received target dynamic address*/
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

    i3c_stm32_activate(dev);
    ret = i3c_addr_slots_init(dev);
    if (ret != 0) {
        // LOG_ERR("Addr slots init fail %d", ret);
        return ret;
    }

    /* I3C Initialization */
    LL_I3C_SetMode(i3c, LL_I3C_MODE_CONTROLLER);
    LL_I3C_SetDataHoldTime(i3c, LL_I3C_SDA_HOLD_TIME_1_5);
    LL_I3C_SetControllerActivityState(i3c, LL_I3C_OWN_ACTIVITY_STATE_0);
    LL_I3C_ConfigClockWaveForm(i3c, 0x00550908);
    LL_I3C_SetCtrlBusCharacteristic(i3c, 0x102f00ee);
    LL_I3C_EnableHJAck(i3c);
    /* Configure FIFO */
    LL_I3C_SetRxFIFOThreshold(i3c, LL_I3C_RXFIFO_THRESHOLD_1_4);
    LL_I3C_SetTxFIFOThreshold(i3c, LL_I3C_TXFIFO_THRESHOLD_1_4);
    LL_I3C_DisableControlFIFO(i3c);
    LL_I3C_DisableStatusFIFO(i3c);

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

    k_sleep(K_MSEC(1));

    LL_I3C_EnableIT_FC(i3c);
    LL_I3C_EnableIT_CFNF(i3c);
    LL_I3C_EnableIT_RXFNE(i3c);
    LL_I3C_EnableIT_TXFNF(i3c);
    LL_I3C_EnableIT_ERR(i3c);
    LL_I3C_EnableIT_IBI(I3C1);
    LL_I3C_EnableIT_HJ(I3C1);

    /* Bus will be idle initially */
    data->msg_state = STM32_I3C_MSG_IDLE;
    data->target_id = 0;
    data->ibi_payload = 0;
    data->ibi_payload_size = 0;
    data->ibi_target_addr = 0;

    return 0;
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
    LL_I3C_ControllerHandleCCC(
        i3c, payload->ccc.id, payload->ccc.data_len,
        (i3c_ccc_is_payload_broadcast(payload) ? LL_I3C_GENERATE_STOP : LL_I3C_GENERATE_RESTART));

    /* Wait for CCC to complete */
    if(k_sem_take(&data->bus_mutex, STM32_I3C_TRANSFER_TIMEOUT) != 0) {
        return -ETIMEDOUT;
    }

    return 0;
}

/* Handles the ENTDAA CCC */
static int i3c_stm32_do_daa(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;

    /* Mark current transfer as DAA */
    data->msg_state = STM32_I3C_MSG_DAA;

    /* Start DAA */
    LL_I3C_ControllerHandleCCC(i3c, I3C_CCC_ENTDAA, 0, LL_I3C_GENERATE_STOP);

    /* Wait for DAA to finish */
    if (k_sem_take(&data->bus_mutex, STM32_I3C_TRANSFER_TIMEOUT) != 0) {
        return -ETIMEDOUT;
    }

    return 0;
}

/* Handles the controller private read/write transfers */
static int stm32_i3c_transfer(const struct device *dev, struct i3c_device_desc *target,
                  struct i3c_msg *msgs, uint8_t num_msgs)
{
    struct i3c_stm32_data *data = dev->data;
    const struct i3c_stm32_config *config = dev->config;
    I3C_TypeDef *i3c = config->i3c;
    data->target_dynamic_addr = target->dynamic_addr;

    for (size_t i = 0; i < num_msgs; i++) {
        if (msgs[i].buf == NULL) {
            continue;
        }

        data->msg = &msgs[i];

        if ((data->msg->flags & I2C_MSG_RW_MASK) == I3C_MSG_READ) {
            data->msg_state = STM32_I3C_MSG_READ;
        } else {
            data->msg_state = STM32_I3C_MSG_WRITE;
        }

        LL_I3C_RequestTransfer(i3c);

        /* Wait for current transfer to complete */
        if (k_sem_take(&data->bus_mutex, STM32_I3C_TRANSFER_TIMEOUT) != 0) {
            return -ETIMEDOUT;
        }
    }

    return 0;
}

/* Initializes the I3C device and I3C bus */
static int i3c_stm32_init(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;

    k_sem_init(&data->bus_mutex, 0, K_SEM_MAX_LIMIT);

    config->irq_config_func(dev);

    i3c_stm32_configure(dev, I3C_CONFIG_CONTROLLER, NULL);

    /* Start by doing a RSTDAA */
    struct i3c_ccc_payload rstdaa_ccc_payload = {0};
    rstdaa_ccc_payload.ccc.id = I3C_CCC_RSTDAA;

    i3c_stm32_do_ccc(dev, &rstdaa_ccc_payload);

    i3c_stm32_do_daa(dev);
    return 0;
}

/* Handles the TX part of private write */
static void i3c_stm32_event_tx_write(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;

    struct i3c_msg *msg = data->msg;

    if (msg->num_xfer < msg->len) {
        LL_I3C_TransmitData8(i3c, msg->buf[msg->num_xfer++]);
    }
}

/* Handles the TX part of the ENTDAA CCC */
static void i3c_stm32_event_tx_daa(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;

    /* Not all PID bytes arrived, wait until they arrive */
    if (data->daa_rx_rcv != 8) {
        return;
    }

    struct i3c_device_desc *target;
    uint8_t bcr;
    uint8_t dcr;
    uint8_t dyn_addr = 0;
    int ret;

    printf("pid_unmodified=0x%016llx\n", data->pid);

    bcr = (data->pid >> 8) & 0xFF;
    dcr = data->pid & 0xFF;
    data->pid >>= 16;

    /* Find the device in the device list */
    ret = i3c_dev_list_daa_addr_helper(&data->drv_data.attached_dev.addr_slots,
                       &config->drv_cfg.dev_list, data->pid, false, false,
                       &target, &dyn_addr);
    if (ret != 0) {
        /* TODO: Error handling */
        return;
    }

    LL_I3C_TransmitData8(i3c, dyn_addr);
    printf("pid=0x%012llx, bcr=0x%02x, dcr=0x%02x, dyn_addr=0x%02x\n", data->pid, bcr, dcr,
           dyn_addr);

    if (target != NULL) {
        /* Update target descriptor */
        target->dynamic_addr = dyn_addr;
        target->bcr = bcr;
        target->dcr = dcr;
    }

    /* Mark the address as used */
    i3c_addr_slots_mark_i3c(&data->drv_data.attached_dev.addr_slots, dyn_addr);

    /* Mark the static address as free */
    if ((target != NULL) && (target->static_addr != 0) && (dyn_addr != target->static_addr)) {
        i3c_addr_slots_mark_free(&data->drv_data.attached_dev.addr_slots, dyn_addr);
    }

    /* Set I3C bus devices configuration */
    if (((target != NULL) && i3c_device_is_ibi_capable(target)) ||
        (((bcr & I3C_BCR_IBI_REQUEST_CAPABLE) == I3C_BCR_IBI_REQUEST_CAPABLE))) {
        LL_I3C_ConfigDeviceCapabilities(
            i3c, (++data->target_id), dyn_addr, LL_I3C_GET_IBI_CAPABLE(bcr),
            LL_I3C_GET_IBI_PAYLOAD(bcr), LL_I3C_GET_CR_CAPABLE(bcr));
    }
}

/* Handles the TX part of CCC */
static void i3c_stm32_event_tx_ccc(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;
    struct i3c_ccc_payload *payload = data->ccc_payload;

    if (payload->ccc.num_xfer < payload->ccc.data_len) {
        LL_I3C_TransmitData8(i3c, payload->ccc.data[payload->ccc.num_xfer++]);
    }
}

/* Handles the TX 2nd part of direct CCC */
static void i3c_stm32_event_tx_ccc_p2(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;
    struct i3c_ccc_target_payload *target = data->ccc_target_payload;

    if (target->num_xfer < target->data_len) {
        LL_I3C_TransmitData8(i3c, target->data[target->num_xfer++]);

        /* After sending all bytes for current target, move on to the next target */
        if (target->num_xfer == target->data_len) {
            data->ccc_target_payload++;
        }
    }
}

/* Handles the RX part of private read */
static void i3c_stm32_event_rx_read(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;

    struct i3c_msg *msg = data->msg;

    if (msg->num_xfer < msg->len) {
        msg->buf[msg->num_xfer++] = LL_I3C_ReceiveData8(i3c);
    }
}

/* Handles the RX part of ENTDAA CCC */
static void i3c_stm32_event_rx_daa(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;

    /* When RXFNF flag is set and we already received 8 bytes from DAA, it means that we are
     * currently working with a new target */
    if (data->daa_rx_rcv == 8) {
        data->daa_rx_rcv = 0;
    }

    data->pid <<= 8;
    data->pid |= LL_I3C_ReceiveData8(i3c);

    data->daa_rx_rcv++;
}

/* Handles the RX 2nd part of direct CCC */
static void i3c_stm32_event_rx_ccc_p2(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;
    struct i3c_ccc_target_payload *target = data->ccc_target_payload;

    if (target->num_xfer < target->data_len) {
        target->data[target->num_xfer++] = LL_I3C_ReceiveData8(i3c);

        /* After receiving all bytes for current target, move on to the next target */
        if (target->num_xfer == target->data_len) {
            data->ccc_target_payload++;
        }
    }
}

/* Handles the CF part of private transfer */
static void i3c_stm32_event_cf(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;

    uint32_t direction = LL_I3C_DIRECTION_READ;
    if ((data->msg->flags & I3C_MSG_RW_MASK) == I3C_MSG_WRITE) {
        direction = LL_I3C_DIRECTION_WRITE;
    }

    LL_I3C_ControllerHandleMessage(i3c, data->target_dynamic_addr, data->msg->len, direction,
                       LL_I3C_CONTROLLER_MTYPE_PRIVATE, LL_I3C_GENERATE_STOP);
}

/* Handles the CF part of direct CCC */
static void i3c_stm_event_cf_ccc(const struct device *dev)
{
    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;
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
        if (data->msg_state == STM32_I3C_MSG_WRITE) {
            i3c_stm32_event_tx_write(dev);
        } else if (data->msg_state == STM32_I3C_MSG_DAA) {
            i3c_stm32_event_tx_daa(dev);
        } else if (data->msg_state == STM32_I3C_MSG_CCC) {
            i3c_stm32_event_tx_ccc(dev);
        } else if (data->msg_state == STM32_I3C_MSG_CCC_P2) {
            i3c_stm32_event_tx_ccc_p2(dev);
        }
    }

    /* RX FIFO not empty handler */
    if (LL_I3C_IsActiveFlag_RXFNE(i3c) && LL_I3C_IsEnabledIT_RXFNE(i3c)) {
        if (data->msg_state == STM32_I3C_MSG_READ) {
            i3c_stm32_event_rx_read(dev);
        } else if (data->msg_state == STM32_I3C_MSG_DAA) {
            i3c_stm32_event_rx_daa(dev);
        } else if (data->msg_state == STM32_I3C_MSG_CCC_P2) {
            i3c_stm32_event_rx_ccc_p2(dev);
        }
    }

    /* Control FIFO not full handler */
    if (LL_I3C_IsActiveFlag_CFNF(i3c) && LL_I3C_IsEnabledIT_CFNF(i3c)) {
        if (data->msg_state == STM32_I3C_MSG_READ ||
            data->msg_state == STM32_I3C_MSG_WRITE) {
            i3c_stm32_event_cf(dev);
        } else if (data->msg_state == STM32_I3C_MSG_CCC ||
               data->msg_state == STM32_I3C_MSG_CCC_P2) {
            i3c_stm_event_cf_ccc(dev);
        }
    }

    /* Frame complete handler */
    if (LL_I3C_IsActiveFlag_FC(i3c) && LL_I3C_IsEnabledIT_FC(i3c)) {
        LL_I3C_ClearFlag_FC(i3c);
        k_sem_give(&data->bus_mutex);

        /* Mark bus as idle after each frame complete */
        data->msg_state = STM32_I3C_MSG_IDLE;
    }

#ifdef CONFIG_I3C_USE_IBI
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
            struct i3c_device_desc *target = NULL;
            target = i3c_dev_list_i3c_addr_find(&data->drv_data.attached_dev,
                                data->ibi_target_addr);
            if (target != NULL) {
                if (i3c_ibi_work_enqueue_target_irq(target, (uint8_t *) &data->ibi_payload,
                                    data->ibi_payload_size) != 0) {
                    LOG_ERR("Error enqueue IBI IRQ work");
                } else {
                   LOG_ERR("IBI from unknown device addr 0x%x", data->ibi_target_addr ); 
                }
            }
        }
    } else if (LL_I3C_IsActiveFlag_HJ(i3c)) {
        int ret;

        LL_I3C_ClearFlag_HJ(i3c);

        ret = i3c_ibi_work_enqueue_hotjoin(dev);
        if (ret != 0) {
            LOG_ERR("IBI Failed to enqueue hotjoin work");
        }
    }
#endif
}

/* Handles the I3C error ISR */
static void i3c_stm32_error_isr(void *arg)
{
    const struct device *dev = (const struct device *)arg;

    const struct i3c_stm32_config *config = dev->config;
    struct i3c_stm32_data *data = dev->data;
    I3C_TypeDef *i3c = config->i3c;

    ARG_UNUSED(data);

    printf("I3C Status Error Register SER=0x%08x\n", i3c->SER);
    LL_I3C_ClearFlag_ERR(i3c);
    while (1)
        ;
}

static const struct i3c_driver_api i3c_stm32_driver_api = {
    .configure = i3c_stm32_configure,
    .i3c_xfers = stm32_i3c_transfer,
    .do_daa = i3c_stm32_do_daa,
    .do_ccc = i3c_stm32_do_ccc,
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
                                                                                                   \
    static const struct i3c_stm32_config i3c_stm32_cfg_##index = {                             \
        .i3c = (I3C_TypeDef *)DT_INST_REG_ADDR(index),                                     \
        STM32_I3C_IRQ_HANDLER_FUNCTION(index).pclken = pclken_##index,                     \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
        .drv_cfg.dev_list.i3c = i3c_stm32_dev_arr_##index,                                 \
        .drv_cfg.dev_list.num_i3c = ARRAY_SIZE(i3c_stm32_dev_arr_##index),                 \
    };                                                                                         \
                                                                                                   \
    static struct i3c_stm32_data i3c_stm32_data_##index;                                       \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(index, &i3c_stm32_init, NULL, &i3c_stm32_data_##index,               \
                  &i3c_stm32_cfg_##index, POST_KERNEL, CONFIG_I3C_INIT_PRIORITY,       \
                  &i3c_stm32_driver_api);                                              \
                                                                                                   \
    STM32_I3C_IRQ_HANDLER(index)
DT_INST_FOREACH_STATUS_OKAY(I3C_STM32_INIT)

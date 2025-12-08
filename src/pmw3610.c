/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>
#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_LOG_LEVEL);

// ==========================================
// ★ VIPスレッド設定
// ==========================================
// Bluetoothスレッド(Prio -10等)よりも優先される強力な設定
// K_PRIO_COOP(1) = 協調スレッド優先度1 (数字が小さいほど強い)
#define PMW3610_THREAD_PRIORITY K_PRIO_COOP(1)
#define PMW3610_THREAD_STACK_SIZE 1024

//////// Sensor initialization steps definition //////////
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,
    ASYNC_INIT_STEP_CLEAR_OB1,
    ASYNC_INIT_STEP_CHECK_OB1,
    ASYNC_INIT_STEP_CONFIGURE,
    ASYNC_INIT_STEP_COUNT
};

static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10 + CONFIG_PMW3610_INIT_POWER_UP_EXTRA_DELAY_MS,
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200,
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, uint8_t len) {
    const struct pixart_config *cfg = dev->config;
    const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf[] = {
        { .buf = NULL, .len = sizeof(addr), },
        { .buf = value, .len = len, },
    };
    const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
    return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
    return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
    const struct pixart_config *cfg = dev->config;
    uint8_t write_buf[] = {addr | SPI_WRITE_BIT, value};
    const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf), };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1, };
    return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    int err = pmw3610_write_reg(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }
    
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    return 0;
}

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi) {
    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_RES_STEP, &value);
    if (err) {
        LOG_ERR("Can't read res step %d", err);
        return err;
    }
    LOG_INF("Get res step register (reg value 0x%x)", value);

    uint8_t cpi_val = cpi / 200;
    value = (value & 0xE0) | (cpi_val & 0x1F);
    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value, 0x00};

    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    for (size_t i = 0; i < sizeof(data); i++) {
        err = pmw3610_write_reg(dev, addr[i], data[i]);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            break;
        }
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    return 0;
}

static int pmw3610_set_axis(const struct device *dev, bool swap_xy, bool inv_x, bool inv_y) {
    LOG_INF("Setting axis swap_xy: %s inv_x: %s inv_y: %s", 
            swap_xy ? "yes" : "no", inv_x ? "yes" : "no", inv_y ? "yes" : "no");

    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_RES_STEP, &value);
    if (err) {
        LOG_ERR("Can't read res step %d", err);
        return err;
    }
    LOG_INF("Get res step register (reg value 0x%x)", value);

#if IS_ENABLED(CONFIG_PMW3610_SWAP_XY)
    value |= (1 << 7);
#else
    if (swap_xy) { value |= (1 << 7); } else { value &= ~(1 << 7); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_X)
    value |= (1 << 6);
#else
    if (inv_x) { value |= (1 << 6); } else { value &= ~(1 << 6); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_Y)
    value |= (1 << 5);
#else
    if (inv_y) { value |= (1 << 5); } else { value &= ~(1 << 5); }
#endif
    LOG_INF("Setting RES_STEP to (reg value 0x%x)", value);

    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value, 0x00};

    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    for (size_t i = 0; i < sizeof(data); i++) {
        err = pmw3610_write_reg(dev, addr[i], data[i]);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            break;
        }
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    if (err) {
        LOG_ERR("Failed to set axis");
        return err;
    }

    return 0;
}

static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    uint8_t value = sample_time / mintime;
    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

static int pmw3610_set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        maxtime = 8160; 
        mintime = 32;
        break;
    case PMW3610_REG_REST1_DOWNSHIFT:
        maxtime = 255 * 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        break;
    case PMW3610_REG_REST2_DOWNSHIFT:
        maxtime = 255 * 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        break;
    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range (%u - %u)", time, mintime, maxtime);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    uint8_t value = time / mintime;
    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static int pmw3610_set_performance(const struct device *dev, bool enabled) {
    const struct pixart_config *config = dev->config;
    int err = 0;

    if (config->force_awake) {
        uint8_t value;
        err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
        if (err) {
            LOG_ERR("Can't read ref-performance %d", err);
            return err;
        }
        LOG_INF("Get performance register (reg value 0x%x)", value);

        uint8_t perf;
        if (config->force_awake_4ms_mode) {
            perf = 0x0d; 
        } else {
            perf = value & 0x0F; 
        }

        if (enabled) {
            perf |= 0xF0; 
        }
        if (perf != value) {
            err = pmw3610_write(dev, PMW3610_REG_PERFORMANCE, perf);
            if (err) {
                LOG_ERR("Can't write performance register %d", err);
                return err;
            }
            LOG_INF("Set performance register (reg value 0x%x)", perf);
        }
        LOG_INF("%s performance mode", enabled ? "enable" : "disable");
    }

    return err;
}

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

static int pmw3610_async_init_power_up(const struct device *dev) {
    int ret = pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }

    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    uint8_t product_id = 0x01;
    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    int err = 0;
    const struct pixart_config *config = dev->config;

    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = pmw3610_read_reg(dev, reg, buf);
    }

    if (!err) { err = pmw3610_set_performance(dev, true); }
    if (!err) { err = pmw3610_set_cpi(dev, config->cpi); }
    if (!err) { err = pmw3610_set_axis(dev, config->swap_xy, config->inv_x, config->inv_y); }
    if (!err) { err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT, CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS); }
    if (!err) { err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS); }
    if (!err) { err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS); }
    if (!err) { err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, CONFIG_PMW3610_REST1_SAMPLE_TIME_MS); }
    if (!err) { err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, CONFIG_PMW3610_REST2_SAMPLE_TIME_MS); }
    if (!err) { err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, CONFIG_PMW3610_REST3_SAMPLE_TIME_MS); }

    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

    return 0;
}

// -----------------------------------------------------------------------------
// ★ 専用スレッド実装 (static最適化版)
// -----------------------------------------------------------------------------

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    
    // データ受け取り用バッファ
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) { return -EBUSY; }

    // ★ 高速化ポイント: SPI通信の準備を static 化して、
    // 毎回のメモリ初期化コスト(CPUサイクル)を削減
    static uint8_t burst_addr = PMW3610_REG_MOTION_BURST;
    static const struct spi_buf tx_buf = { .buf = &burst_addr, .len = 1 };
    static const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    
    struct spi_buf rx_buf[] = {
        { .buf = NULL, .len = 1 },
        { .buf = buf, .len = PMW3610_BURST_SIZE }
    };
    const struct spi_buf_set rx = { .buffers = rx_buf, .count = 2 };

    // SPI通信実行
    int err = spi_transceive_dt(&config->spi, &tx, &rx);
    if (err) { return err; }

#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

    int16_t x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
    int16_t y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);

    // スマートアルゴリズムがOFFなら、ここでの条件分岐も無駄なので#ifdefで囲って消滅させます
#ifdef CONFIG_PMW3610_SMART_ALGORITHM
    if (data->sw_smart_flag) {
        // ... (スマートアルゴリズム有効時の処理)
        int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) + buf[PMW3610_SHUTTER_L_POS];
        if (data->sw_smart_flag && shutter < 45) {
            pmw3610_write(dev, 0x32, 0x00);
            data->sw_smart_flag = false;
        }
        if (!data->sw_smart_flag && shutter > 45) {
            pmw3610_write(dev, 0x32, 0x80);
            data->sw_smart_flag = true;
        }
    }
#endif

    // 前回の差分クリアなどの処理 (REPORT_INTERVAL_MIN使用時)
#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
    static int64_t last_smp_time = 0;
    static int64_t last_rpt_time = 0;
    static int64_t dx = 0;
    static int64_t dy = 0;
    int64_t now = k_uptime_get();

    if (now - last_smp_time >= CONFIG_PMW3610_REPORT_INTERVAL_MIN) {
        dx = 0; dy = 0;
    }
    last_smp_time = now;
    
    dx += x;
    dy += y;

    if (now - last_rpt_time < CONFIG_PMW3610_REPORT_INTERVAL_MIN) { return 0; }
    last_rpt_time = now;
#else
    // インターバル制限なしの場合 (通常はこちらが速い)
    int16_t dx = x;
    int16_t dy = y;
#endif

    int16_t rx = (int16_t)CLAMP(dx, INT16_MIN, INT16_MAX);
    int16_t ry = (int16_t)CLAMP(dy, INT16_MIN, INT16_MAX);
    bool have_x = rx != 0;
    bool have_y = ry != 0;

    if (have_x || have_y) {
#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
        dx = 0; dy = 0;
#endif
        if (have_x) { input_report(dev, config->evt_type, config->x_input_code, rx, !have_y, K_NO_WAIT); }
        if (have_y) { input_report(dev, config->evt_type, config->y_input_code, ry, true, K_NO_WAIT); }
    }
    return err;
}

// 割り込みハンドラ：システムワークキューを使わず、セマフォを渡すだけ(爆速)
static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    
    pmw3610_set_interrupt(dev, false);
    // システムワークキュー(k_work_submit)は使わない！
    // 専用スレッドに合図を送る
    k_sem_give(&data->gpio_sem);
}

// 専用スレッド本体
static void pmw3610_thread(void *p1, void *p2, void *p3) {
    const struct device *dev = (const struct device *)p1;
    struct pixart_data *data = dev->data;

    while (true) {
        // 1. 合図が来るまで寝る
        k_sem_take(&data->gpio_sem, K_FOREVER);

        // 2. 最優先でデータを読む
        pmw3610_report_data(dev);

        // 3. 割り込みを再開
        pmw3610_set_interrupt(dev, true);
    }
}

// -----------------------------------------------------------------------------

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("PMW3610 initialization failed in step %d", data->async_init_step);
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true;
            LOG_INF("PMW3610 initialized");
            pmw3610_set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

static int pmw3610_init_irq(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    return err;
}

static int pmw3610_init(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("%s is not ready", config->spi.bus->name);
		return -ENODEV;
	}

    data->dev = dev;
    data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
    data->ready = false;

    // ★ スレッドとセマフォの初期化
    k_sem_init(&data->gpio_sem, 0, 1);
    
    k_thread_create(&data->thread, data->thread_stack,
                    PMW3610_THREAD_STACK_SIZE,
                    pmw3610_thread,
                    (void *)dev, NULL, NULL,
                    PMW3610_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&data->thread, "pmw3610");

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
    data->sw_smart_flag = false;
#endif

    k_work_init_delayable(&data->init_work, pmw3610_async_init);

    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    k_work_schedule(&data->init_work, K_MSEC(10));

    return 0;
}

#define PMW3610_DEFINE(n)                                                                          \
    static struct pixart_data pmw3610_data_##n;                                                    \
    static const struct pixart_config pmw3610_config_##n = {                                       \
        .spi = SPI_DT_SPEC_INST_GET(n, SPI_WORD_SET(8) | SPI_MODE_GET(0), 0),                      \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .evt_type = DT_INST_PROP_OR(n, evt_type, 0),                                               \
        .x_input_code = DT_INST_PROP_OR(n, x_input_code, 0),                                       \
        .y_input_code = DT_INST_PROP_OR(n, y_input_code, 0),                                       \
        .cpi = DT_INST_PROP_OR(n, cpi, 0),                                                         \
        .swap_xy = DT_INST_PROP(n, swap_xy),                                                       \
        .inv_x = DT_INST_PROP(n, invert_x),                                                        \
        .inv_y = DT_INST_PROP(n, invert_y),                                                        \
        .force_awake = DT_INST_PROP(n, force_awake),                                               \
        .force_awake_4ms_mode = DT_INST_PROP(n, force_awake_4ms_mode),                             \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &pmw3610_data_##n, &pmw3610_config_##n,           \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)

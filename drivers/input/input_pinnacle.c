#define DT_DRV_COMPAT cirque_pinnacle

#include <string.h>

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>

#include "input_pinnacle.h"

LOG_MODULE_REGISTER(pinnacle, CONFIG_INPUT_LOG_LEVEL);

/* Scaled space is 0–1024; centre is (512, 512).  All gesture thresholds are
 * derived at runtime from pinnacle_config so they can be tuned via DTS. */
#define PAD_CENTER 512

/* ── Helpers ───────────────────────────────────────────────────────────────── */

static int32_t dist_sq_from_center(int16_t x, int16_t y) {
    int32_t dx = x - PAD_CENTER;
    int32_t dy = y - PAD_CENTER;
    return dx * dx + dy * dy;
}

static void pinnacle_scale_abs(const struct pinnacle_config *config,
                                int16_t raw_x, int16_t raw_y,
                                int16_t *sx, int16_t *sy) {
    int16_t x = raw_x, y = raw_y;
    if (x < config->absolute_mode_clamp_min_x) x = config->absolute_mode_clamp_min_x;
    else if (x > config->absolute_mode_clamp_max_x) x = config->absolute_mode_clamp_max_x;
    if (y < config->absolute_mode_clamp_min_y) y = config->absolute_mode_clamp_min_y;
    else if (y > config->absolute_mode_clamp_max_y) y = config->absolute_mode_clamp_max_y;
    *sx = (int16_t)((int32_t)(x - config->absolute_mode_clamp_min_x)
                    * config->absolute_mode_scale_to_width
                    / (config->absolute_mode_clamp_max_x - config->absolute_mode_clamp_min_x));
    *sy = (int16_t)((int32_t)(y - config->absolute_mode_clamp_min_y)
                    * config->absolute_mode_scale_to_height
                    / (config->absolute_mode_clamp_max_y - config->absolute_mode_clamp_min_y));
}

/* Integer atan2 returning signed int16; full circle = 65536 units.
 * Adapted from QMK cirque_pinnacle_gestures.c (GPL-2.0). */
static int16_t atan2_16(int32_t dy, int32_t dx) {
    if (dy == 0) {
        return (dx >= 0) ? 0 : 32767;
    }
    int32_t abs_y = (dy > 0) ? dy : -dy;
    int16_t a;
    if (dx >= 0) {
        a = (int16_t)(8192 - (int32_t)8192 * (dx - abs_y) / (dx + abs_y));
    } else {
        a = (int16_t)(24576 - (int32_t)8192 * (dx + abs_y) / (abs_y - dx));
    }
    return (dy < 0) ? (int16_t)(-a) : a;
}

static int pinnacle_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                             const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    return config->seq_read(dev, addr, buf, len);
}
static int pinnacle_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    return config->write(dev, addr, val);
}

// Now that we are counting ZIDLEs it would be very bad to miss one.  But in my testing I see that happen (rarely - once every
// couple of days of usage).  The fact that the current irq system is edge triggered probably isn't great for this reason.
// But for now just have the touch controller emit NUM_ZIDLE_PAD extra idles
#define NUM_ZIDLE  3
#define NUM_ZIDLE_PAD 2

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int pinnacle_i2c_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    return i2c_burst_read_dt(&config->bus.i2c, PINNACLE_READ | addr, buf, len);
}

static int pinnacle_i2c_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    return i2c_reg_write_byte_dt(&config->bus.i2c, PINNACLE_WRITE | addr, val);
}

#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int pinnacle_spi_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    uint8_t tx_buffer[len + 3], rx_dummy[3];
    tx_buffer[0] = PINNACLE_READ | addr;
    memset(&tx_buffer[1], PINNACLE_AUTOINC, len + 2);

    const struct spi_buf tx_buf[2] = {
        {
            .buf = tx_buffer,
            .len = len + 3,
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = 1,
    };
    struct spi_buf rx_buf[2] = {
        {
            .buf = rx_dummy,
            .len = 3,
        },
        {
            .buf = buf,
            .len = len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = 2,
    };
    int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    return ret;
}

static int pinnacle_spi_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    uint8_t tx_buffer[2] = {PINNACLE_WRITE | addr, val};
    uint8_t rx_buffer[2];

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = 2,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = 2,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    const int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    if (ret < 0) {
        LOG_ERR("spi ret: %d", ret);
    }

    if (rx_buffer[1] != PINNACLE_FILLER) {
        LOG_ERR("bad ret val %d - %d", rx_buffer[0], rx_buffer[1]);
        return -EIO;
    }

    k_usleep(50);

    return ret;
}
#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int set_int(const struct device *dev, const bool en) {
    const struct pinnacle_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }

    return ret;
}

static int pinnacle_clear_status(const struct device *dev) {
    int ret = pinnacle_write(dev, PINNACLE_STATUS1, 0);
    if (ret < 0) {
        LOG_ERR("Failed to clear STATUS1 register: %d", ret);
    }

    return ret;
}

static int pinnacle_era_read(const struct device *dev, const uint16_t addr, uint8_t *val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_READ);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_VALUE, val, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read ERA value (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static int pinnacle_era_write(const struct device *dev, const uint16_t addr, uint8_t val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_VALUE, val);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA value (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = pinnacle_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static void pinnacle_send_rel(const struct device *dev, int8_t dx, int8_t dy) {
    const struct pinnacle_config *config = dev->config;
    struct pinnacle_data *data = dev->data;

    pinnacle_clear_status(dev);
    set_int(dev, true);

    bool must_send = false;

    uint8_t btn = data->last_btn;
    if (!config->no_taps && (btn || data->btn_cache)) {
        for (int i = 0; i < 3; i++) {
            uint8_t btn_val = btn & BIT(i);
            if (btn_val != (data->btn_cache & BIT(i))) {
                input_report_key(dev, INPUT_BTN_0 + i, btn_val ? 1 : 0, false, K_FOREVER);
                must_send = true;
            }
        }
    }

    data->btn_cache = btn;
    bool is_touching = (data->last_z > 0);
    bool touch_changed = false;
    if(is_touching) {
        must_send = true;

        if(data->num_z_idle > 0) { // we just recently had z idles
            touch_changed = true;
            data->num_z_idle = 0;
            dx = 0;
            dy = 0; // starting a new press, must reset deltas
        }
    } else {
        data->num_z_idle++;
        if(data->num_z_idle == NUM_ZIDLE) {
            touch_changed = true;
        }
        dx = 0;
        dy = 0;
    }

    LOG_DBG("Rel move: touch_changed=%d z=%d dx=%d dy=%d", touch_changed, data->last_z, dx, dy);

    if(touch_changed)
    {
        // Finalize the input event only if we have something to report
        input_report_key(dev, INPUT_BTN_TOUCH, is_touching ? 1 : 0, false, K_FOREVER);
        must_send = true;
    }

    if(must_send) {
        input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER); 
    }
}

static void pinnacle_send_abs(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    struct pinnacle_data *data = dev->data;
    const struct pinnacle_gesture_params *params = &data->gesture_params;

    /* Derive gesture thresholds from mutable runtime params. */
    int32_t scroll_rim_r  = (int32_t)PAD_CENTER * params->scroll_rim_percent / 100;
    int32_t scroll_rim_sq = scroll_rim_r * scroll_rim_r;
    int32_t dj_rim_r      = (int32_t)PAD_CENTER * params->drag_jump_rim_percent / 100;
    int32_t dj_rim_sq     = dj_rim_r * dj_rim_r;
    int32_t dead_r  = (int32_t)PAD_CENTER * params->dead_radius_percent / 100;
    int32_t dead_sq = dead_r * dead_r;
    int16_t rclick_x_min = (int16_t)(1024 * params->rclick_x_min_percent / 100);

    pinnacle_clear_status(dev);
    set_int(dev, true);

    bool is_touching = (data->last_z > 0);

    /* Update z-idle counter; detect new_contact and lift edges. */
    bool new_contact = false;
    bool lift = false;

    if (is_touching) {
        if (data->num_z_idle > 0) {
            new_contact = true;
            data->num_z_idle = 0;
        }
    } else {
        data->num_z_idle++;
        if (data->num_z_idle == NUM_ZIDLE) {
            lift = true;
        }
    }

    /* Scale coordinates (only valid when touching). */
    int16_t x = 0, y = 0;
    if (is_touching) {
        pinnacle_scale_abs(config, data->last_x, data->last_y, &x, &y);
        LOG_DBG("abs: x=%d y=%d z=%d state=%d", x, y, data->last_z, data->state);
    } else {
        LOG_DBG("abs: z=0 num_z_idle=%d state=%d", data->num_z_idle, data->state);
    }

    switch (data->state) {

    case PINNACLE_STATE_INACTIVE:
        if (new_contact) {
            /* Cancel any pending deferred PAD-off from the previous gesture. */
            k_work_cancel_delayable(&data->pad_off_work);
            int32_t d2 = dist_sq_from_center(x, y);
            /* Scroll exclusion band: centred y-band blocks scroll initiation. */
            int16_t excl_half = (int16_t)((int32_t)1024
                                          * params->scroll_exclusion_zone_percent / 100 / 2);
            bool in_excl_band = (y > (PAD_CENTER - excl_half)
                                 && y < (PAD_CENTER + excl_half));
            if (params->scroll_enable && d2 > scroll_rim_sq && (x - PAD_CENTER) > 0 && !in_excl_band) {
                /* Rim zone (left half only, outside exclusion band) → SCROLL_ACTIVE */
                data->scroll_ref_x = x;
                data->scroll_ref_y = y;
                data->scroll_clicks_rem = 0;
                data->scroll_direction = (y - PAD_CENTER) > 0
                                         ? PINNACLE_SCROLL_HORIZONTAL
                                         : PINNACLE_SCROLL_VERTICAL;
                data->state = PINNACLE_STATE_SCROLL_ACTIVE;
                LOG_INF("gesture: INACTIVE→SCROLL_ACTIVE dir=%d", data->scroll_direction);
            } else if (params->tap_enable) {
                /* Inner, right-click, or exclusion-band zone → TAP_PENDING */
                data->is_left = params->rclick_enable ? (x > rclick_x_min) : true;
                data->touch_start_x = x;
                data->touch_start_y = y;
                data->prev_scaled_x = x;
                data->prev_scaled_y = y;
                data->state = PINNACLE_STATE_TAP_PENDING;
                k_work_schedule(&data->tap_timeout_work,
                                K_MSEC(params->tap_timeout_ms));
                LOG_INF("gesture: INACTIVE→TAP_PENDING is_left=%d", data->is_left);
            } else {
                /* Tap disabled → MOVING immediately */
                data->touch_start_x = x;
                data->touch_start_y = y;
                data->prev_scaled_x = x;
                data->prev_scaled_y = y;
                data->state = PINNACLE_STATE_MOVING;
                LOG_INF("gesture: INACTIVE→MOVING (tap disabled)");
            }
            input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);
        }
        break;

    case PINNACLE_STATE_TAP_PENDING:
        /* Hard press: skip tap timeout and jump straight to DRAGGING. */
        if (params->drag_enable && is_touching && data->last_z >= params->force_drag_z_threshold) {
            k_work_cancel_delayable(&data->tap_timeout_work);
            uint16_t btn_tp = data->is_left ? INPUT_BTN_0 : INPUT_BTN_1;
            input_report_key(dev, btn_tp, 1, true, K_FOREVER);
            data->prev_scaled_x = x;
            data->prev_scaled_y = y;
            data->state = PINNACLE_STATE_DRAGGING;
            LOG_INF("gesture: TAP_PENDING→DRAGGING (hard press z=%d)", data->last_z);
        }
        /* Otherwise movement suppressed; transitions driven by tap_timeout_work. */
        break;

    case PINNACLE_STATE_MOVING:
        if (lift) {
            data->state = PINNACLE_STATE_INACTIVE;
            k_work_schedule(&data->pad_off_work,
                            K_MSEC(params->pad_off_timeout_ms));
            LOG_INF("gesture: MOVING→INACTIVE");
        } else if (is_touching) {
            if (params->drag_enable && data->last_z >= params->force_drag_z_threshold) {
                /* Hard press while moving → DRAGGING, always left button. */
                data->is_left = true;
                data->prev_scaled_x = x;
                data->prev_scaled_y = y;
                input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
                data->state = PINNACLE_STATE_DRAGGING;
                LOG_INF("gesture: MOVING→DRAGGING (hard press z=%d)", data->last_z);
            } else {
                int16_t dx = x - data->prev_scaled_x;
                int16_t dy = y - data->prev_scaled_y;
                data->prev_scaled_x = x;
                data->prev_scaled_y = y;
                if (dx != 0 || dy != 0) {
                    input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
                    input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
                }
            }
        }
        break;

    case PINNACLE_STATE_DRAG_WINDOW:
        if (new_contact) {
            k_work_cancel_delayable(&data->drag_window_work);
            if (data->last_z >= params->double_click_drag_z_threshold) {
                /* Firm contact — begin drag. Wait for drag_pending_timeout to emit movement. */
                data->touch_start_x = x;
                data->touch_start_y = y;
                data->prev_scaled_x = x;
                data->prev_scaled_y = y;
                data->state = PINNACLE_STATE_DRAGGING_PENDING;
                k_work_schedule(&data->drag_pending_timeout_work,
                                K_MSEC(params->drag_pending_timeout_ms));
                
                /* Release the first tap's click before starting the second gesture */
                uint16_t btn = data->is_left ? INPUT_BTN_0 : INPUT_BTN_1;
                input_report_key(dev, btn, 0, true, K_FOREVER);
                
                LOG_INF("gesture: DRAG_WINDOW→DRAGGING_PENDING (z=%d)", data->last_z);
            } else {
                /* Light contact — cancel drag, treat as new gesture.
                 * Button already released. PAD stays ON. */
                data->is_left = params->rclick_enable ? (x > rclick_x_min) : true;
                data->touch_start_x = x;
                data->touch_start_y = y;
                data->prev_scaled_x = x;
                data->prev_scaled_y = y;
                k_work_cancel_delayable(&data->pad_off_work);
                k_work_schedule(&data->tap_timeout_work,
                                K_MSEC(params->tap_timeout_ms));
                data->state = PINNACLE_STATE_TAP_PENDING;
                LOG_INF("gesture: DRAG_WINDOW→TAP_PENDING (light touch)");
            }
        }
        break;

    case PINNACLE_STATE_DRAGGING_PENDING:
        if (lift) {
            /* Finger lifted before drag_pending_timeout_ms (Double Tap). */
            k_work_cancel_delayable(&data->drag_pending_timeout_work);
            uint16_t btn = data->is_left ? INPUT_BTN_0 : INPUT_BTN_1;
            input_report_key(dev, btn, 1, true, K_FOREVER);
            k_work_schedule(&data->tap_click_work, K_MSEC(60));
            data->state = PINNACLE_STATE_INACTIVE;
            k_work_schedule(&data->pad_off_work,
                            K_MSEC(params->pad_off_timeout_ms));
            LOG_INF("gesture: DRAGGING_PENDING→INACTIVE (double tap)");
        }
        /* Otherwise movement suppressed; transitions driven by drag_pending_timeout_work. */
        break;

    case PINNACLE_STATE_DRAGGING:
        if (lift) {
            /* Only enter DRAG_JUMP if the last position was near the rim (user
             * repositioning across the pad). A centre lift ends the drag. */
            int32_t d2_lift = dist_sq_from_center(data->prev_scaled_x, data->prev_scaled_y);
            if (d2_lift > dj_rim_sq) {
                data->state = PINNACLE_STATE_DRAG_JUMP;
                k_work_schedule(&data->drag_jump_work,
                                K_MSEC(params->drag_jump_timeout_ms));
                LOG_INF("gesture: DRAGGING→DRAG_JUMP (at rim)");
            } else {
                uint16_t btn = data->is_left ? INPUT_BTN_0 : INPUT_BTN_1;
                LOG_INF("gesture: emit btn=0 from %s", __func__); input_report_key(dev, btn, 0, true, K_FOREVER);
                data->state = PINNACLE_STATE_INACTIVE;
                k_work_schedule(&data->pad_off_work,
                                K_MSEC(params->pad_off_timeout_ms));
                LOG_INF("gesture: DRAGGING→INACTIVE (not at rim)");
            }
        } else if (is_touching) {
            int16_t dx = x - data->prev_scaled_x;
            int16_t dy = y - data->prev_scaled_y;
            data->prev_scaled_x = x;
            data->prev_scaled_y = y;
            if (dx != 0 || dy != 0) {
                input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
            }
        }
        break;

    case PINNACLE_STATE_DRAG_JUMP:
        if (new_contact) {
            k_work_cancel_delayable(&data->drag_jump_work);
            data->prev_scaled_x = x;
            data->prev_scaled_y = y;
            data->state = PINNACLE_STATE_DRAGGING;
            LOG_INF("gesture: DRAG_JUMP→DRAGGING");
        }
        break;

    case PINNACLE_STATE_SCROLL_ACTIVE:
        if (lift) {
            data->scroll_clicks_rem = 0;
            data->state = PINNACLE_STATE_INACTIVE;
            k_work_schedule(&data->pad_off_work,
                            K_MSEC(params->pad_off_timeout_ms));
            LOG_INF("gesture: SCROLL_ACTIVE→INACTIVE");
        } else if (is_touching) {
            int32_t d2 = dist_sq_from_center(x, y);
            if (d2 < dead_sq) {
                data->state = PINNACLE_STATE_SCROLL_DEAD;
                LOG_INF("gesture: SCROLL_ACTIVE→SCROLL_DEAD");
            } else {
                /* Angular delta from reference position (both centered at origin). */
                int32_t rx = data->scroll_ref_x - PAD_CENTER;
                int32_t ry = data->scroll_ref_y - PAD_CENTER;
                int32_t cx = x - PAD_CENTER;
                int32_t cy = y - PAD_CENTER;
                int32_t dot = rx * cx + ry * cy;
                int32_t det = rx * cy - ry * cx;
                int16_t ang = atan2_16(det, dot);
                data->scroll_clicks_rem += (int32_t)ang * params->wheel_clicks;
                int32_t whole_clicks = data->scroll_clicks_rem / 65536;
                data->scroll_clicks_rem -= whole_clicks * 65536;
                if (whole_clicks != 0) {
                    uint16_t axis = (data->scroll_direction == PINNACLE_SCROLL_HORIZONTAL)
                                    ? INPUT_REL_HWHEEL : INPUT_REL_WHEEL;
                    input_report_rel(dev, axis, (int32_t)whole_clicks, true, K_FOREVER);
                    LOG_DBG("scroll: dir=%d clicks=%d", data->scroll_direction, (int)whole_clicks);
                }
                data->scroll_ref_x = x;
                data->scroll_ref_y = y;
            }
        }
        break;

    case PINNACLE_STATE_SCROLL_DEAD:
        if (lift) {
            data->state = PINNACLE_STATE_INACTIVE;
            k_work_schedule(&data->pad_off_work,
                            K_MSEC(params->pad_off_timeout_ms));
            LOG_INF("gesture: SCROLL_DEAD→INACTIVE");
        } else if (is_touching) {
            int32_t d2 = dist_sq_from_center(x, y);
            if (d2 >= dead_sq) {
                /* Exited dead zone — reset reference and resume scrolling. */
                data->scroll_ref_x = x;
                data->scroll_ref_y = y;
                data->state = PINNACLE_STATE_SCROLL_ACTIVE;
                LOG_INF("gesture: SCROLL_DEAD→SCROLL_ACTIVE");
            }
        }
        break;
    }
}

/* ── Gesture timer callbacks ──────────────────────────────────────────────── */

static void tap_click_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pinnacle_data *data = CONTAINER_OF(dwork, struct pinnacle_data, tap_click_work);
    const struct device *dev = data->dev;
    uint16_t btn = data->is_left ? INPUT_BTN_0 : INPUT_BTN_1;
    LOG_INF("gesture: emit btn=0 from %s", __func__); input_report_key(dev, btn, 0, true, K_FOREVER);
}

static void tap_timeout_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pinnacle_data *data = CONTAINER_OF(dwork, struct pinnacle_data, tap_timeout_work);
    const struct device *dev = data->dev;
    const struct pinnacle_config *config = dev->config;

    if (data->state != PINNACLE_STATE_TAP_PENDING) {
        return;
    }

    if (data->last_z > 0) {
        /* Finger still down → MOVING.
         * If tap-snap is enabled, snap prev_scaled to the current position so
         * movement starts from zero. Otherwise keep prev_scaled at touch_start
         * so the first MOVING packet emits the delta accumulated while pending. */
        if (data->gesture_params.tap_snap) {
            int16_t sx, sy;
            pinnacle_scale_abs(config, data->last_x, data->last_y, &sx, &sy);
            data->prev_scaled_x = sx;
            data->prev_scaled_y = sy;
        }
        data->state = PINNACLE_STATE_MOVING;
        LOG_INF("gesture: TAP_PENDING→MOVING snap=%d", data->gesture_params.tap_snap);
    } else {
        /* Finger lifted → button down, then schedule up to survive BLE batching. */
        uint16_t btn = data->is_left ? INPUT_BTN_0 : INPUT_BTN_1;
        input_report_key(dev, btn, 1, true, K_FOREVER);
        k_work_schedule(&data->tap_click_work, K_MSEC(60));
        if (data->gesture_params.drag_enable) {
            k_work_schedule(&data->drag_window_work,
                            K_MSEC(data->gesture_params.drag_window_timeout_ms));
            data->state = PINNACLE_STATE_DRAG_WINDOW;
            LOG_INF("gesture: TAP_PENDING→DRAG_WINDOW btn=%d", btn);
        } else {
            data->state = PINNACLE_STATE_INACTIVE;
            k_work_schedule(&data->pad_off_work, K_MSEC(data->gesture_params.pad_off_timeout_ms));
            LOG_INF("gesture: TAP_PENDING→INACTIVE btn=%d", btn);
        }
    }
}

static void drag_pending_timeout_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pinnacle_data *data = CONTAINER_OF(dwork, struct pinnacle_data, drag_pending_timeout_work);
    const struct device *dev = data->dev;
    const struct pinnacle_config *config = dev->config;

    if (data->state != PINNACLE_STATE_DRAGGING_PENDING) {
        return;
    }

    if (data->last_z > 0) {
        /* Finger held down past timeout → begin actual DRAGGING.
         * Match tap_timeout_cb logic: if tap-snap is enabled, snap prev_scaled to the
         * current position so movement starts from zero. Otherwise emit accumulated delta. */
        if (data->gesture_params.tap_snap) {
            int16_t sx, sy;
            pinnacle_scale_abs(config, data->last_x, data->last_y, &sx, &sy);
            data->prev_scaled_x = sx;
            data->prev_scaled_y = sy;
        }
        data->state = PINNACLE_STATE_DRAGGING;
        uint16_t btn = data->is_left ? INPUT_BTN_0 : INPUT_BTN_1;
        input_report_key(dev, btn, 1, true, K_FOREVER);
        LOG_INF("gesture: DRAGGING_PENDING→DRAGGING snap=%d (emit btn=1)", data->gesture_params.tap_snap);
    }
}

static void pad_off_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pinnacle_data *data = CONTAINER_OF(dwork, struct pinnacle_data, pad_off_work);
    const struct device *dev = data->dev;

    if (data->state != PINNACLE_STATE_INACTIVE) {
        return;
    }
    input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
    LOG_INF("gesture: PAD OFF (deferred)");
}

static void drag_window_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pinnacle_data *data = CONTAINER_OF(dwork, struct pinnacle_data, drag_window_work);
    const struct device *dev = data->dev;
    if (data->state != PINNACLE_STATE_DRAG_WINDOW) {
        return;
    }

    /* Drag window expired: button already released, just schedule deferred PAD-off. */
    data->state = PINNACLE_STATE_INACTIVE;
    k_work_schedule(&data->pad_off_work, K_MSEC(data->gesture_params.pad_off_timeout_ms));
    LOG_INF("gesture: DRAG_WINDOW→INACTIVE (timeout)");
}

static void drag_jump_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pinnacle_data *data = CONTAINER_OF(dwork, struct pinnacle_data, drag_jump_work);
    const struct device *dev = data->dev;
    if (data->state != PINNACLE_STATE_DRAG_JUMP) {
        return;
    }

    /* Same sync reasoning as drag_window_cb. */
    uint16_t btn = data->is_left ? INPUT_BTN_0 : INPUT_BTN_1;
    LOG_INF("gesture: emit btn=0 from %s", __func__); input_report_key(dev, btn, 0, true, K_FOREVER);
    data->state = PINNACLE_STATE_INACTIVE;
    k_work_schedule(&data->pad_off_work, K_MSEC(data->gesture_params.pad_off_timeout_ms));
    LOG_INF("gesture: DRAG_JUMP→INACTIVE (timeout)");
}

static int pinnacle_read_abs(const struct device *dev) {
    uint8_t packet[6];
    int ret;
    ret = pinnacle_seq_read(dev, PINNACLE_STATUS1, packet, 1);
    if (ret < 0) {
        LOG_ERR("read status: %d", ret);
        return ret;
    }
    if (!(packet[0] & PINNACLE_STATUS1_SW_DR)) {
        return -1;
    }
    ret = pinnacle_seq_read(dev, PINNACLE_2_2_PACKET0, packet, 6);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return ret;
    }
    struct pinnacle_data *data = dev->data;
    // TODO: Enable SW3-SW5 as well
    data->last_btn = packet[0] &
                  (PINNACLE_PACKET0_BTN_PRIM | PINNACLE_PACKET0_BTN_SEC | PINNACLE_PACKET0_BTN_AUX);
    uint8_t x_low = packet[2];
    uint8_t y_low = packet[3];
    uint8_t xy_high = packet[4];
    data->last_x = ((xy_high & 0x0F) << 8) | x_low;
    data->last_y = ((xy_high & 0xF0) << 4) | y_low;
    data->last_z = (uint8_t)(packet[5] & 0x1F);

    LOG_DBG("abs pkt: btn=%d x=%d y=%d z=%d", data->last_btn, data->last_x, data->last_y, data->last_z);
    return 0;
}

static void pinnacle_report_data_abs(const struct device *dev) {
    int ret = pinnacle_read_abs(dev);
    if (ret == 0) {
        pinnacle_send_abs(dev);
    }
}

static void pinnacle_report_data_abs_rel(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    int16_t old_x = data->last_x;
    int16_t old_y = data->last_y;

    int ret = pinnacle_read_abs(dev);

    if (ret == 0) {
        int16_t dx = data->last_x - old_x;
        int16_t dy = data->last_y - old_y;
        const struct pinnacle_config *config = dev->config;

        dx /= config->abs_rel_divisor;
        dy /= config->abs_rel_divisor;

        pinnacle_send_rel(dev, (int8_t) dx, (int8_t) dy);
    }
}

static void pinnacle_report_data_rel(const struct device *dev) {
    uint8_t packet[3];
    int ret;
    ret = pinnacle_seq_read(dev, PINNACLE_STATUS1, packet, 1);
    if (ret < 0) {
        LOG_ERR("read status: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 1, "Pinnacle Status1");

    // Ignore 0xFF packets that indicate communcation failure, or if SW_DR isn't asserted
    if (packet[0] == 0xFF || !(packet[0] & PINNACLE_STATUS1_SW_DR)) {
        return;
    }
    ret = pinnacle_seq_read(dev, PINNACLE_2_2_PACKET0, packet, 3);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 3, "Pinnacle Packets");

    struct pinnacle_data *data = dev->data;
    data->last_btn = packet[0] &
                  (PINNACLE_PACKET0_BTN_PRIM | PINNACLE_PACKET0_BTN_SEC | PINNACLE_PACKET0_BTN_AUX);

    int8_t dx = (int8_t)packet[1];
    int8_t dy = (int8_t)packet[2];

    if (packet[0] & PINNACLE_PACKET0_X_SIGN) {
        WRITE_BIT(dx, 7, 1);
    }
    if (packet[0] & PINNACLE_PACKET0_Y_SIGN) {
        WRITE_BIT(dy, 7, 1);
    }

    // always claim touch changed
    data->last_z = 1;
    pinnacle_send_rel(dev, (int8_t) dx, (int8_t) dy);
}

static void pinnacle_work_cb(struct k_work *work) {
    struct pinnacle_data *data = CONTAINER_OF(work, struct pinnacle_data, work);
    const struct device *dev = data->dev;
    const struct pinnacle_config *config = dev->config;

    if (config->absolute_mode) {
        pinnacle_report_data_abs(dev);
    } else if (config->abs_rel_divisor) {
        pinnacle_report_data_abs_rel(dev);
    } else {
        pinnacle_report_data_rel(dev);
    }
}

static void pinnacle_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct pinnacle_data *data = CONTAINER_OF(cb, struct pinnacle_data, gpio_cb);

    LOG_DBG("HW DR asserted");
    set_int(data->dev, false); // mask the int until we've handled it (now level triggered)
    k_work_submit(&data->work);
}

static int pinnacle_adc_sensitivity_reg_value(enum pinnacle_sensitivity sensitivity) {
    switch (sensitivity) {
    case PINNACLE_SENSITIVITY_1X:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    case PINNACLE_SENSITIVITY_2X:
        return PINNACLE_TRACKING_ADC_CONFIG_2X;
    case PINNACLE_SENSITIVITY_3X:
        return PINNACLE_TRACKING_ADC_CONFIG_3X;
    case PINNACLE_SENSITIVITY_4X:
        return PINNACLE_TRACKING_ADC_CONFIG_4X;
    default:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    }
}

static int pinnacle_tune_edge_sensitivity(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    int ret;

    uint8_t x_val;
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_X_AXIS_WIDE_Z_MIN, &x_val);
    if (ret < 0) {
        LOG_WRN("Failed to read X val");
        return ret;
    }

    LOG_WRN("X val: %d", x_val);

    uint8_t y_val;
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_Y_AXIS_WIDE_Z_MIN, &y_val);
    if (ret < 0) {
        LOG_WRN("Failed to read Y val");
        return ret;
    }

    LOG_WRN("Y val: %d", y_val);

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_X_AXIS_WIDE_Z_MIN, config->x_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set X-Axis Min-Z %d", ret);
        return ret;
    }
    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_Y_AXIS_WIDE_Z_MIN, config->y_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set Y-Axis Min-Z %d", ret);
        return ret;
    }
    return 0;
}

static int pinnacle_set_adc_tracking_sensitivity(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;

    uint8_t val;
    int ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    val &= 0x3F;
    val |= pinnacle_adc_sensitivity_reg_value(config->sensitivity);

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, val);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
    }
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    return ret;
}

static int pinnacle_force_recalibrate(const struct device *dev) {
    uint8_t val;
    int ret = pinnacle_seq_read(dev, PINNACLE_CAL_CFG, &val, 1);
    if (ret < 0) {
        LOG_ERR("Failed to get cal config %d", ret);
    }

    val |= 0x01;
    ret = pinnacle_write(dev, PINNACLE_CAL_CFG, val);
    if (ret < 0) {
        LOG_ERR("Failed to force calibration %d", ret);
    }

    do {
        pinnacle_seq_read(dev, PINNACLE_CAL_CFG, &val, 1);
    } while (val & 0x01);

    return ret;
}

int pinnacle_set_sleep(const struct device *dev, bool enabled) {
    uint8_t sys_cfg;
    int ret = pinnacle_seq_read(dev, PINNACLE_SYS_CFG, &sys_cfg, 1);
    if (ret < 0) {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & PINNACLE_SYS_CFG_EN_SLEEP) != 0) == enabled) {
        return 0;
    }

    LOG_DBG("Setting sleep: %s", (enabled ? "on" : "off"));
    WRITE_BIT(sys_cfg, PINNACLE_SYS_CFG_EN_SLEEP_BIT, enabled ? 1 : 0);

    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, sys_cfg);
    if (ret < 0) {
        LOG_ERR("can't write sleep config %d", ret);
        return ret;
    }

    return ret;
}


int pinnacle_set_shutdown(const struct device *dev, bool enabled) {
    uint8_t sys_cfg;
    int ret = pinnacle_seq_read(dev, PINNACLE_SYS_CFG, &sys_cfg, 1);
    if (ret < 0) {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & PINNACLE_SYS_CFG_SHUTDOWN) != 0) == enabled) {
        LOG_WRN("Shutdown already set to %s", (enabled ? "on" : "off"));
    }

    LOG_DBG("Setting shutdown: %s", (enabled ? "on" : "off"));

    if(enabled) {
        // interrupt pin might have bogus asserts in shtdown so disable ints here
        // FIXME if this is in things are fucked later
        set_int(dev, false);
        }

    WRITE_BIT(sys_cfg, PINNACLE_SYS_CFG_SHUTDOWN_BIT, enabled ? 1 : 0);

    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, sys_cfg);
    if (ret < 0) {
        LOG_ERR("can't write shutdown config %d", ret);
        return ret;
    }

    if (!enabled) {
        // pinnacle_clear_status(dev); // clear any spurious ints on wake
        ret = set_int(dev, true);
    }
    else {
        ret = pinnacle_seq_read(dev, PINNACLE_SYS_CFG, &sys_cfg, 1);
        if (ret < 0) {
            LOG_ERR("can't read sys config %d", ret);
            return ret;
        }
        LOG_DBG("Shutdown readback: %s", (sys_cfg & PINNACLE_SYS_CFG_SHUTDOWN) ? "on" : "off");
    }

    return ret;
}

/* ── Runtime gesture parameter get/set ──────────────────────────────────────── */

#define _PGET(field) \
    if (strcmp(key, #field) == 0) { *out = (int32_t)p->field; return 0; }
#define _PSET(field, type) \
    if (strcmp(key, #field) == 0) { p->field = (type)value; return 0; }

int pinnacle_gesture_param_get(const struct device *dev, const char *key, int32_t *out) {
    struct pinnacle_data *data = dev->data;
    const struct pinnacle_gesture_params *p = &data->gesture_params;
    _PGET(tap_timeout_ms)
    _PGET(drag_window_timeout_ms)
    _PGET(drag_pending_timeout_ms)
    _PGET(drag_jump_timeout_ms)
    _PGET(pad_off_timeout_ms)
    _PGET(scroll_rim_percent)
    _PGET(drag_jump_rim_percent)
    _PGET(dead_radius_percent)
    _PGET(rclick_x_min_percent)
    _PGET(force_drag_z_threshold)
    _PGET(double_click_drag_z_threshold)
    _PGET(wheel_clicks)
    _PGET(scroll_exclusion_zone_percent)
    _PGET(tap_snap)
    _PGET(tap_enable)
    _PGET(rclick_enable)
    _PGET(drag_enable)
    _PGET(scroll_enable)
    return -EINVAL;
}

int pinnacle_gesture_param_set(const struct device *dev, const char *key, int32_t value) {
    struct pinnacle_data *data = dev->data;
    struct pinnacle_gesture_params *p = &data->gesture_params;
    _PSET(tap_timeout_ms, uint16_t)
    _PSET(drag_window_timeout_ms, uint16_t)
    _PSET(drag_pending_timeout_ms, uint16_t)
    _PSET(drag_jump_timeout_ms, uint16_t)
    _PSET(pad_off_timeout_ms, uint16_t)
    _PSET(scroll_rim_percent, uint8_t)
    _PSET(drag_jump_rim_percent, uint8_t)
    _PSET(dead_radius_percent, uint8_t)
    _PSET(rclick_x_min_percent, uint8_t)
    _PSET(force_drag_z_threshold, uint8_t)
    _PSET(double_click_drag_z_threshold, uint8_t)
    _PSET(wheel_clicks, uint8_t)
    _PSET(scroll_exclusion_zone_percent, uint8_t)
    if (strcmp(key, "tap_snap") == 0) { p->tap_snap = (value != 0); return 0; }
    if (strcmp(key, "tap_enable") == 0) { p->tap_enable = (value != 0); return 0; }
    if (strcmp(key, "rclick_enable") == 0) { p->rclick_enable = (value != 0); return 0; }
    if (strcmp(key, "drag_enable") == 0) { p->drag_enable = (value != 0); return 0; }
    if (strcmp(key, "scroll_enable") == 0) { p->scroll_enable = (value != 0); return 0; }
    return -EINVAL;
}

#undef _PGET
#undef _PSET

static int pinnacle_init(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    const struct pinnacle_config *config = dev->config;
    int ret;

    uint8_t fw_id[2];
    ret = pinnacle_seq_read(dev, PINNACLE_FW_ID, fw_id, 2);
    if (ret < 0) {
        LOG_ERR("Failed to get the FW ID %d", ret);
    }

    LOG_DBG("Found device with FW ID: 0x%02x, Version: 0x%02x", fw_id[0], fw_id[1]);

    k_msleep(10);
    ret = pinnacle_write(dev, PINNACLE_STATUS1, 0); // Clear CC
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    k_usleep(50);
    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, PINNACLE_SYS_CFG_RESET);
    if (ret < 0) {
        LOG_ERR("can't reset %d", ret);
        return ret;
    }
    k_msleep(20);
    ret = pinnacle_write(dev, PINNACLE_Z_IDLE, NUM_ZIDLE + NUM_ZIDLE_PAD);
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    ret = pinnacle_set_adc_tracking_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
        return ret;
    }

    ret = pinnacle_tune_edge_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to tune edge sensitivity %d", ret);
        return ret;
    }
    ret = pinnacle_force_recalibrate(dev);
    if (ret < 0) {
        LOG_ERR("Failed to force recalibration %d", ret);
        return ret;
    }

    if (config->sleep_en) {
        ret = pinnacle_set_sleep(dev, true);
        if (ret < 0) {
            return ret;
        }
    }

    uint8_t packet[1];
    ret = pinnacle_seq_read(dev, PINNACLE_SLEEP_INTERVAL, packet, 1);
 
    if (ret >= 0) {
        LOG_DBG("Default sleep interval %d", packet[0]);
    }

    ret = pinnacle_write(dev, PINNACLE_SLEEP_INTERVAL, 255);
    if (ret <= 0) {
        LOG_DBG("Failed to update sleep interaval %d", ret);
    }

    uint8_t feed_cfg2 = PINNACLE_FEED_CFG2_EN_IM | PINNACLE_FEED_CFG2_EN_BTN_SCRL;
    if (config->no_taps) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_TAP;
    }

    if (config->no_secondary_tap) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_SEC;
    }

    if (config->rotate_90) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_ROTATE_90;
    }
    ret = pinnacle_write(dev, PINNACLE_FEED_CFG2, feed_cfg2);
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    uint8_t feed_cfg1 = PINNACLE_FEED_CFG1_EN_FEED;
    if (config->absolute_mode || config->abs_rel_divisor) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_ABS_MODE;
        LOG_INF("Using absolute mode");
    } else {
        LOG_INF("Using relative mode");
    }
    if (config->x_invert) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_INV_X;
    }

    if (config->y_invert) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_INV_Y;
    }
    if (feed_cfg1) {
        ret = pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);
    }
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    data->dev = dev;

    /* Pre-set num_z_idle so the first touch is recognised as a new contact. */
    data->num_z_idle = NUM_ZIDLE;
    data->state = PINNACLE_STATE_INACTIVE;

    /* Copy DTS gesture defaults into the mutable runtime params. Settings
     * subsystem (debug_rpc.c) may override these after init completes. */
    if (config->absolute_mode) {
        struct pinnacle_gesture_params *p = &data->gesture_params;
        p->tap_timeout_ms               = config->tap_timeout_ms;
        p->drag_window_timeout_ms       = config->drag_window_timeout_ms;
        p->drag_pending_timeout_ms      = config->drag_pending_timeout_ms;
        p->drag_jump_timeout_ms         = config->drag_jump_timeout_ms;
        p->pad_off_timeout_ms           = config->pad_off_timeout_ms;
        p->scroll_rim_percent           = config->scroll_rim_percent;
        p->drag_jump_rim_percent        = config->drag_jump_rim_percent;
        p->dead_radius_percent          = config->dead_radius_percent;
        p->rclick_x_min_percent         = config->rclick_x_min_percent;
        p->force_drag_z_threshold        = config->force_drag_z_threshold;
        p->double_click_drag_z_threshold = config->double_click_drag_z_threshold;
        p->wheel_clicks                 = config->wheel_clicks;
        p->scroll_exclusion_zone_percent = config->scroll_exclusion_zone_percent;
        p->tap_snap                     = config->tap_snap;
        p->tap_enable                   = config->tap_enable;
        p->rclick_enable                = config->rclick_enable;
        p->drag_enable                  = config->drag_enable;
        p->scroll_enable                = config->scroll_enable;
    }

    pinnacle_clear_status(dev);

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, pinnacle_gpio_cb, BIT(config->dr.pin));
    ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    k_work_init(&data->work, pinnacle_work_cb);

    if (config->absolute_mode) {
        k_work_init_delayable(&data->tap_timeout_work, tap_timeout_cb);
        k_work_init_delayable(&data->drag_window_work, drag_window_cb);
        k_work_init_delayable(&data->drag_pending_timeout_work, drag_pending_timeout_cb);
        k_work_init_delayable(&data->drag_jump_work, drag_jump_cb);
        k_work_init_delayable(&data->pad_off_work, pad_off_cb);
        k_work_init_delayable(&data->tap_click_work, tap_click_cb);
    }

    pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);

    set_int(dev, true);

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int pinnacle_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        pinnacle_set_shutdown(dev, true);
        return 0;
    case PM_DEVICE_ACTION_RESUME:
        pinnacle_set_shutdown(dev, false);
        return 0;
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define PINNACLE_INST(n)                                                                           \
    static struct pinnacle_data pinnacle_data_##n;                                                 \
    static const struct pinnacle_config pinnacle_config_##n = {                                    \
        COND_CODE_1(DT_INST_ON_BUS(n, i2c),                                                        \
                    (.bus = {.i2c = I2C_DT_SPEC_INST_GET(n)}, .seq_read = pinnacle_i2c_seq_read,   \
                     .write = pinnacle_i2c_write),                                                 \
                    (.bus = {.spi = SPI_DT_SPEC_INST_GET(n,                                        \
                                                         SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |    \
                                                             SPI_TRANSFER_MSB | SPI_MODE_CPHA,     \
                                                         0)},                                      \
                     .seq_read = pinnacle_spi_seq_read, .write = pinnacle_spi_write)),             \
        .rotate_90 = DT_INST_PROP(n, rotate_90),                                                   \
        .x_invert = DT_INST_PROP(n, x_invert),                                                     \
        .y_invert = DT_INST_PROP(n, y_invert),                                                     \
        .sleep_en = DT_INST_PROP(n, sleep),                                                        \
        .no_taps = DT_INST_PROP(n, no_taps),                                                       \
        .no_secondary_tap = DT_INST_PROP(n, no_secondary_tap),                                     \
        .absolute_mode = DT_INST_PROP(n, absolute_mode),                                           \
        .abs_rel_divisor = DT_INST_PROP(n, abs_rel_divisor),                                       \
        .absolute_mode_scale_to_width = DT_INST_PROP(n, absolute_mode_scale_to_width),             \
        .absolute_mode_scale_to_height = DT_INST_PROP(n, absolute_mode_scale_to_height),           \
        .absolute_mode_clamp_min_x = DT_INST_PROP(n, absolute_mode_clamp_min_x),                   \
        .absolute_mode_clamp_max_x = DT_INST_PROP(n, absolute_mode_clamp_max_x),                   \
        .absolute_mode_clamp_min_y = DT_INST_PROP(n, absolute_mode_clamp_min_y),                   \
        .absolute_mode_clamp_max_y = DT_INST_PROP(n, absolute_mode_clamp_max_y),                   \
        .x_axis_z_min = DT_INST_PROP_OR(n, x_axis_z_min, 5),                                       \
        .y_axis_z_min = DT_INST_PROP_OR(n, y_axis_z_min, 4),                                       \
        .sensitivity = DT_INST_ENUM_IDX_OR(n, sensitivity, PINNACLE_SENSITIVITY_1X),               \
        .tap_timeout_ms = DT_INST_PROP(n, tap_timeout_ms),                                         \
        .drag_window_timeout_ms = DT_INST_PROP(n, drag_window_timeout_ms),                         \
        .drag_pending_timeout_ms = DT_INST_PROP(n, drag_pending_timeout_ms),                       \
        .drag_jump_timeout_ms = DT_INST_PROP(n, drag_jump_timeout_ms),                             \
        .pad_off_timeout_ms = DT_INST_PROP(n, pad_off_timeout_ms),                                 \
        .scroll_rim_percent = DT_INST_PROP(n, scroll_rim_percent),                                 \
        .drag_jump_rim_percent = DT_INST_PROP(n, drag_jump_rim_percent),                           \
        .dead_radius_percent = DT_INST_PROP(n, dead_radius_percent),                               \
        .rclick_x_min_percent = DT_INST_PROP(n, rclick_x_min_percent),                             \
        .force_drag_z_threshold = DT_INST_PROP(n, force_drag_z_threshold),                         \
        .double_click_drag_z_threshold = DT_INST_PROP(n, double_click_drag_z_threshold),           \
        .scroll_exclusion_zone_percent = DT_INST_PROP(n, scroll_exclusion_zone_percent),           \
        .wheel_clicks = DT_INST_PROP(n, wheel_clicks),                                             \
        .tap_snap = DT_INST_PROP(n, tap_snap),                                                     \
        .tap_enable = DT_INST_PROP(n, tap_enable),                                                 \
        .rclick_enable = DT_INST_PROP(n, rclick_enable),                                           \
        .drag_enable = DT_INST_PROP(n, drag_enable),                                               \
        .scroll_enable = DT_INST_PROP(n, scroll_enable),                                           \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), dr_gpios, {}),                                   \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, pinnacle_pm_action);                                               \
    DEVICE_DT_INST_DEFINE(n, pinnacle_init, PM_DEVICE_DT_INST_GET(n), &pinnacle_data_##n,          \
                          &pinnacle_config_##n, POST_KERNEL, CONFIG_INPUT_PINNACLE_INIT_PRIORITY,  \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(PINNACLE_INST)

void cirque_pinnacle_inject_abs(const struct device *dev, int16_t x, int16_t y, int8_t z) {
    struct pinnacle_data *data = dev->data;
    data->last_x = x;
    data->last_y = y;
    data->last_z = z;
    pinnacle_send_abs(dev);
}

// SPDX-License-Identifier: GPL-2.0-only
/*
 * KXCJK-1013 3-axis accelerometer driver
 * Copyright (c) 2014, Intel Corporation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/acpi.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/accel/kxtj3-1057.h>

#define KXTJ31057_DRV_NAME "kxtj31057"
#define KXTJ31057_IRQ_NAME "kxtj31057_event"

#define KXTJ31057_REG_XOUT_L            0x06
/*
 * From low byte X axis register, all the other addresses of Y and Z can be
 * obtained by just applying axis offset. The following axis defines are just
 * provide clarity, but not used.
 */
#define KXTJ31057_REG_XOUT_H            0x07
#define KXTJ31057_REG_YOUT_L            0x08
#define KXTJ31057_REG_YOUT_H            0x09
#define KXTJ31057_REG_ZOUT_L            0x0A
#define KXTJ31057_REG_ZOUT_H            0x0B

#define KXTJ31057_REG_DCST_RESP         0x0C
#define KXTJ31057_REG_WHO_AM_I          0x0F
#define KXTJ31057_REG_INT_SRC1          0x16    /* compatible, but called INT_SRC2 in KXTF9 ds */
#define KXTJ31057_REG_INT_SRC2          0x17
#define KXTJ31057_REG_STATUS_REG        0x18
#define KXTJ31057_REG_INT_REL           0x1A
#define KXTJ31057_REG_CTRL1             0x1B
#define KXTJ31057_REG_CTRL2             0x1D    /* mostly compatible, CTRL_REG3 in KTXF9 ds */
#define KXTJ31057_REG_INT_CTRL1         0x1E
#define KXTJ31057_REG_INT_CTRL2         0x1F
#define KXTJ31057_REG_DATA_CTRL         0x21
#define KXTJ31057_REG_WAKE_TIMER        0x29
#define KXTJ31057_REG_NA_COUNTER        0x2A
#define KXTJ31057_REG_SELF_TEST         0x3A
#define KXTJ31057_REG_WAKE_THRES_H      0x6A
#define KXTJ31057_REG_WAKE_THRES_L      0x6B

#define KXTJ31057_REG_CTRL1_BIT_PC1     BIT(7)
#define KXTJ31057_REG_CTRL1_BIT_RES     BIT(6)
#define KXTJ31057_REG_CTRL1_BIT_DRDY    BIT(5)
#define KXTJ31057_REG_CTRL1_BIT_GSEL1   BIT(4)
#define KXTJ31057_REG_CTRL1_BIT_GSEL0   BIT(3)
#define KXTJ31057_REG_CTRL1_BIT_EN16G   BIT(2)
#define KXTJ31057_REG_CTRL1_BIT_WUFE    BIT(1)

#define KXTJ31057_REG_INT_CTRL1_BIT_IEL BIT(3)
#define KXTJ31057_REG_INT_CTRL1_BIT_IEA BIT(4)
#define KXTJ31057_REG_INT_CTRL1_BIT_IEN BIT(5)

#define KXTJ31057_DATA_MASK_12_BIT      0x0FFF
#define KXTJ31057_MAX_STARTUP_TIME_US   100000

#define KXTJ31057_SLEEP_DELAY_MS        2000

#define KXTJ31057_REG_INT_SRC1_BIT_WUFS BIT(1)
#define KXTJ31057_REG_INT_SRC1_TAP_NONE         0
#define KXTJ31057_REG_INT_SRC1_TAP_SINGLE               BIT(2)
#define KXTJ31057_REG_INT_SRC1_TAP_DOUBLE               BIT(3)
#define KXTJ31057_REG_INT_SRC1_BIT_DRDY BIT(4)

/* KXCJK: INT_SOURCE2: motion detect, KXTF9: INT_SRC_REG1: tap detect */
#define KXTJ31057_REG_INT_SRC2_BIT_ZP   BIT(0)
#define KXTJ31057_REG_INT_SRC2_BIT_ZN   BIT(1)
#define KXTJ31057_REG_INT_SRC2_BIT_YP   BIT(2)
#define KXTJ31057_REG_INT_SRC2_BIT_YN   BIT(3)
#define KXTJ31057_REG_INT_SRC2_BIT_XP   BIT(4)
#define KXTJ31057_REG_INT_SRC2_BIT_XN   BIT(5)

#define KXTJ31057_DEFAULT_WAKE_THRES    1

enum kx_chipset {
        KXTJ31057,
        KXTJ21009,
        KX_MAX_CHIPS /* this must be last */
};

struct kxtj31057_data {
        struct i2c_client *client;
        struct iio_trigger *dready_trig;
        struct iio_trigger *motion_trig;
        struct mutex mutex;
        s16 buffer[8];
        u8 odr_bits;
        u8 range;
        int wake_thres;
        int wake_dur;
        bool active_high_intr;
        bool dready_trigger_on;
        int ev_enable_state;
        bool motion_trigger_on;
        int64_t timestamp;
        enum kx_chipset chipset;
        bool is_smo8500_device;
};

enum kxtj31057_axis {
        AXIS_X,
        AXIS_Y,
        AXIS_Z,
        AXIS_MAX,
};

enum kxtj31057_mode {
        STANDBY,
        OPERATION,
};

enum kxtj31057_range {
        KXTJ31057_RANGE_2G,
        KXTJ31057_RANGE_4G,
        KXTJ31057_RANGE_8G,
        KXTJ31057_RANGE_16G,
};

struct kx_odr_map {
        int val;
        int val2;
        int odr_bits;
        int wuf_bits;
};

static const struct kx_odr_map samp_freq_table[] = {
        { 0, 781000, 0x08, 0x00 },
        { 1, 563000, 0x09, 0x01 },
        { 3, 125000, 0x0A, 0x02 },
        { 6, 250000, 0x0B, 0x03 },
        { 12, 500000, 0x00, 0x04 },
        { 25, 0, 0x01, 0x05 },
        { 50, 0, 0x02, 0x06 },
        { 100, 0, 0x03, 0x06 },
        { 200, 0, 0x04, 0x06 },
        { 400, 0, 0x05, 0x06 },
        { 800, 0, 0x06, 0x06 },
        { 1600, 0, 0x07, 0x06 },
};

static const char *const kxtj31057_samp_freq_avail =
        "0.781000 1.563000 3.125000 6.250000 12.500000 25 50 100 200 400 800 1600";

/* Refer to section 4 of the specification */
static const struct {
        int odr_bits;
        int usec;
} odr_start_up_times[KX_MAX_CHIPS][12] = {
        /* KXTJ3-1057 */
        {
                 {0x08, 1240000},
                {0x09, 621000},
                {0x0A, 309000},
                {0x0B, 151000},
                {0, 80000},
                {0x01, 41000},
                {0x02, 21000},
                {0x03, 11000},
                {0x04, 6000},
                {0x05, 4000},
                {0x06, 3000},
                {0x07, 2000},
        },
        /* KXCTJ2-1009 */
        {
                {0x08, 1240000},
                {0x09, 621000},
                {0x0A, 309000},
                {0x0B, 151000},
                {0, 80000},
                {0x01, 41000},
                {0x02, 21000},
                {0x03, 11000},
                {0x04, 6000},
                {0x05, 4000},
                {0x06, 3000},
                {0x07, 2000},
        },
};

static const struct {
        u16 scale;
        u8 gsel_0;
        u8 gsel_1;
} KXTJ31057_scale_table[] = { {9582, 0, 0},
                              {19163, 1, 0},
                              {38326, 0, 1} };

static int kxtj31057_set_mode(struct kxtj31057_data *data,
                              enum kxtj31057_mode mode)
{
        int ret;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_CTRL1);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_ctrl1\n");
                return ret;
        }

        if (mode == STANDBY)
                ret &= ~KXTJ31057_REG_CTRL1_BIT_PC1;
        else
                ret |= KXTJ31057_REG_CTRL1_BIT_PC1;

        ret = i2c_smbus_write_byte_data(data->client,
                                        KXTJ31057_REG_CTRL1, ret);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error writing reg_ctrl1\n");
                return ret;
        }

        return 0;
}

static int kxtj31057_get_mode(struct kxtj31057_data *data,
                              enum kxtj31057_mode *mode)
{
        int ret;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_CTRL1);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_ctrl1\n");
                return ret;
        }

        if (ret & KXTJ31057_REG_CTRL1_BIT_PC1)
                *mode = OPERATION;
        else
                *mode = STANDBY;

        return 0;
}

static int kxtj31057_set_range(struct kxtj31057_data *data, int range_index)
{
        int ret;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_CTRL1);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_ctrl1\n");
                return ret;
        }

        ret &= ~(KXTJ31057_REG_CTRL1_BIT_GSEL0 |
                 KXTJ31057_REG_CTRL1_BIT_GSEL1);
        ret |= (KXTJ31057_scale_table[range_index].gsel_0 << 3);
        ret |= (KXTJ31057_scale_table[range_index].gsel_1 << 4);

        ret = i2c_smbus_write_byte_data(data->client,
                                        KXTJ31057_REG_CTRL1,
                                        ret);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error writing reg_ctrl1\n");
                return ret;
        }

        data->range = range_index;

        return 0;
}

static int kxtj31057_chip_init(struct kxtj31057_data *data)
{
        int ret;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_WHO_AM_I);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading who_am_i\n");
                return ret;
        }

        dev_dbg(&data->client->dev, "KXTJ31057 Chip Id %x\n", ret);

        ret = kxtj31057_set_mode(data, STANDBY);
        if (ret < 0)
                return ret;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_CTRL1);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_ctrl1\n");
                return ret;
        }

        /* Set 12 bit mode */
        ret |= KXTJ31057_REG_CTRL1_BIT_RES;

        ret = i2c_smbus_write_byte_data(data->client, KXTJ31057_REG_CTRL1,
                                        ret);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_ctrl\n");
                return ret;
        }

        /* Setting range to 4G */
        ret = kxtj31057_set_range(data, KXTJ31057_RANGE_4G);
        if (ret < 0)
                return ret;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_DATA_CTRL);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_data_ctrl\n");
                return ret;
        }

        data->odr_bits = ret;

        /* Set up INT polarity */
        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_INT_CTRL1);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_int_ctrl1\n");
                return ret;
        }

        if (data->active_high_intr)
                ret |= KXTJ31057_REG_INT_CTRL1_BIT_IEA;
        else
                ret &= ~KXTJ31057_REG_INT_CTRL1_BIT_IEA;

        ret = i2c_smbus_write_byte_data(data->client, KXTJ31057_REG_INT_CTRL1,
                                        ret);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error writing reg_int_ctrl1\n");
                return ret;
        }

        ret = kxtj31057_set_mode(data, OPERATION);
        if (ret < 0)
                return ret;

        data->wake_thres = KXTJ31057_DEFAULT_WAKE_THRES;

        return 0;
}

#ifdef CONFIG_PM
static int kxtj31057_get_startup_times(struct kxtj31057_data *data)
{
        int i;
        int idx = data->chipset;

        for (i = 0; i < ARRAY_SIZE(odr_start_up_times[idx]); ++i) {
                if (odr_start_up_times[idx][i].odr_bits == data->odr_bits)
                        return odr_start_up_times[idx][i].usec;
        }

        return KXTJ31057_MAX_STARTUP_TIME_US;
}
#endif

static int kxtj31057_set_power_state(struct kxtj31057_data *data, bool on)
{
#ifdef CONFIG_PM
        int ret;

        if (on)
                ret = pm_runtime_get_sync(&data->client->dev);
        else {
                pm_runtime_mark_last_busy(&data->client->dev);
                ret = pm_runtime_put_autosuspend(&data->client->dev);
        }
        if (ret < 0) {
                dev_err(&data->client->dev,
                        "Failed: %s for %d\n", __func__, on);
                if (on)
                        pm_runtime_put_noidle(&data->client->dev);
                return ret;
        }
#endif

        return 0;
}

static int kxtj31057_chip_update_thresholds(struct kxtj31057_data *data)
{
        int waketh_reg, ret;

        ret = i2c_smbus_write_byte_data(data->client,
                                        KXTJ31057_REG_WAKE_TIMER,
                                        data->wake_dur);
        if (ret < 0) {
                dev_err(&data->client->dev,
                        "Error writing reg_wake_timer\n");
                return ret;
        }

        waketh_reg = KXTJ31057_REG_WAKE_THRES_H;
        ret = i2c_smbus_write_byte_data(data->client, waketh_reg,
                                        data->wake_thres);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error writing reg_wake_thres\n");
                return ret;
        }

        return 0;
}

static int kxtj31057_setup_any_motion_interrupt(struct kxtj31057_data *data,
                                                bool status)
{
        int ret;
        enum kxtj31057_mode store_mode;

        ret = kxtj31057_get_mode(data, &store_mode);
        if (ret < 0)
                return ret;

        /* This is requirement by spec to change state to STANDBY */
        ret = kxtj31057_set_mode(data, STANDBY);
        if (ret < 0)
                return ret;

        ret = kxtj31057_chip_update_thresholds(data);
        if (ret < 0)
                return ret;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_INT_CTRL1);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_int_ctrl1\n");
                return ret;
        }

        if (status)
                ret |= KXTJ31057_REG_INT_CTRL1_BIT_IEN;
        else
                ret &= ~KXTJ31057_REG_INT_CTRL1_BIT_IEN;

        ret = i2c_smbus_write_byte_data(data->client, KXTJ31057_REG_INT_CTRL1,
                                        ret);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error writing reg_int_ctrl1\n");
                return ret;
        }

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_CTRL1);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_ctrl1\n");
                return ret;
        }

        if (status)
                ret |= KXTJ31057_REG_CTRL1_BIT_WUFE;
        else
                ret &= ~KXTJ31057_REG_CTRL1_BIT_WUFE;

        ret = i2c_smbus_write_byte_data(data->client,
                                        KXTJ31057_REG_CTRL1, ret);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error writing reg_ctrl1\n");
                return ret;
        }

        if (store_mode == OPERATION) {
                ret = kxtj31057_set_mode(data, OPERATION);
                if (ret < 0)
                        return ret;
        }

        return 0;
}

static int kxtj31057_setup_new_data_interrupt(struct kxtj31057_data *data,
                                              bool status)
{
        int ret;
        enum kxtj31057_mode store_mode;

        ret = kxtj31057_get_mode(data, &store_mode);
        if (ret < 0)
                return ret;

        /* This is requirement by spec to change state to STANDBY */
        ret = kxtj31057_set_mode(data, STANDBY);
        if (ret < 0)
                return ret;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_INT_CTRL1);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_int_ctrl1\n");
                return ret;
        }

        if (status)
                ret |= KXTJ31057_REG_INT_CTRL1_BIT_IEN;
        else
                ret &= ~KXTJ31057_REG_INT_CTRL1_BIT_IEN;

        ret = i2c_smbus_write_byte_data(data->client, KXTJ31057_REG_INT_CTRL1,
                                        ret);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error writing reg_int_ctrl1\n");
                return ret;
        }

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_CTRL1);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_ctrl1\n");
                return ret;
        }

        if (status)
                ret |= KXTJ31057_REG_CTRL1_BIT_DRDY;
        else
                ret &= ~KXTJ31057_REG_CTRL1_BIT_DRDY;

        ret = i2c_smbus_write_byte_data(data->client,
                                        KXTJ31057_REG_CTRL1, ret);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error writing reg_ctrl1\n");
                return ret;
        }

        if (store_mode == OPERATION) {
                ret = kxtj31057_set_mode(data, OPERATION);
                if (ret < 0)
                        return ret;
        }

        return 0;
}

static const struct kx_odr_map *kxtj31057_find_odr_value(
        const struct kx_odr_map *map, size_t map_size, int val, int val2)
{
        int i;

        for (i = 0; i < map_size; ++i) {
                if (map[i].val == val && map[i].val2 == val2)
                        return &map[i];
        }

        return ERR_PTR(-EINVAL);
}

static int kxtj31057_convert_odr_value(const struct kx_odr_map *map,
                                       size_t map_size, int odr_bits,
                                       int *val, int *val2)
{
        int i;

        for (i = 0; i < map_size; ++i) {
                if (map[i].odr_bits == odr_bits) {
                        *val = map[i].val;
                        *val2 = map[i].val2;
                        return IIO_VAL_INT_PLUS_MICRO;
                }
        }

        return -EINVAL;
}

static int kxtj31057_set_odr(struct kxtj31057_data *data, int val, int val2)
{
        int ret;
        enum kxtj31057_mode store_mode;
        const struct kx_odr_map *odr_setting;

        ret = kxtj31057_get_mode(data, &store_mode);
        if (ret < 0)
                return ret;

        odr_setting = kxtj31057_find_odr_value(samp_freq_table,
                                                ARRAY_SIZE(samp_freq_table),
                                                val, val2);

        if (IS_ERR(odr_setting))
                return PTR_ERR(odr_setting);

        /* To change ODR, the chip must be set to STANDBY as per spec */
        ret = kxtj31057_set_mode(data, STANDBY);
        if (ret < 0)
                return ret;

        ret = i2c_smbus_write_byte_data(data->client, KXTJ31057_REG_DATA_CTRL,
                                        odr_setting->odr_bits);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error writing data_ctrl\n");
                return ret;
        }

        data->odr_bits = odr_setting->odr_bits;

        ret = i2c_smbus_write_byte_data(data->client, KXTJ31057_REG_CTRL2,
                                        odr_setting->wuf_bits);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error writing reg_ctrl2\n");
                return ret;
        }

        if (store_mode == OPERATION) {
                ret = kxtj31057_set_mode(data, OPERATION);
                if (ret < 0)
                        return ret;
        }

        return 0;
}

static int kxtj31057_get_odr(struct kxtj31057_data *data, int *val, int *val2)
{
        return kxtj31057_convert_odr_value(samp_freq_table,
                                                ARRAY_SIZE(samp_freq_table),
                                                data->odr_bits, val, val2);
}

static int kxtj31057_get_acc_reg(struct kxtj31057_data *data, int axis)
{
        u8 reg = KXTJ31057_REG_XOUT_L + axis * 2;
        int ret;

        ret = i2c_smbus_read_word_data(data->client, reg);
        if (ret < 0) {
                dev_err(&data->client->dev,
                        "failed to read accel_%c registers\n", 'x' + axis);
                return ret;
        }

        return ret;
}

static int kxtj31057_set_scale(struct kxtj31057_data *data, int val)
{
        int ret, i;
        enum kxtj31057_mode store_mode;

        for (i = 0; i < ARRAY_SIZE(KXTJ31057_scale_table); ++i) {
                if (KXTJ31057_scale_table[i].scale == val) {
                        ret = kxtj31057_get_mode(data, &store_mode);
                        if (ret < 0)
                                return ret;

                        ret = kxtj31057_set_mode(data, STANDBY);
                        if (ret < 0)
                                return ret;

                        ret = kxtj31057_set_range(data, i);
                        if (ret < 0)
                                return ret;

                        if (store_mode == OPERATION) {
                                ret = kxtj31057_set_mode(data, OPERATION);
                                if (ret)
                                        return ret;
                        }

                        return 0;
                }
        }

        return -EINVAL;
}

static int kxtj31057_read_raw(struct iio_dev *indio_dev,
                              struct iio_chan_spec const *chan, int *val,
                              int *val2, long mask)
{
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret;

        switch (mask) {
        case IIO_CHAN_INFO_RAW:
                mutex_lock(&data->mutex);
                if (iio_buffer_enabled(indio_dev))
                        ret = -EBUSY;
                else {
                        ret = kxtj31057_set_power_state(data, true);
                        if (ret < 0) {
                                mutex_unlock(&data->mutex);
                                return ret;
                        }
                        ret = kxtj31057_get_acc_reg(data, chan->scan_index);
                        if (ret < 0) {
                                kxtj31057_set_power_state(data, false);
                                mutex_unlock(&data->mutex);
                                return ret;
                        }
                        *val = sign_extend32(ret >> 4, 11);
                        ret = kxtj31057_set_power_state(data, false);
                }
                mutex_unlock(&data->mutex);

                if (ret < 0)
                        return ret;

                return IIO_VAL_INT;

        case IIO_CHAN_INFO_SCALE:
                *val = 0;
                *val2 = KXTJ31057_scale_table[data->range].scale;
                return IIO_VAL_INT_PLUS_MICRO;

        case IIO_CHAN_INFO_SAMP_FREQ:
                mutex_lock(&data->mutex);
                ret = kxtj31057_get_odr(data, val, val2);
                mutex_unlock(&data->mutex);
                return ret;

        default:
                return -EINVAL;
        }
}

static int kxtj31057_write_raw(struct iio_dev *indio_dev,
                               struct iio_chan_spec const *chan, int val,
                               int val2, long mask)
{
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret;

        switch (mask) {
        case IIO_CHAN_INFO_SAMP_FREQ:
                mutex_lock(&data->mutex);
                ret = kxtj31057_set_odr(data, val, val2);
                mutex_unlock(&data->mutex);
                break;
        case IIO_CHAN_INFO_SCALE:
                if (val)
                        return -EINVAL;

                mutex_lock(&data->mutex);
                ret = kxtj31057_set_scale(data, val2);
                mutex_unlock(&data->mutex);
                break;
        default:
                ret = -EINVAL;
        }

        return ret;
}

static int kxtj31057_read_event(struct iio_dev *indio_dev,
                                   const struct iio_chan_spec *chan,
                                   enum iio_event_type type,
                                   enum iio_event_direction dir,
                                   enum iio_event_info info,
                                   int *val, int *val2)
{
        struct kxtj31057_data *data = iio_priv(indio_dev);

        *val2 = 0;
        switch (info) {
        case IIO_EV_INFO_VALUE:
                *val = data->wake_thres;
                break;
        case IIO_EV_INFO_PERIOD:
                *val = data->wake_dur;
                break;
        default:
                return -EINVAL;
        }

        return IIO_VAL_INT;
}

static int kxtj31057_write_event(struct iio_dev *indio_dev,
                                    const struct iio_chan_spec *chan,
                                    enum iio_event_type type,
                                    enum iio_event_direction dir,
                                    enum iio_event_info info,
                                    int val, int val2)
{
        struct kxtj31057_data *data = iio_priv(indio_dev);

        if (data->ev_enable_state)
                return -EBUSY;

        switch (info) {
        case IIO_EV_INFO_VALUE:
                data->wake_thres = val;
                break;
        case IIO_EV_INFO_PERIOD:
                data->wake_dur = val;
                break;
        default:
                return -EINVAL;
        }

        return 0;
}

static int kxtj31057_read_event_config(struct iio_dev *indio_dev,
                                          const struct iio_chan_spec *chan,
                                          enum iio_event_type type,
                                          enum iio_event_direction dir)
{
        struct kxtj31057_data *data = iio_priv(indio_dev);

        return data->ev_enable_state;
}

static int kxtj31057_write_event_config(struct iio_dev *indio_dev,
                                           const struct iio_chan_spec *chan,
                                           enum iio_event_type type,
                                           enum iio_event_direction dir,
                                           int state)
{
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret;

        if (state && data->ev_enable_state)
                return 0;

        mutex_lock(&data->mutex);

        if (!state && data->motion_trigger_on) {
                data->ev_enable_state = 0;
                mutex_unlock(&data->mutex);
                return 0;
        }

        /*
         * We will expect the enable and disable to do operation in
         * in reverse order. This will happen here anyway as our
         * resume operation uses sync mode runtime pm calls, the
         * suspend operation will be delayed by autosuspend delay
         * So the disable operation will still happen in reverse of
         * enable operation. When runtime pm is disabled the mode
         * is always on so sequence doesn't matter
         */
        ret = kxtj31057_set_power_state(data, state);
        if (ret < 0) {
                mutex_unlock(&data->mutex);
                return ret;
        }

        ret =  kxtj31057_setup_any_motion_interrupt(data, state);
        if (ret < 0) {
                kxtj31057_set_power_state(data, false);
                data->ev_enable_state = 0;
                mutex_unlock(&data->mutex);
                return ret;
        }

        data->ev_enable_state = state;
        mutex_unlock(&data->mutex);

        return 0;
}

static int kxtj31057_buffer_preenable(struct iio_dev *indio_dev)
{
        struct kxtj31057_data *data = iio_priv(indio_dev);

        return kxtj31057_set_power_state(data, true);
}

static int kxtj31057_buffer_postdisable(struct iio_dev *indio_dev)
{
        struct kxtj31057_data *data = iio_priv(indio_dev);

        return kxtj31057_set_power_state(data, false);
}

static ssize_t kxtj31057_get_samp_freq_avail(struct device *dev,
                                             struct device_attribute *attr,
                                             char *buf)
{
        //struct iio_dev *indio_dev = dev_to_iio_dev(dev);
        //struct kxtj31057_data *data = iio_priv(indio_dev);
        const char *str;

        str = kxtj31057_samp_freq_avail;

        return sprintf(buf, "%s\n", str);
}

static IIO_DEVICE_ATTR(in_accel_sampling_frequency_available, S_IRUGO,
                       kxtj31057_get_samp_freq_avail, NULL, 0);

static IIO_CONST_ATTR(in_accel_scale_available, "0.009582 0.019163 0.038326");

static struct attribute *kxtj31057_attributes[] = {
        &iio_dev_attr_in_accel_sampling_frequency_available.dev_attr.attr,
        &iio_const_attr_in_accel_scale_available.dev_attr.attr,
        NULL,
};

static const struct attribute_group kxtj31057_attrs_group = {
        .attrs = kxtj31057_attributes,
};

static const struct iio_event_spec kxtj31057_event = {
                .type = IIO_EV_TYPE_THRESH,
                .dir = IIO_EV_DIR_EITHER,
                .mask_separate = BIT(IIO_EV_INFO_VALUE) |
                                 BIT(IIO_EV_INFO_ENABLE) |
                                 BIT(IIO_EV_INFO_PERIOD)
};

#define KXTJ31057_CHANNEL(_axis) {                                      \
        .type = IIO_ACCEL,                                              \
        .modified = 1,                                                  \
        .channel2 = IIO_MOD_##_axis,                                    \
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                   \
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |          \
                                BIT(IIO_CHAN_INFO_SAMP_FREQ),           \
        .scan_index = AXIS_##_axis,                                     \
        .scan_type = {                                                  \
                .sign = 's',                                            \
                .realbits = 12,                                         \
                .storagebits = 16,                                      \
                .shift = 4,                                             \
                .endianness = IIO_LE,                                   \
        },                                                              \
        .event_spec = &kxtj31057_event,                         \
        .num_event_specs = 1                                            \
}

static const struct iio_chan_spec kxtj31057_channels[] = {
        KXTJ31057_CHANNEL(X),
        KXTJ31057_CHANNEL(Y),
        KXTJ31057_CHANNEL(Z),
        IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_buffer_setup_ops kxtj31057_buffer_setup_ops = {
        .preenable              = kxtj31057_buffer_preenable,
        //.postenable             = iio_triggered_buffer_postenable,
        .postdisable            = kxtj31057_buffer_postdisable,
        //.predisable             = iio_triggered_buffer_predisable,
};

static const struct iio_info kxtj31057_info = {
        .attrs                  = &kxtj31057_attrs_group,
        .read_raw               = kxtj31057_read_raw,
        .write_raw              = kxtj31057_write_raw,
        .read_event_value       = kxtj31057_read_event,
        .write_event_value      = kxtj31057_write_event,
        .write_event_config     = kxtj31057_write_event_config,
        .read_event_config      = kxtj31057_read_event_config,
};

static const unsigned long kxtj31057_scan_masks[] = {0x7, 0};

static irqreturn_t kxtj31057_trigger_handler(int irq, void *p)
{
        struct iio_poll_func *pf = p;
        struct iio_dev *indio_dev = pf->indio_dev;
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret;

        mutex_lock(&data->mutex);
        ret = i2c_smbus_read_i2c_block_data_or_emulated(data->client,
                                                        KXTJ31057_REG_XOUT_L,
                                                        AXIS_MAX * 2,
                                                        (u8 *)data->buffer);
        mutex_unlock(&data->mutex);
        if (ret < 0)
                goto err;

        iio_push_to_buffers_with_timestamp(indio_dev, data->buffer,
                                           data->timestamp);
err:
        iio_trigger_notify_done(indio_dev->trig);

        return IRQ_HANDLED;
}

static int kxtj31057_trig_try_reen(struct iio_trigger *trig)
{
        struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_INT_REL);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_int_rel\n");
                return ret;
        }

        return 0;
}

static int kxtj31057_data_rdy_trigger_set_state(struct iio_trigger *trig,
                                                bool state)
{
        struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret;

        mutex_lock(&data->mutex);

        if (!state && data->ev_enable_state && data->motion_trigger_on) {
                data->motion_trigger_on = false;
                mutex_unlock(&data->mutex);
                return 0;
        }

        ret = kxtj31057_set_power_state(data, state);
        if (ret < 0) {
                mutex_unlock(&data->mutex);
                return ret;
        }
        if (data->motion_trig == trig)
                ret = kxtj31057_setup_any_motion_interrupt(data, state);
        else
                ret = kxtj31057_setup_new_data_interrupt(data, state);
        if (ret < 0) {
                kxtj31057_set_power_state(data, false);
                mutex_unlock(&data->mutex);
                return ret;
        }
        if (data->motion_trig == trig)
                data->motion_trigger_on = state;
        else
                data->dready_trigger_on = state;

        mutex_unlock(&data->mutex);

        return 0;
}

static const struct iio_trigger_ops kxtj31057_trigger_ops = {
        .set_trigger_state = kxtj31057_data_rdy_trigger_set_state,
        .try_reenable = kxtj31057_trig_try_reen,
};

static void kxtj31057_report_motion_event(struct iio_dev *indio_dev)
{
        struct kxtj31057_data *data = iio_priv(indio_dev);

        int ret = i2c_smbus_read_byte_data(data->client,
                                           KXTJ31057_REG_INT_SRC2);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_int_src2\n");
                return;
        }

        if (ret & KXTJ31057_REG_INT_SRC2_BIT_XN)
                iio_push_event(indio_dev,
                               IIO_MOD_EVENT_CODE(IIO_ACCEL,
                                                  0,
                                                  IIO_MOD_X,
                                                  IIO_EV_TYPE_THRESH,
                                                  IIO_EV_DIR_FALLING),
                               data->timestamp);

        if (ret & KXTJ31057_REG_INT_SRC2_BIT_XP)
                iio_push_event(indio_dev,
                               IIO_MOD_EVENT_CODE(IIO_ACCEL,
                                                  0,
                                                  IIO_MOD_X,
                                                  IIO_EV_TYPE_THRESH,
                                                  IIO_EV_DIR_RISING),
                               data->timestamp);

        if (ret & KXTJ31057_REG_INT_SRC2_BIT_YN)
                iio_push_event(indio_dev,
                               IIO_MOD_EVENT_CODE(IIO_ACCEL,
                                                  0,
                                                  IIO_MOD_Y,
                                                  IIO_EV_TYPE_THRESH,
                                                  IIO_EV_DIR_FALLING),
                               data->timestamp);

        if (ret & KXTJ31057_REG_INT_SRC2_BIT_YP)
                iio_push_event(indio_dev,
                               IIO_MOD_EVENT_CODE(IIO_ACCEL,
                                                  0,
                                                  IIO_MOD_Y,
                                                  IIO_EV_TYPE_THRESH,
                                                  IIO_EV_DIR_RISING),
                               data->timestamp);

        if (ret & KXTJ31057_REG_INT_SRC2_BIT_ZN)
                iio_push_event(indio_dev,
                               IIO_MOD_EVENT_CODE(IIO_ACCEL,
                                                  0,
                                                  IIO_MOD_Z,
                                                  IIO_EV_TYPE_THRESH,
                                                  IIO_EV_DIR_FALLING),
                               data->timestamp);

        if (ret & KXTJ31057_REG_INT_SRC2_BIT_ZP)
                iio_push_event(indio_dev,
                               IIO_MOD_EVENT_CODE(IIO_ACCEL,
                                                  0,
                                                  IIO_MOD_Z,
                                                  IIO_EV_TYPE_THRESH,
                                                  IIO_EV_DIR_RISING),
                               data->timestamp);
}

static irqreturn_t kxtj31057_event_handler(int irq, void *private)
{
        struct iio_dev *indio_dev = private;
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_INT_SRC1);
        if (ret < 0) {
                dev_err(&data->client->dev, "Error reading reg_int_src1\n");
                goto ack_intr;
        }

        if (ret & KXTJ31057_REG_INT_SRC1_BIT_WUFS) {
                kxtj31057_report_motion_event(indio_dev);
        }

ack_intr:
        if (data->dready_trigger_on)
                return IRQ_HANDLED;

        ret = i2c_smbus_read_byte_data(data->client, KXTJ31057_REG_INT_REL);
        if (ret < 0)
                dev_err(&data->client->dev, "Error reading reg_int_rel\n");

        return IRQ_HANDLED;
}

static irqreturn_t kxtj31057_data_rdy_trig_poll(int irq, void *private)
{
        struct iio_dev *indio_dev = private;
        struct kxtj31057_data *data = iio_priv(indio_dev);

        data->timestamp = iio_get_time_ns(indio_dev);

        if (data->dready_trigger_on)
                iio_trigger_poll(data->dready_trig);
        else if (data->motion_trigger_on)
                iio_trigger_poll(data->motion_trig);

        if (data->ev_enable_state)
                return IRQ_WAKE_THREAD;
        else
                return IRQ_HANDLED;
}

static const char *kxtj31057_match_acpi_device(struct device *dev,
                                               enum kx_chipset *chipset,
                                               bool *is_smo8500_device)
{
        const struct acpi_device_id *id;

        id = acpi_match_device(dev->driver->acpi_match_table, dev);
        if (!id)
                return NULL;

        if (strcmp(id->id, "SMO8500") == 0)
                *is_smo8500_device = true;

        *chipset = (enum kx_chipset)id->driver_data;

        return dev_name(dev);
}

static int kxtj31057_probe(struct i2c_client *client,
                           const struct i2c_device_id *id)
{
        struct kxtj31057_data *data;
        struct iio_dev *indio_dev;
        struct kxtj3_1057_platform_data *pdata;
        const char *name;
        int ret;

        indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
        if (!indio_dev)
                return -ENOMEM;

        data = iio_priv(indio_dev);
        i2c_set_clientdata(client, indio_dev);
        data->client = client;

        pdata = dev_get_platdata(&client->dev);
        if (pdata)
                data->active_high_intr = pdata->active_high_intr;
        else
                data->active_high_intr = true; /* default polarity */

        if (id) {
                data->chipset = (enum kx_chipset)(id->driver_data);
                name = id->name;
        } else if (ACPI_HANDLE(&client->dev)) {
                name = kxtj31057_match_acpi_device(&client->dev,
                                                   &data->chipset,
                                                   &data->is_smo8500_device);
        } else
                return -ENODEV;

        ret = kxtj31057_chip_init(data);
        if (ret < 0)
                return ret;

        mutex_init(&data->mutex);

        indio_dev->dev.parent = &client->dev;
        indio_dev->channels = kxtj31057_channels;
        indio_dev->num_channels = ARRAY_SIZE(kxtj31057_channels);
        indio_dev->available_scan_masks = kxtj31057_scan_masks;
        indio_dev->name = name;
        indio_dev->modes = INDIO_DIRECT_MODE;
        indio_dev->info = &kxtj31057_info;

        if (client->irq > 0 && !data->is_smo8500_device) {
                ret = devm_request_threaded_irq(&client->dev, client->irq,
                                                kxtj31057_data_rdy_trig_poll,
                                                kxtj31057_event_handler,
                                                IRQF_TRIGGER_RISING,
                                                KXTJ31057_IRQ_NAME,
                                                indio_dev);
                if (ret)
                        goto err_poweroff;

                data->dready_trig = devm_iio_trigger_alloc(&client->dev,
                                                           "%s-dev%d",
                                                           indio_dev->name,
                                                           indio_dev->id);
                if (!data->dready_trig) {
                        ret = -ENOMEM;
                        goto err_poweroff;
                }

                data->motion_trig = devm_iio_trigger_alloc(&client->dev,
                                                          "%s-any-motion-dev%d",
                                                          indio_dev->name,
                                                          indio_dev->id);
                if (!data->motion_trig) {
                        ret = -ENOMEM;
                        goto err_poweroff;
                }

                data->dready_trig->dev.parent = &client->dev;
                data->dready_trig->ops = &kxtj31057_trigger_ops;
                iio_trigger_set_drvdata(data->dready_trig, indio_dev);
                indio_dev->trig = data->dready_trig;
                iio_trigger_get(indio_dev->trig);
                ret = iio_trigger_register(data->dready_trig);
                if (ret)
                        goto err_poweroff;

                data->motion_trig->dev.parent = &client->dev;
                data->motion_trig->ops = &kxtj31057_trigger_ops;
                iio_trigger_set_drvdata(data->motion_trig, indio_dev);
                ret = iio_trigger_register(data->motion_trig);
                if (ret) {
                        data->motion_trig = NULL;
                        goto err_trigger_unregister;
                }
        }

        ret = iio_triggered_buffer_setup(indio_dev,
                                         &iio_pollfunc_store_time,
                                         kxtj31057_trigger_handler,
                                         &kxtj31057_buffer_setup_ops);
        if (ret < 0) {
                dev_err(&client->dev, "iio triggered buffer setup failed\n");
                goto err_trigger_unregister;
        }

        ret = pm_runtime_set_active(&client->dev);
        if (ret)
                goto err_buffer_cleanup;

        pm_runtime_enable(&client->dev);
        pm_runtime_set_autosuspend_delay(&client->dev,
                                         KXTJ31057_SLEEP_DELAY_MS);
        pm_runtime_use_autosuspend(&client->dev);

        ret = iio_device_register(indio_dev);
        if (ret < 0) {
                dev_err(&client->dev, "unable to register iio device\n");
                goto err_buffer_cleanup;
        }

        return 0;

err_buffer_cleanup:
        if (data->dready_trig)
                iio_triggered_buffer_cleanup(indio_dev);
err_trigger_unregister:
        if (data->dready_trig)
                iio_trigger_unregister(data->dready_trig);
        if (data->motion_trig)
                iio_trigger_unregister(data->motion_trig);
err_poweroff:
        kxtj31057_set_mode(data, STANDBY);

        return ret;
}

static int kxtj31057_remove(struct i2c_client *client)
{
        struct iio_dev *indio_dev = i2c_get_clientdata(client);
        struct kxtj31057_data *data = iio_priv(indio_dev);

        iio_device_unregister(indio_dev);

        pm_runtime_disable(&client->dev);
        pm_runtime_set_suspended(&client->dev);
        pm_runtime_put_noidle(&client->dev);

        if (data->dready_trig) {
                iio_triggered_buffer_cleanup(indio_dev);
                iio_trigger_unregister(data->dready_trig);
                iio_trigger_unregister(data->motion_trig);
        }

        mutex_lock(&data->mutex);
        kxtj31057_set_mode(data, STANDBY);
        mutex_unlock(&data->mutex);

        return 0;
}

#ifdef CONFIG_PM_SLEEP
static int kxtj31057_suspend(struct device *dev)
{
        struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret;

        mutex_lock(&data->mutex);
        ret = kxtj31057_set_mode(data, STANDBY);
        mutex_unlock(&data->mutex);

        return ret;
}

static int kxtj31057_resume(struct device *dev)
{
        struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret = 0;

        mutex_lock(&data->mutex);
        ret = kxtj31057_set_mode(data, OPERATION);
        if (ret == 0)
                ret = kxtj31057_set_range(data, data->range);
        mutex_unlock(&data->mutex);

        return ret;
}
#endif

#ifdef CONFIG_PM
static int kxtj31057_runtime_suspend(struct device *dev)
{
        struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret;

        ret = kxtj31057_set_mode(data, STANDBY);
        if (ret < 0) {
                dev_err(&data->client->dev, "powering off device failed\n");
                return -EAGAIN;
        }
        return 0;
}

static int kxtj31057_runtime_resume(struct device *dev)
{
        struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
        struct kxtj31057_data *data = iio_priv(indio_dev);
        int ret;
        int sleep_val;

        ret = kxtj31057_set_mode(data, OPERATION);
        if (ret < 0)
                return ret;

        sleep_val = kxtj31057_get_startup_times(data);
        if (sleep_val < 20000)
                usleep_range(sleep_val, 20000);
        else
                msleep_interruptible(sleep_val/1000);

        return 0;
}
#endif

static const struct dev_pm_ops kxtj31057_pm_ops = {
        SET_SYSTEM_SLEEP_PM_OPS(kxtj31057_suspend, kxtj31057_resume)
        SET_RUNTIME_PM_OPS(kxtj31057_runtime_suspend,
                           kxtj31057_runtime_resume, NULL)
};

static const struct acpi_device_id kx_acpi_match[] = {
        {"KXTJ1057", KXTJ31057},
        {"KIOX0009", KXTJ21009},
        {"KXTJ1009", KXTJ21009},
        {"KXJ2109",  KXTJ21009},
        { },
};
MODULE_DEVICE_TABLE(acpi, kx_acpi_match);

static const struct i2c_device_id kxtj31057_id[] = {
        {"kxtj31057", KXTJ31057},
        {"kxtj21009", KXTJ21009},
        {}
};

MODULE_DEVICE_TABLE(i2c, kxtj31057_id);

static const struct of_device_id kxtj31057_of_match[] = {
        { .compatible = "kionix,kxtj31057", },
        { .compatible = "kionix,kxtj21009", },
        { }
};
MODULE_DEVICE_TABLE(of, kxtj31057_of_match);

static struct i2c_driver kxtj31057_driver = {
        .driver = {
                .name   = KXTJ31057_DRV_NAME,
                .acpi_match_table = ACPI_PTR(kx_acpi_match),
                .of_match_table = kxtj31057_of_match,
                .pm     = &kxtj31057_pm_ops,
        },
        .probe          = kxtj31057_probe,
        .remove         = kxtj31057_remove,
        .id_table       = kxtj31057_id,
};
module_i2c_driver(kxtj31057_driver);

MODULE_AUTHOR("Srinivas Pandruvada <srinivas.pandruvada@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("KXTJ31057 accelerometer driver");

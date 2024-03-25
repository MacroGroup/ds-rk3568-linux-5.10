// SPDX-License-Identifier: GPL-2.0
/*
 * SC8815 charger single mode driver for Firefly AIO-3562JQ
 *
 * Copyright (c) Firefly Technology Co., Ltd.
 *
 * Author: Eric <lth@t-chip.com.cn>
 */
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/delay.h>

#define ON true
#define OFF false

/* Read & Write */
/* Register 00H */
#define SC8815_VBAT_SET             0x00

#define SC8815_IRCOMP_MASK          0xc0
#define SC8815_IRCOMP_SHIFT         6

#define SC8815_VBAT_SEL_MASK        0x20
#define SC8815_VBAT_SEL_SHIFT       5

#define SC8815_CSEL_MASK            0x18
#define SC8815_CSEL_SHIFT           3

#define SC8815_VCELL_SET_MASK       0x07
#define SC8815_VCELL_SET_SHIFT      0

/* Register 01H */
#define SC8815_VBUSREF_I_SET        0x01

/* Register 02H */
#define SC8815_VBUSREF_I_SET2       0x02
#define SC8815_VBUSREF_I_SET2_MASK  0xc0
#define SC8815_VBUSREF_I_SET2_SHIFT 6

/* Register 03H */
#define SC8815_VBUSREF_E_SET        0x03

/* Register 04H */
#define SC8815_VBUSREF_E_SET2       0x04
#define SC8815_VBUSREF_E_SET2_MASK  0xc0
#define SC8815_VBUSREF_E_SET2_SHIFT 6

/* Register 05H */
#define SC8815_IBUS_LIM_SET         0x05

/* Register 06H */
#define SC8815_IBAT_LIM_SET         0x06

/* Register 07H */
#define SC8815_VINREG_SET           0x07

/* Register 08H */
#define SC8815_RATIO                0x08
#define SC8815_IBAT_RATIO_MASK      0x10
#define SC8815_IBAT_RATIO_SHIFT     4

#define SC8815_IBUS_RATIO_MASK      0x0c
#define SC8815_IBUS_RATIO_SHIFT     2

#define SC8815_VBAT_MON_RATIO_MASK  0x02
#define SC8815_VBAT_MON_RATIO_SHIFT 1

#define SC8815_VBUS_RATIO_MASK      0x01
#define SC8815_VBUS_RATIO_SHIFT     0

/* Register 09H */
#define SC8815_CTRL0_SET            0x09
#define SC8815_EN_OTG_MASK          0x80
#define SC8815_EN_OTG_SHIFT         7

#define SC8815_VINREG_RATIO_MASK    0x10
#define SC8815_VINREG_RATIO_SHIFT   4

#define SC8815_FREQ_SET_MASK        0x0c
#define SC8815_FREQ_SET_SHIFT       2

#define SC8815_DT_SET_MASK          0x03
#define SC8815_DT_SET_SHIFT         0

/* Register 0AH */
#define SC8815_CTRL1_SET            0x0a
#define SC8815_ICHAR_SEL_MASK       0x80
#define SC8815_ICHAR_SEL_SHIFT      7

#define SC8815_DIS_TRICKLE_MASK     0x40
#define SC8815_DIS_TRICKLE_SHIFT    6

#define SC8815_DIS_TERM_MASK        0x20
#define SC8815_DIS_TERM_SHIFT       5

#define SC8815_FB_SEL_MASK          0x10
#define SC8815_FB_SEL_SHIFT         4

#define SC8815_TRICKLE_SET_MASK     0x08
#define SC8815_TRICKLE_SET_SHIFT    3

#define SC8815_DIS_OVP_MASK         0x04
#define SC8815_DIS_OVP_SHIFT        2

/* Register 0BH */
#define SC8815_CTRL2_SET            0x0b
#define SC8815_FACTORY_MASK         0x08
#define SC8815_FACTORY_SHIFT        3

#define SC8815_EN_DITHER_MASK       0x04
#define SC8815_EN_DITHER_SHIFT      2

#define SC8815_SLEW_SET_MASK        0x03
#define SC8815_SLEW_SET_SHIFT       0

/* Register 0CH */
#define SC8815_CTRL3_SET            0x0c
#define SC8815_EN_PAGTE_MASK        0x80
#define SC8815_EN_PAGTE_SHIFT       7

#define SC8815_GPO_CTRL_MASK        0x40
#define SC8815_GPO_CTRL_SHIFT       6

#define SC8815_AD_START_MASK        0x20
#define SC8815_AD_START_SHIFT       5

#define SC8815_ILIM_BW_SEL_MASK     0x10
#define SC8815_ILIM_BW_SEL_SHIFT    4

#define SC8815_LOOP_SET_MASK        0x08
#define SC8815_LOOP_SET_SHIFT       3

#define SC8815_DIS_SFB_MASK         0x04
#define SC8815_DIS_SFB_SHIFT        2

#define SC8815_EOC_SET_MASK         0x02
#define SC8815_EOC_SET_SHIFT        1

#define SC8815_EN_PFM_MASK          0x01
#define SC8815_EN_PFM_SHIFT         0

/* Read Only */
/* Register 0DH */
#define SC8815_VBUS_FB_VALUE        0x0d

/* Register 0EH */
#define SC8815_VBUS_FB_VALUE2       0x0e

/* Register 0FH */
#define SC8815_VBAT_FB_VALUE        0x0f

/* Register 10H */
#define SC8815_VBAT_FB_VALUE2       0x10

/* Register 11H */
#define SC8815_IBUS_VALUE           0x11

/* Register 12H */
#define SC8815_IBUS_VALUE2          0x12

/* Register 13H */
#define SC8815_IBAT_VALUE           0x13

/* Register 14H */
#define SC8815_IBAT_VALUE2          0x14

/* Register 15H */
#define SC8815_ADIN_VALUE           0x15

/* Register 16H */
#define SC8815_ADIN_VALUE2          0x16

/* Register 17H */
#define SC8815_STATUS               0x17

/* Mask R/W */
/* Register 19H */
#define SC8815_MASK                 0x19

struct sc8815
{
    struct device *dev;
    struct i2c_client *client;

    struct mutex i2c_rw_lock;

    struct gpio_desc *pstop;
};

static int sc8815_read_byte(struct sc8815 *sc, u8 reg, u8 *data)
{
    s32 ret;

    ret = i2c_smbus_read_byte_data(sc->client, reg);
    if (ret < 0) {
        dev_err(sc->dev, "i2c read fail: can't read from reg 0x%02X\n", reg);
        return ret;
    }

    *data = (u8) ret;
    return 0;
}

__attribute__((unused)) static int sc8815_read(struct sc8815 *sc, u8 reg, u8 *data)
{
    int ret;

    mutex_lock(&sc->i2c_rw_lock);
    ret = sc8815_read_byte(sc, reg, data);
    mutex_unlock(&sc->i2c_rw_lock);

    return ret;
}


static int sc8815_write_byte(struct sc8815 *sc, u8 reg, u8 data)
{
    s32 ret;

    ret = i2c_smbus_write_byte_data(sc->client, reg, data);
    if (ret < 0) {
        dev_err(sc->dev, "i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n", data, reg, ret);
        return ret;
    }

    return 0;
}

static int sc8815_write(struct sc8815 *sc, bool update_bits, u8 reg, u8 mask, u8 data)
{
    int ret;
	u8 tmp;

	mutex_lock(&sc->i2c_rw_lock);

    if (update_bits) {
        ret = sc8815_read_byte(sc, reg, &tmp);
	    if (ret) {
		    dev_err(sc->dev, "%s: Read Failed: reg=%02X, ret=%d\n", __func__, reg, ret);
		    goto out;
	    }
        tmp &= ~mask;
	    tmp |= data & mask;
    } else {
        tmp = data;
    }

	ret = sc8815_write_byte(sc, reg, tmp);
	if (ret)
		dev_err(sc->dev, "%s: Write Failed: reg=%02X, ret=%d\n", __func__, reg, ret);
out:
	mutex_unlock(&sc->i2c_rw_lock);
	return ret;
}

/**
 * @brief Set sc8815 enter/exit standby mode
 * @param sc sc8815 struct
 * @param on ON of OFF
 * @return int
 */
__attribute__((unused)) static int sc8815_standby(struct sc8815 *sc, bool on)
{
    int ret;

    if (on) {
        ret = gpiod_direction_output(sc->pstop, 1);
        dev_info(sc->dev, "standby\n");
    } else {
        ret = gpiod_direction_output(sc->pstop, 0);
        dev_info(sc->dev, "active\n");
    }

    if (ret)
        dev_err(sc->dev, "set pstop pin value failed: %d\n", ret);

    return ret;
}

/**
 * @brief Set sc8815 EN_OTG, ON means charging, OFF means discharging
 * @param sc sc8815 struct
 * @param on ON or OFF
 * @return int
 */
static int sc8815_set_otg(struct sc8815 *sc, bool on)
{
    int ret;
    u8 val;

    if (on) {
        val = 1 << SC8815_EN_OTG_SHIFT;
        ret = sc8815_write(sc, true, SC8815_CTRL0_SET, SC8815_EN_OTG_MASK, val);
    } else {
        val = 0;
        ret = sc8815_write(sc, true, SC8815_CTRL0_SET, SC8815_EN_OTG_MASK, val);
    }

    if (ret)
        dev_err(sc->dev, "set otg %d failed: %d\n", val >> SC8815_EN_OTG_SHIFT, ret);

    return ret;
}

/**
 * @brief Set sc8815 FACTORY bit to 1 after power up
 * @param sc sc8815 struct
 * @return int
 */
static int sc8815_set_factory(struct sc8815 *sc)
{
    int ret;
    u8 val = 1 << SC8815_FACTORY_SHIFT;
    ret = sc8815_write(sc, true, SC8815_CTRL2_SET, SC8815_FACTORY_MASK, val);
    if (ret)
        dev_err(sc->dev, "set factory failed: %d\n", ret);

    return ret;
}

/**
 * @brief Set sc8815 AD_START
 * @param sc sc8815 struct
 * @param on ON or OFF
 * @return int
 */
static int sc8815_set_adc(struct sc8815 *sc, bool on)
{
    int ret;
    u8 val;

    if (on) {
        val = 1 << SC8815_AD_START_SHIFT;
        ret = sc8815_write(sc, true, SC8815_CTRL3_SET, SC8815_AD_START_MASK, val);
    } else {
        val = 0;
        ret = sc8815_write(sc, true, SC8815_CTRL3_SET, SC8815_AD_START_MASK, val);
    }

    if (ret)
        dev_err(sc->dev, "set adc %d failed: %d\n", val >> SC8815_AD_START_SHIFT, ret);

    return ret;
}

static int sc8815_init(struct sc8815 *sc)
{
    int ret;

    ret = sc8815_set_factory(sc);
    if (ret)
        goto out;

    ret = sc8815_set_otg(sc, OFF);
    if (ret)
        goto out;

    ret = sc8815_set_adc(sc, ON);
    if (ret)
        goto out;

    return 0;
out:
    dev_err(sc->dev, "sc8815 init failed: %d\n", ret);
    return ret;
}

/**
 * @brief Config sc8815 working mode
 * @param sc
 * @return int
 */
static int sc8815_config_single_mode(struct sc8815 *sc)
{
    int ret;
    u8 val;

    /* 2 cell, each 4.2V */
    ret = sc8815_write(sc, false, SC8815_VBAT_SET, 0, 0x09);
    if (ret)
        goto out;

    /* IBUS limit 2A */
    ret = sc8815_write(sc, false, SC8815_IBUS_LIM_SET, 0, 0xaa);
    if (ret)
        goto out;

    /* IBAT ratio 6x, limit 1A */
    ret = sc8815_write(sc, true, SC8815_RATIO, SC8815_IBAT_RATIO_MASK, 0);
    if (ret)
        goto out;

    ret = sc8815_write(sc, false, SC8815_IBAT_LIM_SET, 0, 0x2a);
    if (ret)
        goto out;

    /* VINREG ratio 40x, set to 8V*/
    val = 1 << SC8815_VINREG_RATIO_SHIFT;
    ret = sc8815_write(sc, true, SC8815_CTRL0_SET, SC8815_VINREG_RATIO_MASK, val);
    if (ret)
        goto out;

    ret = sc8815_write(sc, false, SC8815_VINREG_SET, 0, 0xc7);
    if (ret)
        goto out;

    return 0;
out:
    dev_err(sc->dev, "config single mode failed: %d\n", ret);
    return ret;
}

/** TODO
static ssize_t registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
}

static ssize_t registers_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
}

static DEVICE_ATTR_RW(registers);

static void sc8815_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_factory);
}
**/

static const struct of_device_id sc8815_charger_match[] = {
    { .compatible = "sc8815,aio-3562jq", },
    {},
};

static int sc8815_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct sc8815 *sc;
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    int ret;

    sc = devm_kzalloc(&client->dev, sizeof(*sc), GFP_KERNEL);
    if (!sc)
        return -ENOMEM;

    sc->client = client;
    sc->dev = &client->dev;

    mutex_init(&sc->i2c_rw_lock);

    i2c_set_clientdata(client, sc);

    match = of_match_node(sc8815_charger_match, node);
    if (!match){
        dev_err(sc->dev, "no match found in device tree\n");
        return -ENODEV;
    }

    /* Normally there's a pstop pin to switch sc8815 between working mode and standby mode.
       But sc8815 goes to working mode by default after power up.
       So comment it out unless really needs pstop.
    sc->pstop = devm_gpiod_get_optional(&client->dev, "pstop", GPIOD_ASIS);
    if (IS_ERR(sc->pstop)) {
        ret = PTR_ERR(sc->pstop);
        dev_err(sc->dev, "Failed to get pstop gpio: %d\n", ret);
        return ret;
    }

    ret = sc8815_standby(sc, ON);
    if (ret)
        return ret;

    msleep(500);
    */

    ret = sc8815_init(sc);
    if (ret)
        return ret;

    ret = sc8815_config_single_mode(sc);
    if (ret)
        return ret;

    /*
    msleep(500);

    ret = sc8815_standby(sc, OFF);
    if (ret)
        return ret;
    */

    dev_info(sc->dev, "probe success\n");
    return 0;
}

static int sc8815_charger_remove(struct i2c_client *client)
{
    struct sc8815 *sc = i2c_get_clientdata(client);
    //sc8815_standby(sc, ON);
    mutex_destroy(&sc->i2c_rw_lock);
    return 0;
}

static const struct i2c_device_id sc8815_charger_id[] = {
    { "sc8815,aio-3562jq", 0 },
    {},
};

static struct i2c_driver sc8815_charger_driver = {
    .driver = {
        .name = "sc8815-charger",
        .owner = THIS_MODULE,
        .of_match_table = sc8815_charger_match,
    },
    .probe = sc8815_charger_probe,
    .remove = sc8815_charger_remove,
    .id_table = sc8815_charger_id,
};

module_i2c_driver(sc8815_charger_driver);

MODULE_AUTHOR("Firefly Technology Co., Ltd.");
MODULE_DESCRIPTION("SC8815 charger single mode driver for Firefly AIO-3562JQ");
MODULE_LICENSE("GPL");

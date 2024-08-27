/*
 * drivers/input/touchscreen/gslX680.c
 *
 * Copyright (c) 2012 Shanghai Basewin
 *	Guan Yuwei<guanyuwei@basewin.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */


#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>

#include <linux/blkdev.h>

#define PC9202_I2C_NAME 			"pc9202-wdt"
#define SW2001_REG_DECRYPT_CTRL 	0x00
#define SW2001_REG_WDT_CTRL 		0x01
#define WDT_KICK_S_0_64				0xc8
#define WDT_KICK_S_2_56				0xd8
#define WDT_KICK_S_10_24			0xe8
#define WDT_KICK_S_40_96			0xf8
#define WDT_KICK_DISABLED			(0x00)
#define PC9202_VERSION		"firefly 1.0.0   04/15/2023"

struct pc9202_pdata {
	int major;
	struct class *cls;
	struct device *dev;
	struct gpio_desc *wd_en_gpio;
	struct i2c_client	*client;
	struct mutex		lock;
	unsigned char demoBuffer[16];
};

struct pc9202_device {
	struct cdev cdev;
	struct pc9202_pdata * data;
};

bool iWriteByte(uint8_t addr, uint8_t data, struct pc9202_pdata *pdata);

void enable_wdt(struct gpio_desc * gpio, struct pc9202_pdata *pdata)
{
	printk("====== enabled 9202 wdt ======\n");
	gpiod_direction_output(gpio, 1);
}

void disable_wdt(struct gpio_desc * gpio, struct pc9202_pdata *pdata)
{
	printk("====== disabled 9202 wdt ======\n");
	gpiod_direction_output(gpio, 0);
	msleep(10);
	iWriteByte(SW2001_REG_WDT_CTRL,WDT_KICK_DISABLED, pdata);
}

/*-------------------------------------------------------------------------*/
/* sw2001_write parameter:
 * reg:
 * data:
 */
int sw2001_write(unsigned char reg, unsigned char *buf, int len , struct pc9202_pdata *pdata)
{
	int	 status=0;

	if (!pdata)
		return -ENODEV;

	mutex_lock(&pdata->lock);

	//status = i2c_smbus_write_byte_data(pdata->client, reg, data);
	status = i2c_smbus_write_i2c_block_data(pdata->client, reg, len, buf);

	mutex_unlock(&pdata->lock);

	//printk("%s: sw2001_write(0x%x,0x%x)\n", "pc9202", reg, *buf);

	if(status)
		printk("error.. status=0x%x\n",status);

	if(status>=len) return 0;
	else return status;
}

//********************************************************************************
// function    : iWriteByte(uint8_t addr, uint8_t data)
// description : write data to byte addr(internal register address in device)
// parameters  :
//                addr : register address in device
//                data:  data
// return        :
//               TRUE : success
//               FALSE : fail
//*********************************************************************************
bool iWriteByte(uint8_t addr, uint8_t data , struct pc9202_pdata *pdata)
{
	//sw2001_write(addr, &data, 1);
	return sw2001_write(addr, &data, 1 , pdata);
}

/* sw2001_read parameter:
 * reg:
 * data:
 */
int sw2001_read(unsigned char reg, unsigned char *buf, int len, struct pc9202_pdata *pdata)
{
	int	 status=0;

	if (!pdata)
		return -ENODEV;
	mutex_lock(&pdata->lock);
	//status = i2c_smbus_read_byte_data(pdata->client, reg);
	status = i2c_smbus_read_i2c_block_data(pdata->client, reg, len, buf);
	mutex_unlock(&pdata->lock);
	printk("%s: sw2001_read(0x%x) return 0x%x\n", "pc9202", reg, status);

	printk("status=0x%x\n",status);

	if(status>=len) return 0;
	else return status;
}
EXPORT_SYMBOL(sw2001_read);

bool iReadByte(uint8_t addr, uint8_t *data ,struct pc9202_pdata *pdata)
{
	//sw2001_read(addr, data, 1);
	return sw2001_read(addr, data, 1, pdata);
}

static int pc9202_wdt_suspend(struct device *dev)
{
    return 0;
}

static int pc9202_wdt_resume(struct device *dev)
{
    return 0;
}

int wdt_open(struct inode *inode, struct file *filp)
{
	struct pc9202_device *dev = container_of(inode->i_cdev, struct pc9202_device, cdev);
	filp->private_data = dev->data;
	//printk("wdt_open \r\n");
    return 0;
}

int wdt_release(struct inode *inode, struct file *filp)
{
	return 0;
}
ssize_t wdt_read(struct file *filp, char __user *buf, size_t count,loff_t *f_pos)
{
	return count;
}

ssize_t wdt_write(struct file *filp, const char __user *buf, size_t count,loff_t *f_pos)
{
	/* 把数据复制到内核空间 */
	uint8_t len = (int)count;
	struct pc9202_pdata *data = filp->private_data;
	if(len>2)
	{
		printk("wdt:inviald value too long\r\n");
		return count;
	}
	if (copy_from_user(data->demoBuffer+*f_pos, buf, count))
	{
		count = -EFAULT;
    }
	switch(data->demoBuffer[0])
	{
		case '0':iWriteByte(SW2001_REG_WDT_CTRL,WDT_KICK_S_0_64, data);break;//0.64
		case '1':iWriteByte(SW2001_REG_WDT_CTRL,WDT_KICK_S_2_56, data);break;//2.56
		case '2':iWriteByte(SW2001_REG_WDT_CTRL,WDT_KICK_S_10_24, data);break;//10.24
		case '3':iWriteByte(SW2001_REG_WDT_CTRL,WDT_KICK_S_40_96, data);break;//40.96
		case 'e':enable_wdt(data->wd_en_gpio, data);break; //enable wdt
		case 'd':disable_wdt(data->wd_en_gpio, data);break; //disable wdt
		default:printk("wdt:inviald value \r\n");break;
	}
	//printk("wdt_weite %c\r\n", data->demoBuffer[0]);
	return count;
}

struct file_operations wdt_fops = {
	.owner = THIS_MODULE,
	.read =	wdt_read,
	.write = wdt_write,
	.open = wdt_open,
	.release = wdt_release,
};

static int pc9202_wdt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct device_node *node = client->dev.of_node;
	struct pc9202_device	*dev;
	struct pc9202_pdata	*data;
	uint8_t reg_value;
	int retry_count, ret;
	dev_t devno;
	const char * driver_name =  "wdt_crl";

	dev_info(&client->dev, "Version: %s\n", PC9202_VERSION);

	if (!of_device_is_available(client->dev.of_node)) {
		return 0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -ENODEV;
	}

	dev = kzalloc(sizeof(struct pc9202_device), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	data = kzalloc(sizeof(struct pc9202_pdata), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev->data = data;

	mutex_init(&data->lock);
	data->client = client;
	
	i2c_set_clientdata(client,dev);

	for (retry_count = 0; retry_count < 100; retry_count++) {
		ret = iReadByte(SW2001_REG_WDT_CTRL, &reg_value, data);
		if(0 != ret) {
			dev_err(data->dev,"====== i2c detect failed watchdog init err: 0x%x ======\n", reg_value);
		} else {
			dev_err(data->dev,"====== i2c detect success watchdog init ======\n");
			break;
		}
		msleep(10);
	}

	if (0 != ret) {
		dev_err(data->dev,"====== i2c detect failed watchdog init ======\n");
		goto err;
	}

	ret = of_property_read_string(node, "driver-names", &driver_name);
	
	// register chrdev
	alloc_chrdev_region(&devno, 0, 1, driver_name);
	
	data->major = MAJOR(devno);
	
	cdev_init(&dev->cdev, &wdt_fops);

	ret = cdev_add(&dev->cdev, devno, 1);
	if (ret < 0) {
		unregister_chrdev_region(devno, 1);
		printk(KERN_WARNING "cdev_add failed\n");
		return ret;
	}

	// 创建类
	data->cls = class_create(THIS_MODULE, driver_name);
	// 创建设备节点
	data->dev = device_create(data->cls, NULL, MKDEV(data->major, 0), NULL, driver_name);

	data->wd_en_gpio = devm_gpiod_get_optional(&client->dev, "wd-en", GPIOD_ASIS);

	if (IS_ERR(data->wd_en_gpio)) {
		ret = PTR_ERR(data->wd_en_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(data->dev, "failed to get reset GPIO: %d\n", ret);
		return ret;
	}

	return 0;
err:
	kfree(data);
	kfree(dev);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static int pc9202_wdt_remove(struct i2c_client *client)
{
	struct pc9202_device    *dev = i2c_get_clientdata(client);

	//PR_DEBUG("%s\n",__func__);

	kfree(dev->data);
	kfree(dev);
	i2c_set_clientdata(client, NULL);
    return 0;
}

static struct of_device_id pc9202_wdt_ids[] = {
    {.compatible = "firefly,pc9202"},
    {}
};

static const struct i2c_device_id pc9202_wdt_id[] = {
    {PC9202_I2C_NAME, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, pc9202_wdt_id);

static const struct dev_pm_ops pc9202_wdt_ops = {
	//.write		= pc9202_wdt_write,
	.suspend    = pc9202_wdt_suspend,
	.resume     = pc9202_wdt_resume,
};

static struct i2c_driver pc9202_wdt_driver = {
    .driver = {
        .name = PC9202_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(pc9202_wdt_ids),
        .pm = &pc9202_wdt_ops,
    },
    .probe		= pc9202_wdt_probe,
    .remove		= pc9202_wdt_remove,
    .id_table	= pc9202_wdt_id,
};

static int __init pc9202_wdt_init(void)
{
    int ret;
    ret = i2c_add_driver(&pc9202_wdt_driver);
    return ret;
}
static void __exit pc9202_wdt_exit(void)
{
    i2c_del_driver(&pc9202_wdt_driver);
    return;
}

late_initcall(pc9202_wdt_init);
module_exit(pc9202_wdt_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PC9202 watchdog driver");
MODULE_AUTHOR("zjy, zhengjunye123@126.com");
MODULE_ALIAS("platform:rk3399");

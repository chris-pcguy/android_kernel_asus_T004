/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>

#include <linux/qpnp/vibrator.h>
#include "../../staging/android/timed_output.h"
//ASUS_BSP +++ freddy "[A86][vib][NA][Spec] added to vibrator after phone inserted into pad" 
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>	

#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>

//ASUS_BSP --- freddy "[A86][vib][NA][Spec] added to vibrator after phone inserted into pad"

#define QPNP_VIB_VTG_CTL(base)		(base + 0x41)
#define QPNP_VIB_EN_CTL(base)		(base + 0x46)

#define QPNP_VIB_DEFAULT_TIMEOUT	15000
#define QPNP_VIB_DEFAULT_VTG_LVL	3100
#define QPNP_VIB_DEFAULT_VTG_MAX	3100
#define QPNP_VIB_DEFAULT_VTG_MIN	1200

#define QPNP_VIB_EN			BIT(7)
#define QPNP_VIB_VTG_SET_MASK		0x1F
#define QPNP_VIB_LOGIC_SHIFT		4
static int g_vib_stop_val =0;

extern bool g_drv2605_probe_ok;

struct qpnp_vib {
	struct spmi_device *spmi;
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;

	u8  reg_vtg_ctl;
	u8  reg_en_ctl;
	u16 base;
	int state;
	int vtg_min;
	int vtg_max;
	int vtg_level;
	int vtg_default;
	int timeout;
	spinlock_t lock;
};

static struct qpnp_vib *vib_dev;

static ssize_t qpnp_vib_level_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qpnp_vib *vib = container_of(tdev, struct qpnp_vib, timed_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", vib->vtg_level);
}

static ssize_t qpnp_vib_level_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qpnp_vib *vib = container_of(tdev, struct qpnp_vib, timed_dev);
	int val;
	int rc;

	rc = kstrtoint(buf, 10, &val);
	if (rc) {
		pr_err("%s: error getting level\n", __func__);
		return -EINVAL;
	}

	if (val < vib->vtg_min) {
		pr_err("%s: level %d not in range (%d - %d), using min.",
			__func__, val, vib->vtg_min, vib->vtg_max);
		val = vib->vtg_min;
	} else if (val > vib->vtg_max) {
		pr_err("%s: level %d not in range (%d - %d), using max.",
			__func__, val, vib->vtg_min, vib->vtg_max);
		val = vib->vtg_max;
	}

	vib->vtg_level = val;

	return strnlen(buf, count);
}

static ssize_t qpnp_vib_min_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qpnp_vib *vib = container_of(tdev, struct qpnp_vib, timed_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", vib->vtg_min);
}

static ssize_t qpnp_vib_max_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qpnp_vib *vib = container_of(tdev, struct qpnp_vib, timed_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", vib->vtg_max);
}

static ssize_t qpnp_vib_default_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct qpnp_vib *vib = container_of(tdev, struct qpnp_vib, timed_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", vib->vtg_default);
}

static DEVICE_ATTR(vtg_level, S_IRUGO | S_IWUSR, qpnp_vib_level_show, qpnp_vib_level_store);
static DEVICE_ATTR(vtg_min, S_IRUGO, qpnp_vib_min_show, NULL);
static DEVICE_ATTR(vtg_max, S_IRUGO, qpnp_vib_max_show, NULL);
static DEVICE_ATTR(vtg_default, S_IRUGO, qpnp_vib_default_show, NULL);

static int qpnp_vib_read_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_readl(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
		dev_err(&vib->spmi->dev,
			"Error reading address: %X - ret %X\n", reg, rc);

	return rc;
}

static int qpnp_vib_write_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_writel(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
		dev_err(&vib->spmi->dev,
			"Error writing address: %X - ret %X\n", reg, rc);

	return rc;
}

int qpnp_vibrator_config(struct qpnp_vib_config *vib_cfg)
{
	u8 reg = 0;
	int rc = -EINVAL, level;

	if (vib_dev == NULL) {
		pr_err("%s: vib_dev is NULL\n", __func__);
		return -ENODEV;
	}

	level = vib_cfg->drive_mV / 100;
	if (level) {
		if ((level < vib_dev->vtg_min) ||
				(level > vib_dev->vtg_max)) {
			dev_err(&vib_dev->spmi->dev, "Invalid voltage level\n");
			return -EINVAL;
		}
	} else {
		dev_err(&vib_dev->spmi->dev, "Voltage level not specified\n");
		return -EINVAL;
	}

	/* Configure the VTG CTL regiser */
	reg = vib_dev->reg_vtg_ctl;
	reg &= ~QPNP_VIB_VTG_SET_MASK;
	reg |= (level & QPNP_VIB_VTG_SET_MASK);
	rc = qpnp_vib_write_u8(vib_dev, &reg, QPNP_VIB_VTG_CTL(vib_dev->base));
	if (rc)
		return rc;
	vib_dev->reg_vtg_ctl = reg;

	/* Configure the VIB ENABLE regiser */
	reg = vib_dev->reg_en_ctl;
	reg |= (!!vib_cfg->active_low) << QPNP_VIB_LOGIC_SHIFT;
	if (vib_cfg->enable_mode == QPNP_VIB_MANUAL)
		reg |= QPNP_VIB_EN;
	else
		reg |= BIT(vib_cfg->enable_mode - 1);
	rc = qpnp_vib_write_u8(vib_dev, &reg, QPNP_VIB_EN_CTL(vib_dev->base));
	if (rc < 0)
		return rc;
	vib_dev->reg_en_ctl = reg;

	return rc;
}
EXPORT_SYMBOL(qpnp_vibrator_config);

static int qpnp_vib_set(struct qpnp_vib *vib, int on)
{
	int rc;
	u8 val;

	if (on) {
		val = vib->reg_vtg_ctl;
		val &= ~QPNP_VIB_VTG_SET_MASK;
		val |= (vib->vtg_level & QPNP_VIB_VTG_SET_MASK);
		rc = qpnp_vib_write_u8(vib, &val, QPNP_VIB_VTG_CTL(vib->base));
		if (rc < 0)
			return rc;
		vib->reg_vtg_ctl = val;
		val = vib->reg_en_ctl;
		val |= QPNP_VIB_EN;
		rc = qpnp_vib_write_u8(vib, &val, QPNP_VIB_EN_CTL(vib->base));
		//ASUS BSP freddy++ add Turn on vibrator and start Timer debug msg
			//[A86][vib][Fix][NA] add for fix sometimes no vibration when pad inserted.
	#if 0		 
		if (g_vib_stop_val > 100)
		{
			printk("[vibrator] Turn on vibrator, timer= %d ms\n",g_vib_stop_val);
		}	
	#endif	
		printk("[vibrator] Turn on vibrator, timer= %d ms\n",g_vib_stop_val);
		hrtimer_start(&vib->vib_timer,
			      ktime_set(g_vib_stop_val / 1000, (g_vib_stop_val % 1000) * 1000000),
			      HRTIMER_MODE_REL);		
		//ASUS BSP freddy-- Turn on vibrator and start Timer
		if (rc < 0)
			return rc;
		vib->reg_en_ctl = val;
	} else {
		val = vib->reg_en_ctl;
		val &= ~QPNP_VIB_EN;
		rc = qpnp_vib_write_u8(vib, &val, QPNP_VIB_EN_CTL(vib->base));
		printk("[vibrator] Turn off vibrator !\n");
		if (rc < 0)
			return rc;
		vib->reg_en_ctl = val;
	}

	return rc;
}

static void qpnp_vib_enable(struct timed_output_dev *dev, int value)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib,
					 timed_dev);
	unsigned long flags;

retry:
	spin_lock_irqsave(&vib->lock, flags);
	if (hrtimer_try_to_cancel(&vib->vib_timer) < 0) {
		spin_unlock_irqrestore(&vib->lock, flags);
		cpu_relax();
		goto retry;
	}

	if (value == 0)
		vib->state = 0;
	else {
		value = (value > vib->timeout ?
				 vib->timeout : value);
		vib->state = 1;
//ASUS BSP freddy++ "[A86][vib][Fix][NA] add for fix sometimes no vibration when pad inserted.  "
		g_vib_stop_val = value;
		/*
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
		*/
//ASUS BSP freddy-- "[A86][vib][Fix][NA] add for fix sometimes no vibration when pad inserted.  "		
	}
	qpnp_vib_set(vib, vib->state);

	spin_unlock_irqrestore(&vib->lock, flags);
}

static int qpnp_vib_get_time(struct timed_output_dev *dev)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib,
							 timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart qpnp_vib_timer_func(struct hrtimer *timer)
{
	struct qpnp_vib *vib = container_of(timer, struct qpnp_vib,
							 vib_timer);
	unsigned long flags;

	spin_lock_irqsave(&vib->lock, flags);

	vib->state = 0;
	printk("[vibrator] schedule_work to turn off\n");
	qpnp_vib_set(vib, vib->state);

	spin_unlock_irqrestore(&vib->lock, flags);

	return HRTIMER_NORESTART;
}

#ifdef CONFIG_PM
static int qpnp_vibrator_suspend(struct device *dev)
{
	struct qpnp_vib *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->vib_timer);
	/* turn-off vibrator */
	printk("[vibrator] %s\n",__func__);
	qpnp_vib_set(vib, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(qpnp_vibrator_pm_ops, qpnp_vibrator_suspend, NULL);

static int __devinit qpnp_vibrator_probe(struct spmi_device *spmi)
{
	struct qpnp_vib *vib;
	struct resource *vib_resource;
	int rc;
	u8 val;
	u32 temp_val;

	printk("[qpnp_vibrator] %s +++\n",__FUNCTION__);
	vib = devm_kzalloc(&spmi->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->spmi = spmi;

	vib->timeout = QPNP_VIB_DEFAULT_TIMEOUT;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-timeout-ms", &temp_val);
	if (!rc) {
		vib->timeout = temp_val;
	} else if (rc != EINVAL) {
		dev_err(&spmi->dev, "Unable to read vib timeout\n");
		return rc;
	}

	vib->vtg_level = QPNP_VIB_DEFAULT_VTG_LVL;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-vtg-level-mV", &temp_val);
	if (!rc) {
		vib->vtg_level = temp_val;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read vtg level\n");
		return rc;
	}

	vib->vtg_max = QPNP_VIB_DEFAULT_VTG_MAX;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-vtg-max-mV", &temp_val);
	if (!rc) {
		vib->vtg_max = min(temp_val, (u32)QPNP_VIB_DEFAULT_VTG_MAX);
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read vtg max level\n");
		return rc;
	}

	vib->vtg_min = QPNP_VIB_DEFAULT_VTG_MIN;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-vtg-min-mV", &temp_val);
	if (!rc) {
		vib->vtg_min = max(temp_val, (u32)QPNP_VIB_DEFAULT_VTG_MIN);
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read vtg min level\n");
		return rc;
	}

	vib->vtg_level /= 100;
	vib->vtg_min /= 100;
	vib->vtg_max /= 100;
	vib->vtg_default = vib->vtg_level;

	vib_resource = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!vib_resource) {
		dev_err(&spmi->dev, "Unable to get vibrator base address\n");
		return -EINVAL;
	}
	vib->base = vib_resource->start;

	/* save the control registers values */
	rc = qpnp_vib_read_u8(vib, &val, QPNP_VIB_VTG_CTL(vib->base));
	if (rc < 0)
		return rc;
	vib->reg_vtg_ctl = val;

	rc = qpnp_vib_read_u8(vib, &val, QPNP_VIB_EN_CTL(vib->base));
	if (rc < 0)
		return rc;
	vib->reg_en_ctl = val;

	spin_lock_init(&vib->lock);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = qpnp_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = qpnp_vib_get_time;
	vib->timed_dev.enable = qpnp_vib_enable;

	dev_set_drvdata(&spmi->dev, vib);

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		return rc;

	rc = device_create_file(vib->timed_dev.dev, &dev_attr_vtg_level);
	if (rc < 0)
		goto error_create_level;
	rc = device_create_file(vib->timed_dev.dev, &dev_attr_vtg_min);
	if (rc < 0)
		goto error_create_min;
	rc = device_create_file(vib->timed_dev.dev, &dev_attr_vtg_max);
	if (rc < 0)
		goto error_create_max;
	rc = device_create_file(vib->timed_dev.dev, &dev_attr_vtg_default);
	if (rc < 0)
		goto error_create_default;

	vib_dev = vib;
	printk("[qpnp_vibrator] %s ---\n",__FUNCTION__);

	return 0;

error_create_default:
	device_remove_file(vib->timed_dev.dev, &dev_attr_vtg_max);
error_create_max:
	device_remove_file(vib->timed_dev.dev, &dev_attr_vtg_min);
error_create_min:
	device_remove_file(vib->timed_dev.dev, &dev_attr_vtg_level);
error_create_level:
	timed_output_dev_unregister(&vib->timed_dev);
	return rc;
}
//ASUS_BSP +++ freddy "[A86][vib][NA][Spec] added to vibrator after phone inserted into pad"
static int mp_event_report(struct notifier_block *this, unsigned long event, void *ptr)
{
	
        switch (event)
	{
		case P01_ADD:
		{
			printk("[vibrator] PAD ADD vibrator enable!!\n");
			qpnp_vib_enable(&vib_dev->timed_dev,500);
			
			return NOTIFY_DONE;
		}
		case P01_REMOVE:
		{
			printk("[vibrator] PAD REMOVE vibrator !!\n");
			qpnp_vib_enable(&vib_dev->timed_dev,0);
			
		}
		default:
			
			return NOTIFY_DONE;
        }
}

static struct notifier_block mp_notifier = {
        .notifier_call = mp_event_report,
        .priority = VIBRATOR_MP_NOTIFY,
};
//ASUS_BSP --- freddy "[A86][vib][NA][Spec] added to vibrator after phone inserted into pad"

static int  __devexit qpnp_vibrator_remove(struct spmi_device *spmi)
{
	struct qpnp_vib *vib = dev_get_drvdata(&spmi->dev);

	hrtimer_cancel(&vib->vib_timer);
	device_remove_file(vib->timed_dev.dev, &dev_attr_vtg_level);
	device_remove_file(vib->timed_dev.dev, &dev_attr_vtg_min);
	device_remove_file(vib->timed_dev.dev, &dev_attr_vtg_max);
	device_remove_file(vib->timed_dev.dev, &dev_attr_vtg_default);
	timed_output_dev_unregister(&vib->timed_dev);

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "qcom,qpnp-vibrator",
	},
	{}
};

static struct spmi_driver qpnp_vibrator_driver = {
	.driver		= {
		.name	= "qcom,qpnp-vibrator",
		.of_match_table = spmi_match_table,
		.pm	= &qpnp_vibrator_pm_ops,
	},
	.probe		= qpnp_vibrator_probe,
	.remove		= __devexit_p(qpnp_vibrator_remove),
};

/*
qpnp-vibrator works on 
A86_SR1 ~ A86_SR4, 
after A86_ER(second source)depend on g_drv2605_probe_ok == 0
*/
static int __init qpnp_vibrator_init(void)
{
	printk("[qpnp_vibrator] init:  +++\n");
// ASUS BSP freddy+++ [A86][vib][spec][Others] "Turn off qpnp-vibrator after A86_SR4 "
	if ((g_ASUS_hwID <= A86_SR4) || (g_drv2605_probe_ok))
	{
		printk("[%s] dont support A86_SR4", __func__);	
		return 0;
	}
// ASUS BSP freddy--- [A86][vib][spec][Others] "Turn off qpnp-vibrator after A86_SR4 "	


//ASUS_BSP +++ freddy "[A86][vib][NA][Spec] added to vibrator after phone inserted into pad"
	register_microp_notifier(&mp_notifier);  
	notify_register_microp_notifier(&mp_notifier, "qpnp_vibrator"); 
//ASUS_BSP --- freddy "[A86][vib][NA][Spec] added to vibrator after phone inserted into pad"	
	return spmi_driver_register(&qpnp_vibrator_driver);
}
//ASUS_BSP +++ freddy"[A86][vib][NA][Spec] delay init for vibrator second source after A86_ER"
late_initcall(qpnp_vibrator_init);
//ASUS_BSP --- freddy"[A86][vib][NA][Spec] delay init for vibrator second source after A86_ER"

static void __exit qpnp_vibrator_exit(void)
{
	if ((g_ASUS_hwID <= A86_SR4) || (g_drv2605_probe_ok))
	{
		printk("[%s] dont support A86_SR4", __func__);	
		return ;
	}

//ASUS_BSP +++ freddy "[A86][vib][NA][Spec] added to vibrator after phone inserted into pad"
	unregister_microp_notifier(&mp_notifier);  
	notify_unregister_microp_notifier(&mp_notifier, "qpnp_vibrator"); 
//ASUS_BSP --- freddy "[A86][vib][NA][Spec] added to vibrator after phone inserted into pad"	
	return spmi_driver_unregister(&qpnp_vibrator_driver);
}
module_exit(qpnp_vibrator_exit);

MODULE_DESCRIPTION("qpnp vibrator driver");
MODULE_LICENSE("GPL v2");

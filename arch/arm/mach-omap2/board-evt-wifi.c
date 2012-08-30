/* linux/arch/arm/mach-omap2/board-zoom2-wifi.c
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wifi_tiwlan.h>

#ifdef CONFIG_WLAN_POWER_EVT1
#include <linux/i2c/twl.h>

#define EVT_WIFI_IRQ_GPIO		15
#define EVT_WIFI_ENPOW_GPIO		16
#define EVT_WIFI_PMENA_GPIO		22

#define PM_RECEIVER                     TWL4030_MODULE_PM_RECEIVER
#define ENABLE_VAUX2_DEDICATED          0x05
#define ENABLE_VAUX2_DEV_GRP            0x20
#define ENABLE_VAUX3_DEDICATED          0x03
#define ENABLE_VAUX3_DEV_GRP            0x20

#define t2_out(c, r, v) twl_i2c_write_u8(c, r, v)

#define t2_out(c, r, v) twl_i2c_write_u8(c, r, v)
#endif

static int evt_wifi_cd;		/* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int wifi_v18io_power_enable_init(void)
{
	int return_value = 0;
#ifdef CONFIG_WLAN_POWER_EVT1
	printk("Enabling VAUX for wifi \n");
	return_value  = t2_out(PM_RECEIVER,ENABLE_VAUX2_DEDICATED,TWL4030_VAUX2_DEDICATED);
	return_value |= t2_out(PM_RECEIVER,ENABLE_VAUX2_DEV_GRP,TWL4030_VAUX2_DEV_GRP);
	if(0 != return_value)
		printk("Enabling VAUX for wifi incomplete error: %d\n",return_value);
#endif
	return return_value;
}


int omap_wifi_status_register(void (*callback)(int card_present,
						void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

int omap_wifi_status(int irq)
{
	return evt_wifi_cd;
}

int evt_wifi_set_carddetect(int val)
{
	pr_info("%s: %d\n", __func__, val);
	evt_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_info("%s: Nobody to notify\n", __func__);
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(evt_wifi_set_carddetect);
#endif

static int evt_wifi_power_state;

int evt_wifi_power(int on)
{
	pr_info("%s: %d\n", __func__, on);
	gpio_set_value(EVT_WIFI_PMENA_GPIO, on);
	evt_wifi_power_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(evt_wifi_power);
#endif

static int evt_wifi_reset_state;
int evt_wifi_reset(int on)
{
	pr_info("%s: %d\n", __func__, on);
	evt_wifi_reset_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(evt_wifi_reset);
#endif

struct wifi_platform_data evt_wifi_control = {
	.set_power	= evt_wifi_power,
	.set_reset	= evt_wifi_reset,
	.set_carddetect	= evt_wifi_set_carddetect,
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct resource evt_wifi_resources[] = {
	[0] = {
		.name		= "device_wifi_irq",
		.start		= OMAP_GPIO_IRQ(EVT_WIFI_IRQ_GPIO),
		.end		= OMAP_GPIO_IRQ(EVT_WIFI_IRQ_GPIO),
		.flags		= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device evt_wifi_device = {
	.name		= "device_wifi",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(evt_wifi_resources),
	.resource	= evt_wifi_resources,
	.dev		= {
		.platform_data = &evt_wifi_control,
	},
};
#endif

static int __init evt_wifi_init(void)
{
	int ret;

	pr_info("%s: start\n", __func__);
	ret = gpio_request(EVT_WIFI_IRQ_GPIO, "wifi_irq");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			EVT_WIFI_IRQ_GPIO);
		goto out;
	}
	ret = gpio_request(EVT_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			EVT_WIFI_PMENA_GPIO);
		gpio_free(EVT_WIFI_IRQ_GPIO);
		goto out;
	}
	ret = gpio_request(EVT_WIFI_ENPOW_GPIO, "wifi_en_pow");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			EVT_WIFI_ENPOW_GPIO);
		gpio_free(EVT_WIFI_ENPOW_GPIO);
		goto out;
	}
	wifi_v18io_power_enable_init();
	gpio_direction_input(EVT_WIFI_IRQ_GPIO);
	gpio_direction_output(EVT_WIFI_PMENA_GPIO, 0);
	gpio_direction_output(EVT_WIFI_ENPOW_GPIO, 1);

#ifdef CONFIG_WIFI_CONTROL_FUNC
	ret = platform_device_register(&evt_wifi_device);
#endif
out:
	return ret;
}

device_initcall(evt_wifi_init);

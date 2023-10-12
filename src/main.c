/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/lbs.h>

#include <zephyr/settings/settings.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>

#include <zephyr/drivers/sensor.h>


#define DISPLAY_BUFFER_PITCH 128
//#define ENABLE_LSM6DSO

uint8_t buf2[512] = {
// 'freg', 128x32px
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x07, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x07, 0x07, 0x07, 0x0f, 0x1f, 
0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0x7f, 0x1f, 0x03, 0x01, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xef, 0xef, 
0xf3, 0x80, 0xf0, 0xe0, 0xf8, 0xf0, 0xf0, 0xe0, 0xe0, 0xc0, 0xc0, 0xc0, 0x80, 0x80, 0x80, 0x80, 
0x80, 0x80, 0x80, 0xc0, 0xc0, 0xe0, 0xf0, 0x30, 0x08, 0x80, 0x00, 0xc0, 0xf0, 0xf0, 0xf8, 0xfc, 
0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xdb, 0xeb, 0xc1, 0xc1, 0xe0, 0xf3, 0xfd, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

#ifdef DISPLAY
static const struct device *display = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
#endif
/*
todo:
RGB LED - minimally done
LUX METER
ADC
IMU
DISPLAY - is doing something
SHUTTER
SHOT LOGGING
SLEEP MODE
BLE STACK -  dfu + advertising
*/

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define RUN_LED_BLINK_INTERVAL	1000

LOG_MODULE_REGISTER(Ricoh_R1S, CONFIG_LOG_DEFAULT_LEVEL);

#define LED_COMM DT_ALIAS(led0)
#define LED_R DT_ALIAS(led1)
#define LED_B DT_ALIAS(led2)
#define LED_G DT_ALIAS(led3)

static struct gpio_dt_spec mled0 = GPIO_DT_SPEC_GET_OR(LED_COMM, 	gpios,{0});
static struct gpio_dt_spec mled1 = GPIO_DT_SPEC_GET_OR(LED_R, 		gpios,{0});
static struct gpio_dt_spec mled2 = GPIO_DT_SPEC_GET_OR(LED_G, 		gpios,{0});
static struct gpio_dt_spec mled3 = GPIO_DT_SPEC_GET_OR(LED_B, 		gpios,{0});


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/*static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
		      0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d),
};*/

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};


static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}
	gpio_pin_set_dt(&mled2, 1);
	gpio_pin_set_dt(&mled3, 0);
	LOG_INF("Connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	gpio_pin_set_dt(&mled2, 0);
	gpio_pin_set_dt(&mled3, 1);
	LOG_INF("Disconnected (reason %u)", reason);
}

#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_ERR("Security failed: %s level %u err %d", addr, level,
			err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_LBS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_ERR("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void app_led_cb(bool led_state)
{
	//dk_set_led(USER_LED, led_state);
}

static bool app_button_cb(void)
{
	//return app_button_state;
}

static struct bt_lbs_cb lbs_callbacs = {
	.led_cb    = app_led_cb,
	.button_cb = app_button_cb,
};

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	/*if (has_changed & USER_BUTTON) {
		uint32_t user_button_state = button_state & USER_BUTTON;

		bt_lbs_send_button_state(user_button_state);
		app_button_state = user_button_state ? true : false;
	}*/
}

static int init_button(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		printk("Cannot init buttons (err: %d)\n", err);
	}

	return err;
}

int config_leds(){

	int ret;

	if (!device_is_ready(mled0.port)) {
		LOG_ERR("Error: LED device is not ready %s !", mled0.port->name);
		return -1;
	}
	ret = gpio_pin_configure_dt(&mled0, GPIO_OUTPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure LED device");
		return -1;
	}

	if (!device_is_ready(mled1.port)) {
		LOG_ERR("Error: LED device is not ready %s !", mled1.port->name);
		return -1;
	}
	ret = gpio_pin_configure_dt(&mled1, GPIO_OUTPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure LED device");
		return -1;
	}

	if (!device_is_ready(mled2.port)) {
		LOG_ERR("Error: LED device is not ready %s !", mled2.port->name);
		return -1;
	}
	ret = gpio_pin_configure_dt(&mled2, GPIO_OUTPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure LED device");
		return -1;
	}

	if (!device_is_ready(mled3.port)) {
		LOG_ERR("Error: LED device is not ready %s !", mled3.port->name);
		return -1;
	}
	ret = gpio_pin_configure_dt(&mled3, GPIO_OUTPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure LED device");
		return -1;
	}
		
	gpio_pin_set_dt(&mled0, 1);
	gpio_pin_set_dt(&mled1, 0);	
	gpio_pin_set_dt(&mled2, 0);
	gpio_pin_set_dt(&mled3, 0);

	return 0;

}

#ifdef ENABLE_LSM6DSO

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

static void fetch_and_display(const struct device *dev)
{
	struct sensor_value x, y, z;
	static int trig_cnt;

	trig_cnt++;

	// lsm6dso accel 
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &z);

	LOG_INF("accel x:%f ms/2 y:%f ms/2 z:%f ms/2",	out_ev(&x), out_ev(&y), out_ev(&z));

	// lsm6dso gyro 
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &z);

	LOG_INF("gyro x:%f rad/s y:%f rad/s z:%f rad/s",
			out_ev(&x), out_ev(&y), out_ev(&z));

	LOG_INF("trig_cnt:%d\n", trig_cnt);
}

static int set_sampling_freq(const struct device *dev)
{
	int ret = 0;
	struct sensor_value odr_attr;

	// set accel/gyro sampling frequency to 12.5 Hz 
	odr_attr.val1 = 12.5;
	odr_attr.val2 = 0;

	ret = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret != 0) {
		LOG_INF("Cannot set sampling frequency for accelerometer.");
		return ret;
	}

	ret = sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret != 0) {
		LOG_INF("Cannot set sampling frequency for gyro.");
		return ret;
	}

	return 0;
}

#endif

void main(void)
{
	int err;

	LOG_INIT();
	LOG_INF("Ricoh_R1s Starting");

	if(config_leds()){
		LOG_ERR("Couldn't configure LEDs");
	}
#ifdef ENABLE_LSM6DSO
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(lsmdso));

	if (!device_is_ready(dev)) {
		printk("%s: device not ready.", dev->name);
		//return;
	}

	LOG_INF("Testing LSM6DSO sensor in polling mode.\n");
	if (set_sampling_freq(dev) != 0) {
		//return;
	}

	fetch_and_display(dev);

#endif

#ifdef DISPLAY

	if (display == NULL) {
		LOG_ERR("device pointer is NULL");
		return;
	}

	if (!device_is_ready(display)) {
		LOG_ERR("display device is not ready");
		return;
	}

	struct display_capabilities capabilities;
	display_get_capabilities(display, &capabilities);

	const uint16_t x_res = capabilities.x_resolution;
	const uint16_t y_res = capabilities.y_resolution;

	LOG_DBG("x_resolution: %d", x_res);
	LOG_DBG("y_resolution: %d", y_res);
	LOG_DBG("supported pixel formats: %d", capabilities.supported_pixel_formats);
	LOG_DBG("screen_info: %d", capabilities.screen_info);
	LOG_DBG("current_pixel_format: %d", capabilities.current_pixel_format);
	LOG_DBG("current_orientation: %d", capabilities.current_orientation);
		
	const struct display_buffer_descriptor buf_desc = {
		.width = x_res,
		.height = y_res,
		.buf_size = x_res * y_res,
		.pitch = DISPLAY_BUFFER_PITCH
	};

	if (display_write(display, 0, 0, &buf_desc, buf2) != 0) {
		LOG_ERR("could not write to display");
	}

	if (display_set_contrast(display, 0) != 0) {
		LOG_ERR("could not set display contrast");
	}
	size_t ms_sleep = 5;

#endif

	if (IS_ENABLED(CONFIG_BT_LBS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks.");
			return;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks.");
			return;
		}
	}

	LOG_INF("build time: " __DATE__ " " __TIME__);

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_lbs_init(&lbs_callbacs);
	if (err) {
		LOG_ERR("Failed to init LBS (err:%d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");


	while(1) {

		if (LOG_PROCESS() == false) {
			gpio_pin_toggle_dt(&mled1);
#ifdef ENABLE_LSM6DSO
			fetch_and_display(dev);
#endif
			k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
		}
	}
}

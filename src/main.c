
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <stddef.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <hal/nrf_gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
// adc

#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_power.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>
#include "ble.h"
#include "ws2812.h"
/* NFC */
#include <stdio.h>
#include <nfc_t2t_lib.h>
#include <nfc/ndef/msg.h>
#include <nfc/ndef/text_rec.h>
#include <nfc/ndef/uri_msg.h>
#include <nfc/ndef/record.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
// #include "sys_off.h"
#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
// #error "No suitable devicetree overlay specified"
#endif
#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct adc_dt_spec adc_channels[] = {DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)};

// #define BNO055_INT_PIN 11
#define INT_EN 0x10
#define INT_STATUS 0x37
#define INT_MASK 0x0F
#define OPAR_MODE_REG 0x3d
#define NDOF 0x0c
#define ACC_INT_Settings 0x12
#define PAGE_ID 0x07
#define BNO055_INT_STA_ADDR 0X37

#define SLEEP_TIME_MS 1000
#define ACC_NM_INT (1 << 1)

// #define ACC_AM_MSK (1 << 0)
#define ACC_NM_MSK (1 << 1)
#define ACC_INT_SETTINGS 0x12
#define ACC_NM_THRES 0x03
#define ACC_NM_SET (1 << 0) | (30 << 1)
// 0x15
#define ACCONLY 0x01

// int wdt_channel_id;

// command
#define SYS_TRIGGER 0x3f
#define ACC_AM_THRES 0x12
#define POWER_MODE_ADDR 0x3E
// #define ACC_AM_DUR
#define RST_INT 0x40
#define ACC_AM_MSK 0x40
#define ACC_AM_INT 0x40
#define ACC_HIGH_G 0x20
#define BNO055_i2C_ADDR 0x29
// #define AM_NM_X_AXIS 0x04
#define SW0_NODE DT_ALIAS(sw0)
#define MY_STACK_SIZE 1024
#define MY_PRIORITY 5

/* battery */
int32_t bat_voltage = 0;
uint8_t soc = 0;

const struct device *wdt = DEVICE_DT_GET(DT_NODELABEL(wdt31));
K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);
struct k_thread my_thread_data;
uint8_t r = 0, g = 50, b = 0;

uint8_t buf;
uint8_t count = 0;
int8_t in_Hand_Threshold = 3;
/* NFC */
#define MAX_REC_COUNT 1
#define NDEF_MSG_BUF_SIZE 256

static void nfc_callback(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t data_len)
{
	switch (event)
	{
	case NFC_T2T_EVENT_FIELD_ON:
		printf("NFC tag Field event\n");
		break;
	case NFC_T2T_EVENT_FIELD_OFF:
		printf("NFC tag write event\n");
		break;
	default:
		printf("Unknown NFC event\n");
		break;
	}
}
bool nfc_flag = 1;
void nfc_write(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	static const uint8_t uri[] = "mlworkx.com";
	uint8_t ndef_mdg_buf[128];
	size_t len = sizeof(ndef_mdg_buf);

	nfc_ndef_uri_msg_encode(
		NFC_URI_HTTP_WWW,
		uri,
		sizeof(uri) - 1,
		ndef_mdg_buf,
		&len);

	nfc_t2t_setup(nfc_callback, NULL);
	nfc_t2t_payload_set(ndef_mdg_buf, len);
	nfc_t2t_emulation_start();

	while (nfc_flag == 1)
	{
		k_sleep(K_MSEC(100));
	}

	nfc_t2t_emulation_stop();
	nfc_t2t_done();
}

static const struct gpio_dt_spec Interrupt = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static const struct device *i2c = DEVICE_DT_GET(DT_ALIAS(i2c1));
float ax, ay, az, mx, my, gx, gy, gz, mz, qw, qx, qy, qz, lax, lay, laz, heading, roll, pitch, gvx, gvy, gvz;
int16_t axis_diff_xp = 0;
int16_t axis_diff_xa = 0;
int16_t axis_diff_yp = 0;
int16_t axis_diff_ya = 0;
int16_t axis_diff_zp = 0;
int16_t axis_diff_za = 0;
bool bno055_Eflag = 0;
void read_bno055(void)
{
	uint8_t buf[38];
	k_msleep(1000);
	if (i2c_burst_read(i2c, BNO055_i2C_ADDR, 0x08, buf, 38) < 0)
	{
		printk("Read error!\n");
		bno055_Eflag = 1;
		led_status = 7;
		k_msleep(2000);
	}
	else
	{
		bno055_Eflag = 0;
	}
	ax = (buf[1] << 8) | buf[0];
	ay = (buf[3] << 8) | buf[2];
	az = (buf[5] << 8) | buf[4];
	mx = (buf[7] << 8) | buf[6];
	my = (buf[9] << 8) | buf[8];
	mz = (buf[11] << 8) | buf[10];
	gx = (buf[13] << 8) | buf[12];
	gy = (buf[15] << 8) | buf[14];
	gz = (buf[17] << 8) | buf[16];
	heading = (buf[19] << 8) | buf[18];
	roll = (buf[21] << 8) | buf[20];
	pitch = (buf[23] << 8) | buf[22];
	qw = (buf[25] << 8) | buf[24];
	qx = (buf[27] << 8) | buf[26];
	qy = (buf[29] << 8) | buf[28];
	qz = (buf[31] << 8) | buf[30];
	lax = (buf[33] << 8) | buf[32];
	lay = (buf[35] << 8) | buf[34];
	laz = (buf[37] << 8) | buf[36];
	// gvx = (buf[39] << 8) | buf[38];
	// gvy = (buf[41] << 8) | buf[40];
	// gvz = (buf[43] << 8) | buf[42];

	ax = ax / 100.0f;
	ay = ay / 100.0f;
	az = az / 100.0f;
	mx = mx / 16.0f;
	my = my / 16.0f;
	mz = mz / 16.0f;
	gx = gx / 16.0f;
	gy = gy / 16.0f;
	gz = gz / 16.0f;
	heading = heading / 16.0f;
	roll = roll / 16.0f;
	pitch = pitch / 16.0f;
	qw = qw / 16384.0f;
	qx = qx / 16384.0f;
	qy = qy / 16384.0f;
	qz = qz / 16384.0f;
	lax = lax / 100.0f;
	lay = lay / 100.0f;
	laz = laz / 100.0f;
	// gvx = gvx/100.0f;
	// gvy = gvx/100.0f;
	// gvz = gvx/100.0f;

	axis_diff_xa = axis_diff_xp - ax;
	axis_diff_xp = ax;
	axis_diff_ya = axis_diff_yp - ay;
	axis_diff_yp = ay;
	axis_diff_za = axis_diff_zp - az;
	axis_diff_zp = az;
}
void send_all_bno055(void)
{
	read_bno055();
	// Devided voltage on 4.2v is 2.166
	// Devided voltage on 3.2v is 1.604
	// Devided voltage at 1v is 0.5012919896640827
	memset(data, 0, sizeof(data));
	data[0] = 0xaa; // header
	data[1] = 0x01; // header all data
	data[2] = 0x00; // reserved
	data[3] = 0x01; // version
	data[4] = soc;
	data[6] = bat_voltage / 100;
	data[8] = 0x00;
	memcpy(&data[9], &ax, sizeof(float)); // accel float
	memcpy(&data[13], &ay, sizeof(float));
	memcpy(&data[17], &az, sizeof(float));
	memcpy(&data[21], &gx, sizeof(float)); // gyro float
	memcpy(&data[25], &gy, sizeof(float));
	memcpy(&data[29], &gz, sizeof(float));
	memcpy(&data[33], &qw, sizeof(float)); // Quaterion code
	memcpy(&data[37], &qx, sizeof(float));
	memcpy(&data[41], &qy, sizeof(float));
	memcpy(&data[45], &qz, sizeof(float));
	memcpy(&data[49], &heading, sizeof(float)); // Eular float
	memcpy(&data[53], &roll, sizeof(float));
	memcpy(&data[57], &pitch, sizeof(float));
	// memcpy(&data[61], &gvx, sizeof(float));
	// memcpy(&data[65], &gvy, sizeof(float));
	// memcpy(&data[69], &gvz, sizeof(float));
	memcpy(&data[61], &led_status, sizeof(int8_t)); // status code
	data[62] = 0x55;
	if (ble_connected)
	{
		notify_data();
	}
}

uint8_t blink = 1;
bool led_flag = 0;

void ws2812_led()
{
	/* The format of payload is:
	BYTE 0: HEADER
	BYTE 1: RED INTENSITY
	BYTE 2: GREEN INTENSITY
	BYTE 3: BLUE INTENSITY
	BYTE 4: BLINK
	BYTE 5: FOOTER
	*/
	// Make sure that the buffer received is of 6 bytes
	// and has the desired header and footer
	while (1)
	{
		if (request_recieved)
		{
			// reset flag
			request_recieved = false;

			// check validity of request
			if ((recieved_payload_length == 6) && (buf_ptr[recieved_payload_length - 1] == 0x55) && (buf_ptr[0] == 0xAA))
			{
				printk("VALID PAYLOAD");

				// Update R G B Blink values
				r = buf_ptr[1];
				g = buf_ptr[2];
				b = buf_ptr[3];
				blink = buf_ptr[4];

				// set led status as 3
				led_status = 3;
			}
			else
			{
				printk("INVALID PAYLOAD");
			}
		}
		if (led_flag)
		{
			led_status = 3;
		}
		printk("ws2812_led status: %d\n", led_status);
		if (led_status == 0)
		{
			ws2812_set_colors(50, 50, 50);
		}
		else if (led_status == 1)
		{
			ws2812_set_colors(0, 0, 50);
		}
		else if (led_status == 2)
		{
			ws2812_set_colors(0, 50, 0);
			// led_status = 3;
		}
		else if (led_status == 3)
		{
			ws2812_set_colors(r, g, b);
			led_flag = 1;
		}
		else if (led_status == 4)
		{
			ws2812_set_colors(50, 0, 0);
		}
		else if (led_status == 5)
		{
			ws2812_set_colors(0, 0, 0);
		}
		else if (led_status == 6)
		{
			ws2812_set_colors(0xff, 0, 0xff);
		}
		else if (led_status == 7)
		{
			ws2812_set_colors(99, 78, 00);
		}
		if (blink)
		{
			k_msleep(500);
			ws2812_set_colors(0, 0, 0);
			k_msleep(500);
		}
		k_msleep(100);
	}
}

#define PDN_PIN NRF_GPIO_PIN_MAP(1, 13) // P1.13
#define RX_PIN NRF_GPIO_PIN_MAP(1, 9)	// P1.9
#define TX_PIN NRF_GPIO_PIN_MAP(1, 6)	// P1.6
#define ANT_PIN NRF_GPIO_PIN_MAP(1, 10) // P1.10
static void fem_Init_54dk_cust()
{
	nrf_gpio_cfg_output(PDN_PIN); // PDN
	// nrf_gpio_cfg_output(17); // MODE/CSD
	nrf_gpio_cfg_output(TX_PIN);  // TX
	nrf_gpio_cfg_output(RX_PIN);  // RX
	nrf_gpio_cfg_output(ANT_PIN); // Clear=20 Set=10
	nrf_gpio_pin_set(23);		  // PDN
	nrf_gpio_pin_set(17);		  // MODE/CSD
};
static void fem_Tx_54_20()
{
	printf("Rx is disable\n");
	nrf_gpio_pin_clear(RX_PIN); // RX
	nrf_gpio_pin_set(ANT_PIN);	// Clear=20 Set=10
	nrf_gpio_pin_set(TX_PIN);	// TX
	printf("Tx is Enabled\n");
};
static void fem_Tx_54_10()
{
	printf("Rx is disable\n");
	nrf_gpio_pin_clear(RX_PIN);	 // RX
	nrf_gpio_pin_clear(ANT_PIN); // Clear=20 Set=10
	nrf_gpio_pin_set(TX_PIN);	 // TX
	printf("Tx is Enabled\n");
};

int bat_pwr_init()
{
	int err;

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++)
	{
		if (!adc_is_ready_dt(&adc_channels[i]))
		{
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return 0;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0)
		{
			printk("Could not setup channel #%d (%d)\n", i, err);
			return 0;
		}
	}
	return 0;
}
void bat_read()
{
	uint8_t err;
	uint16_t adc_buf;
	struct adc_sequence sequence = {
		.buffer = &adc_buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(adc_buf),
	};
	printk("ADC reading[%u]:\n", count++);
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++)
	{
		int32_t val_mv;

		printk("- %s, channel %d: ",
			   adc_channels[i].dev->name,
			   adc_channels[i].channel_id);
		(void)adc_sequence_init_dt(&adc_channels[i], &sequence);
		err = adc_read(adc_channels[i].dev, &sequence);
		if (err < 0)
		{
			printk("Could not read (%d)\n", err);
			continue;
		}
		/*
		 * If using differential mode, the 16 bit value
		 * in the ADC sample buffer should be a signed 2's
		 * complement value.
		 */
		if (adc_channels[i].channel_cfg.differential)
		{
			val_mv = (int32_t)((int16_t)adc_buf);
		}
		else
		{
			val_mv = (int32_t)adc_buf;
		}
		printk("%" PRId32, val_mv);
		err = adc_raw_to_millivolts_dt(&adc_channels[i],
									   &val_mv);
		/* conversion to mV may not be supported, skip if not */
		if (err < 0)
		{
			printk(" (value in mV not available)\n");
		}
		else
		{
			printk(" = %" PRId32 " mV\n", val_mv);
		}
		if (val_mv > 2166)
		{
			val_mv = 2166;
		}
		else if (val_mv < 1604)
		{
			val_mv = 1604;
		}
		soc = ((val_mv - 1604) * 100 / (2166 - 1604));
		bat_voltage = val_mv;
		printk("\nCharged = %d per", soc);
	}
}

void print_reset_cause(uint32_t reset_cause)
{
	if (reset_cause & RESET_DEBUG)
	{
		printf("Reset by debugger.\n");
	}
	else if (reset_cause & RESET_CLOCK)
	{
		printf("Wakeup from System OFF by GRTC.\n");
	}
	else if (reset_cause & RESET_LOW_POWER_WAKE)
	{
		printf("Wakeup from System OFF by GPIO.\n");
	}
	else
	{
		printf("Other wake up cause 0x%08X.\n", reset_cause);
	}
}

int sys_off()
{

	uint32_t reset_cause;
	hwinfo_get_reset_cause(&reset_cause);
	print_reset_cause(reset_cause);
	hwinfo_clear_reset_cause();
	sys_poweroff();
	return 0;
}

uint8_t read_int_status()
{
	uint8_t int_status;
	int ret = i2c_burst_read(i2c, BNO055_i2C_ADDR, BNO055_INT_STA_ADDR, &int_status, 1);
	if (ret < 0)
	{
		printf("I2C read error: %d\n", ret);
		return ret;
	}
	return int_status;
}

int wdt_channel_id;
void watch_dog_timer_init()
{
	struct wdt_timeout_cfg wdt_config = {
		.flags = WDT_FLAG_RESET_SOC, // Reset SoC on timeout
		.window.min = 0,			 // Minimum time before feeding (usually 0)
		.window.max = 10000,		 // Timeout period in milliseconds
		.callback = NULL,			 // Optional: function to run before reset
	};
	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	// uint8_t options = WDT_OPT_PAUSE_IN_SLEEP;
	wdt_setup(wdt, NULL);
};

void bno055_enter_low_power(void)
{
	/* Low_power */
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, OPAR_MODE_REG, 0x00);
	k_msleep(20);

	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, POWER_MODE_ADDR, 0x01);
	k_msleep(20);
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, PAGE_ID, 1);
	k_msleep(20);

	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, INT_EN, ACC_AM_INT | ACC_NM_INT);
	k_msleep(20);

	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, INT_MASK, 0x00);
	k_msleep(20);

	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, ACC_AM_THRES, 0x0E);
	k_msleep(20);

	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, ACC_INT_SETTINGS, 0b00011110);
	k_msleep(20);

	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, ACC_NM_THRES, 0x03);
	k_msleep(20);

	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, ACC_NM_SET, (1 << 0) | (30 << 1));
	k_msleep(20);

	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, PAGE_ID, 0);
	k_msleep(20);
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, OPAR_MODE_REG, ACCONLY);
	k_msleep(20);
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, SYS_TRIGGER, 0x40);
}
// #define ACC_AM_INT (1 << 0)

int main(void)
{
	start_time = k_uptime_get();
	int err;
	watch_dog_timer_init();

	ws2812_init();
	k_msleep(100);

	k_thread_create(&my_thread_data, my_stack_area, K_THREAD_STACK_SIZEOF(my_stack_area), ws2812_led, NULL, NULL, NULL, MY_PRIORITY, 0, K_NO_WAIT);
	bat_pwr_init();
	bat_read();

	if (soc < 20)
	{
		// i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, OPAR_MODE_REG, 0x00); // enter CONFIGMODE (verified)
		// k_msleep(20);
		// // // Go to page 1 set of registers
		// i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, PAGE_ID, 1); // (verified)
		// k_msleep(20);
		// i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, INT_EN, ACC_AM_INT); // INT enable (verified)
		// k_msleep(20);
		// i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, ACC_AM_THRES, 0x0e); // Threshold
		// k_msleep(20);
		// i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, INT_MASK, ACC_AM_MSK); // INT_MSK (verified)
		// k_msleep(20);
		// i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, PAGE_ID, 0); // Exiting page_ID (verified)
		// k_msleep(20);
		// i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, OPAR_MODE_REG, NDOF); // NDOF mode (fusion mode) (verified)
		// k_msleep(20);
		// i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, SYS_TRIGGER, 0x40); // Resetting Int pin
		// gpio_pin_configure_dt(&Interrupt, GPIO_INPUT);
		// gpio_pin_interrupt_configure_dt(&Interrupt, GPIO_INT_LEVEL_HIGH);

		uint8_t low_pwr_count = 0;
		while (low_pwr_count < 4)
		{
			led_status = 4;
			low_pwr_count++;
			k_msleep(1000);
		}
		// if (read_int_status())
		// {
		// 	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, SYS_TRIGGER, 0x40); // Resetting Int pin
		// }
		// else
		// {
		// 	led_status = 5;
		// 	k_msleep(1000);
		// 	nfc_flag = 0;
		// 	sys_off();
		// }
	}

	fem_Init_54dk_cust();
	fem_Tx_54_20();
	err = ble_init();
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
	}
	start_advertising();

	/* Old AM&BNo055 configuration */
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, OPAR_MODE_REG, 0x00); // enter CONFIGMODE (verified)
	k_msleep(20);
	// // Go to page 1 set of registers
	// to set int_en, acc_am_threshold, acc_am_settings registers
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, PAGE_ID, 1); // (verified)
	k_msleep(20);
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, INT_EN, ACC_AM_INT); // INT enable (verified)
	k_msleep(20);
	// -- part start --
	// This part is mislabelled: ACC_AM_THRES is the settings register here, not the threshold
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, ACC_AM_THRES, 0x0e); // Threshold
	k_msleep(20);
	// -- part end --
	// PROBLEM: Actual threshold was never set
	// SPACE HERE to set the actual threshold of the accelerometer at reg 0x11
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, INT_MASK, ACC_AM_MSK); // INT_MSK (verified)
	k_msleep(20);
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, PAGE_ID, 0); // Exiting page_ID (verified)
	k_msleep(20);
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, OPAR_MODE_REG, NDOF); // NDOF mode (fusion mode) (verified)
	k_msleep(20);
	i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, SYS_TRIGGER, 0x40); // Resetting Int pin

	/* Down part is good */

	while (1)
	{
		wdt_feed(wdt, wdt_channel_id);
		bat_read();
		send_all_bno055();
		k_msleep(100);
		int val_time = k_uptime_get();
		// nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_HIGH);
		printk("current time %d", val_time);
		if (!ble_connected && val_time > start_time + 10000 && bno055_Eflag == 0)
		{
			// bno055_enter_low_power();
			led_flag = 0;
			blink = 0;
			gpio_pin_configure_dt(&Interrupt, GPIO_INPUT);
			if (read_int_status())
			{
				i2c_reg_write_byte(i2c, BNO055_i2C_ADDR, SYS_TRIGGER, 0x40); // Resetting Int pin
			}
			else
			{
				gpio_pin_interrupt_configure_dt(&Interrupt, GPIO_INT_LEVEL_HIGH);
				ble_deinit();
				led_status = 5;
				ws2812_set_colors(0, 0, 0);
				k_msleep(500);
				nfc_flag = 0;
				sys_off();
			}
		}
		// else if (ble_connected)
		else if (ble_connected || axis_diff_xa > in_Hand_Threshold || axis_diff_ya > in_Hand_Threshold || axis_diff_ya < -in_Hand_Threshold || axis_diff_xa < -in_Hand_Threshold || axis_diff_ya < -in_Hand_Threshold || axis_diff_ya < -in_Hand_Threshold)
		{
			start_time = k_uptime_get();
		}
		else
		{
			continue;
		}
	}

	return 0;
}
K_THREAD_DEFINE(
	nfc_thread_id,
	1024,
	nfc_write,
	NULL, NULL, NULL,
	5,
	0,
	0);
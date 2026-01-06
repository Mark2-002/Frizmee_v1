#include <inttypes.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>


// static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);


void print_reset_cause(uint32_t reset_cause)
{
	if (reset_cause & RESET_DEBUG) {
		printf("Reset by debugger.\n");
	} else if (reset_cause & RESET_CLOCK) {
		printf("Wakeup from System OFF by GRTC.\n");
	} else if (reset_cause & RESET_LOW_POWER_WAKE) {
		printf("Wakeup from System OFF by GPIO.\n");
	} else  {
		printf("Other wake up cause 0x%08X.\n", reset_cause);
	}
}

int sys_off (void)
{
	int rc;
	uint32_t reset_cause;
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (!device_is_ready(cons)) {
		printf("%s: device not ready.\n", cons->name);
		return 0;
	}

	hwinfo_get_reset_cause(&reset_cause);
	print_reset_cause(reset_cause);

	// Override sw0 or button0 in the dts to use the UART TX pin
	// of the frizbee schematic (P1.04)


	// Configure the GPIO pin to receive an interrupt
	rc = gpio_pin_configure_dt(&sw0, GPIO_INPUT);
	if (rc < 0) {
		printf("Could not configure sw0 GPIO (%d)\n", rc);
		return 0;
	}

	rc = gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_LEVEL_ACTIVE);
	if (rc < 0) {
		printf("Could not configure sw0 GPIO interrupt (%d)\n", rc);
		return 0;
	}

	// Prompt user
	printf("Entering system off; press sw0 to restart\n");

	rc = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
	if (rc < 0) {
		printf("Could not suspend console (%d)\n", rc);
		return 0;
	}

	hwinfo_clear_reset_cause();
	sys_poweroff();
	// The code section will never reach here as the system reboots as the GPIO comes

	return 0;
}

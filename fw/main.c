#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>

#include "uxb_locm3.h"

#define LED_PORT GPIOB
#define LED_PIN GPIO3

#define I2C_TIMEOUT 100000


UxbMasterLocm3 uxb;
UxbInterface iface1;

UxbSlot slot1;
uint8_t slot1_buffer[32];

UxbSlot descriptor_slot;
uint8_t descriptor_slot_buffer[64];

#define UXB_DESCRIPTOR_SIZE 6
const char *uxb_descriptor[] = {
	"device=tomato-single-cell",
	"hw-version=1.0.0+20171011",
	"fw-version=0.1.0",
	"cspeed=1",
	"dspeed=1",
	"interface=battery",
};

const uint8_t uxb_iface1_address[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22
};

const uint8_t uxb_remote_address[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

int32_t bat_voltage;
int32_t bat_current;
int32_t bat_charge;
int32_t bat_temp;

extern void initialise_monitor_handles(void);

static uxb_master_locm3_ret_t uxb_read_descriptor(void *context, uint8_t *buf, size_t len) {
	(void)context;

	uint8_t zero = 0;

	if (len != 1) {
		return UXB_MASTER_LOCM3_RET_FAILED;
	}
	if (buf[0] == 0) {
		/* Send the 0 back. */
		uxb_slot_send_data(&descriptor_slot, &zero, 1, true);
	} else {
		uint8_t descriptor_index = buf[0] - 1;
		if (descriptor_index >= UXB_DESCRIPTOR_SIZE) {
			uxb_slot_send_data(&descriptor_slot, &zero, 1, true);
		} else {
			uxb_slot_send_data(&descriptor_slot, (uint8_t *)uxb_descriptor[descriptor_index], strlen(uxb_descriptor[descriptor_index]) + 1, true);
		}
	}

	return UXB_MASTER_LOCM3_RET_OK;
}


static uxb_master_locm3_ret_t uxb_data_received(void *context, uint8_t *buf, size_t len) {
	(void)context;
	(void)buf;
	(void)len;

	uint8_t tx[16] = {
		(bat_voltage >> 24) & 0xff,
		(bat_voltage >> 16) & 0xff,
		(bat_voltage >> 8) & 0xff,
		(bat_voltage >> 0) & 0xff,

		(bat_current >> 24) & 0xff,
		(bat_current >> 16) & 0xff,
		(bat_current >> 8) & 0xff,
		(bat_current >> 0) & 0xff,

		(bat_charge >> 24) & 0xff,
		(bat_charge >> 16) & 0xff,
		(bat_charge >> 8) & 0xff,
		(bat_charge >> 0) & 0xff,

		(bat_temp >> 24) & 0xff,
		(bat_temp >> 16) & 0xff,
		(bat_temp >> 8) & 0xff,
		(bat_temp >> 0) & 0xff,
	};

	uxb_slot_send_data(&slot1, tx, 16, true);
	return UXB_MASTER_LOCM3_RET_OK;
}


static void clock_setup(void) {
	// rcc_set_hpre(RCC_CFGR_HPRE_DIV2);
}


static void gpio_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Initialize the STAT LED. */
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);

	/* Initialize STC3100 I2C. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9 | GPIO10);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO9 | GPIO10);
}


static void i2c_setup(void) {
	rcc_periph_clock_enable(RCC_I2C1);
	i2c_reset(I2C1);
	i2c_peripheral_disable(I2C1);
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, I2C_CR1_DNF_DISABLED);
	i2c_set_speed(I2C1, i2c_speed_sm_100k, 8);
	i2c_enable_stretching(I2C1);
	i2c_set_7bit_addr_mode(I2C1);
	i2c_peripheral_enable(I2C1);
}


static void uxb_setup(void) {
	/* Initialize the UXB bus. */
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_TIM14);

	/* Setup a timer for precise UXB protocol delays. */
	timer_reset(TIM14);
	timer_set_mode(TIM14, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM14);
	timer_direction_up(TIM14);
	timer_disable_preload(TIM14);
	timer_enable_update_event(TIM14);
	timer_set_prescaler(TIM14, (rcc_ahb_frequency / 1000000) - 1);
	timer_set_period(TIM14, 65535);
	timer_enable_counter(TIM14);

	uxb_master_locm3_init(&uxb, &(struct uxb_master_locm3_config) {
		.spi_port = SPI1,
		.spi_af = GPIO_AF0,
		.sck_port = GPIOA, .sck_pin = GPIO5,
		.miso_port = GPIOA, .miso_pin = GPIO6,
		.mosi_port = GPIOA, .mosi_pin = GPIO7,
		.frame_port = GPIOA, .frame_pin = GPIO4,
		.delay_timer = TIM14,
		.delay_timer_freq_mhz = 1,
	});

	uxb_interface_init(&iface1);
	uxb_interface_set_address(&iface1, uxb_iface1_address, uxb_remote_address);
	uxb_master_locm3_add_interface(&uxb, &iface1);

	uxb_slot_init(&slot1);
	uxb_slot_set_slot_number(&slot1, 5);
	uxb_slot_set_slot_buffer(&slot1, slot1_buffer, sizeof(slot1_buffer));
	uxb_slot_set_data_received(&slot1, uxb_data_received, NULL);
	uxb_interface_add_slot(&iface1, &slot1);

	uxb_slot_init(&descriptor_slot);
	uxb_slot_set_slot_number(&descriptor_slot, 0);
	uxb_slot_set_slot_buffer(&descriptor_slot, descriptor_slot_buffer, sizeof(descriptor_slot_buffer));
	uxb_slot_set_data_received(&descriptor_slot, uxb_read_descriptor, NULL);
	uxb_interface_add_slot(&iface1, &descriptor_slot);

	/* Setup uxb exti interrupts. */
	nvic_enable_irq(NVIC_EXTI4_15_IRQ);
	rcc_periph_clock_enable(RCC_SYSCFG_COMP);
	exti_select_source(EXTI4, GPIOA);
	exti_set_trigger(EXTI4, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI4);

}


void exti4_15_isr(void) {
	exti_reset_request(EXTI4);
	exti_disable_request(EXTI4);
	uxb_master_locm3_frame_irq(&uxb);
	exti_enable_request(EXTI4);
}


static void stc3100_init(void) {
	// i2c_transfer7(I2C1, 0x70, (uint8_t[]){0x01, 0x02}, 2, NULL, 0);
	i2c_transfer7(I2C1, 0x70, (uint8_t[]){0x00, 0x10}, 2, NULL, 0);
}


static uint8_t stc3100_status(void) {
	uint8_t status;
	i2c_transfer7(I2C1, 0x70, (uint8_t[]){0x01}, 1, &status, 1);
	return status;
}


static int32_t stc3100_voltage(void) {
	uint8_t voltage[2];
	i2c_transfer7(I2C1, 0x70, (uint8_t[]){0x08}, 1, voltage, 2);
	return ((voltage[1] << 8 | voltage[0]) * 2440) / 1000;
}


static int32_t stc3100_current(void) {
	uint8_t current_code[2];
	i2c_transfer7(I2C1, 0x70, (uint8_t[]){0x06}, 1, current_code, 2);
	int16_t current = (current_code[1] << 10) | (current_code[0] << 2);
	return current / 4 * 1177 / 5;
}


static int32_t stc3100_temp(void) {
	uint8_t temp_code[2];
	i2c_transfer7(I2C1, 0x70, (uint8_t[]){0x0a}, 1, temp_code, 2);
	int16_t temperature = (temp_code[1] << 12) | (temp_code[0] << 4);
	return temperature * 125 / 16;
}


static int32_t stc3100_charge(void) {
	uint8_t charge_code[2];
	i2c_transfer7(I2C1, 0x70, (uint8_t[]){0x02}, 1, charge_code, 2);
	int16_t charge = (charge_code[1] << 8) | charge_code[0];
	return charge * 67 / 50 / 10;
}


static void pwm_setup(void) {


}


int main(void) {

	clock_setup();
	// initialise_monitor_handles();
	gpio_setup();
	pwm_setup();
	i2c_setup();
	uxb_setup();
	stc3100_init();

	while (1) {
		for (uint32_t i = 0; i < 100000; i++) {
			__asm__("nop");
		}

		bat_voltage = stc3100_voltage();
		bat_current = stc3100_current();
		bat_temp = stc3100_temp();
		bat_charge = stc3100_charge();
	}

	return 0;
}



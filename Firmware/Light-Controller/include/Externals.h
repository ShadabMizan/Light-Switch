#ifndef EXTERNALS_H
#define EXTERNALS_H

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#define LED_RED_GPIO        GPIO_NUM_6
#define LED_GREEN_GPIO      GPIO_NUM_13
#define RELAY_GPIO          GPIO_NUM_21
#define LINE_DETECT_GPIO    GPIO_NUM_20
#define TRIAC_DRIVE_GPIO    GPIO_NUM_16

#define DIAL_ADC_CHANNEL    ADC_CHANNEL_4

#define LED_BLINK_MS        75

#define ZERO_CROSS_DETECTED_BIT     (1 << 0)
#define DIAL_MOVEMENT_DETECTED_BIT  (2 << 0)

#define DIAL_READER_STACK_DEPTH         256

#define DIAL_READER_NUM_ADC_SAMPLES     32
#define DIAL_READER_SLEEP_MS            50
#define DIAL_MOVEMENT_THRESHOLD         30

// 1 for printing the ADC average values, 0 for not
#define DIAL_READER_PRINT_ADC_AVGS      1

#define TRIAC_DRIVER_STACK_DEPTH        256

typedef enum {
    GREEN_LED,
    RED_LED
} LED_t;

void Externals_Init(void);
void LED_Blink(LED_t led, uint8_t num_blinks);
void Relay_Toggle(void);

extern float dial_volume;

#endif

#ifndef EXTERNALS_H
#define EXTERNALS_H

#include "driver/gpio.h"
#include "driver/adc.h"

#define LED_RED_GPIO        GPIO_NUM_6
#define LED_GREEN_GPIO      GPIO_NUM_13
#define RELAY_GPIO          GPIO_NUM_21
#define LINE_DETECT_GPIO    GPIO_NUM_20
#define TRIAC_DRIVE_GPIO    GPIO_NUM_16

#define DIAL_ADC_CHANNEL    ADC1_CHANNEL_4

#define LED_BLINK_MS        100

typedef enum {
    GREEN_LED,
    RED_LED
} Ext_LED_t;

#endif

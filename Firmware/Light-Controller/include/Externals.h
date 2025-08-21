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

#define LED_BLINK_MS        100

#define DIAL_READER_STACK_DEPTH         2048
#define DIAL_READER_NUM_ADC_SAMPLES     32
#define DIAL_READER_SLEEP_MS            50
#define DIAL_MOVEMENT_THRESHOLD         30

// 1 for printing, 0 for not
#define DIAL_READER_PRINT_ADC_AVGS      0
#define LINE_MONITOR_PRINT_LINE_STATUS  0

#define AC_FREQ_HZ         60.0f
#define HALF_PERIOD_US     (1000000.0f / (2.0f * AC_FREQ_HZ))  // 8333.33 us for 60 Hz     

#define TRIAC_GATE_PULSE_US             100

#define LINE_MONITOR_STACK_DEPTH        2048
#define LINE_MONITOR_NO_ZC_DETECTED_THRESHOLD_MS    4 * HALF_PERIOD_US  // A ZC hasn't been detected for two periods
#define LINE_MONITOR_SLEEP_MS           50

#define LED_BLINKER_STACK_DEPTH         2048

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

typedef enum {
    GREEN_LED,
    RED_LED
} LED_t;

typedef enum {
    IDLE = 0,
    ZC_DELAY_EXPIRED,
    PULSE_ON_EXPIRED
} Triac_Timer_State_t;

typedef struct {
    LED_t led;
    uint8_t num_blinks;
} Blink_LED_Args_t;

void Externals_Init(void);

void Blink_LED(LED_t led, uint8_t num_blinks);
void Toggle_Relay(void);

void Set_PowerDelivered(float percentage);
float Get_PowerDelivered(void);

int Get_LineStatus(void);

#endif

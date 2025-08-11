#include "Externals.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_adc_cal_characteristics_t adc1_cal_chars;

void Externals_Init(void) {
    // Set up LEDs
    esp_rom_gpio_pad_select_gpio(LED_GREEN_GPIO);
    gpio_set_direction(LED_GREEN_GPIO, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(LED_RED_GPIO);
    gpio_set_direction(LED_RED_GPIO, GPIO_MODE_OUTPUT);

    // Set up Potentiometer Dial
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_BITWIDTH_12, 0, &adc1_cal_chars);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(DIAL_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // Set up the Relay GPIO as an output
    esp_rom_gpio_pad_select_gpio(RELAY_GPIO);
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);

    // Set up Line Detection as Rising & Falling EdgeInterrupt
    esp_rom_gpio_pad_select_gpio(LINE_DETECT_GPIO);
    gpio_set_direction(LINE_DETECT_GPIO, GPIO_MODE_INPUT);
    

    
    // Set up Triac gate drive

}

// Reads the mechanical dial's setting and maps it between 0-100
int Dial_GetValue(void) {
    int raw = adc1_get_raw(DIAL_ADC_CHANNEL);   // between 0-4096

    return raw * 100/4096;  // Scale to 0-100
}

void LED_Blink(Ext_LED_t led, uint8_t num_blinks) {
    gpio_num_t led_gpio;
    switch (led) {
        case GREEN_LED:
            led_gpio = LED_GREEN_GPIO;
            break;
        case RED_LED:
            led_gpio = LED_RED_GPIO;
            break;
    }

    for (int i = 0; i < num_blinks; i++) {
        gpio_set_level(led_gpio, 1);
        vTaskDelay(LED_BLINK_MS/portTICK_PERIOD_MS);
        gpio_set_level(led_gpio, 0);
        vTaskDelay(LED_BLINK_MS/portTICK_PERIOD_MS);
    }
}

void Relay_Toggle(void) {
    static int level = 0;
    level = !level;
    gpio_set_level(RELAY_GPIO, level);
}

#include "Externals.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_adc_cal_characteristics_t adc1_cal_chars;

static void IRAM_ATTR LineDetect_ISR(void *arg);

void Externals_Init(void) {
    // Set up LED GPIOs
    esp_rom_gpio_pad_select_gpio(LED_GREEN_GPIO);
    gpio_set_direction(LED_GREEN_GPIO, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(LED_RED_GPIO);
    gpio_set_direction(LED_RED_GPIO, GPIO_MODE_OUTPUT);

    // Set up the Relay GPIO as an output
    esp_rom_gpio_pad_select_gpio(RELAY_GPIO);
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);

    // Set up Line Detection as Rising & Falling Edge Interrupt
    esp_rom_gpio_pad_select_gpio(LINE_DETECT_GPIO);
    gpio_set_direction(LINE_DETECT_GPIO, GPIO_MODE_INPUT);
    
    gpio_set_intr_type(LINE_DETECT_GPIO, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(LINE_DETECT_GPIO, LineDetect_ISR, NULL);
    gpio_intr_enable(LINE_DETECT_GPIO);

    // Set up Triac gate drive
    esp_rom_gpio_pad_select_gpio(TRIAC_DRIVE_GPIO);
    gpio_set_direction(TRIAC_DRIVE_GPIO, GPIO_MODE_OUTPUT);

    // Set up Potentiometer Dial
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_BITWIDTH_12, 0, &adc1_cal_chars);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(DIAL_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

// Reads the mechanical dial's setting and normalizes it
double Dial_GetValueNormalized(void) {
    int raw = adc1_get_raw(DIAL_ADC_CHANNEL);   // between 0-4095
    return raw/4095.0;
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

    for (uint8_t i = 0; i < num_blinks; i++) {
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

void IRAM_ATTR LineDetect_ISR(void *arg) {
    
}

void Triac_PulseGate(void) {

}
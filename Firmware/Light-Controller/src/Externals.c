#include "Externals.h"
#include "Priorities.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/* Public Resources */
float dial_volume;      // TODO: Make this into a Queue

/* Private Resources */
static adc_oneshot_unit_handle_t adc1_handle;

static EventGroupHandle_t externals_eventgroup;

static TaskHandle_t dial_reader_task;

static TaskHandle_t triac_driver_task;
static QueueHandle_t triac_driver_queue;

static inline void led_init(void);
static inline void relay_init(void);
static inline void line_detect_init(void);
static inline void triac_drive_init(void);
static inline void dial_init(void);

static void line_detect_isr_handler(void *arg);
static void mains_timer_isr_handler(void *arg);

static void Dial_Reader(void *pvParameters);
static void Triac_Driver(void *pvParameters);

/* Public Functions */
void Externals_Init(void) {
    externals_eventgroup = xEventGroupCreate();
    if (externals_eventgroup == NULL) {
        printf("Exernals Event Group Create Error\r\n");
    }

    led_init();
    dial_init();

    relay_init();
    line_detect_init();
    triac_drive_init();
}

void LED_Blink(LED_t led, uint8_t num_blinks) {
    gpio_num_t led_gpio = RED_LED;
    switch (led) {
        case RED_LED:
            led_gpio = RED_LED;
            break;
        case GREEN_LED:
            led_gpio = GREEN_LED;
            break;
    }

    for (uint8_t i = 0; i < num_blinks; i++) {
        gpio_set_level(led_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_MS));
        gpio_set_level(led_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_MS));
    }
}

void Relay_Toggle(void) {
    static uint8_t relay_state = 0;

    relay_state = !relay_state;
    gpio_set_level(RELAY_GPIO, relay_state);

    LED_Blink(GREEN_LED, 1);
}

/* Tasks */
void Dial_Reader(void *pvParameters) {    
    int adc_raw;
    int adc_raw_avg = 0;
    int last_adc_reading = 0;
    esp_err_t status = ESP_OK;
    while (1) {
        for (int i = 0; i < DIAL_READER_NUM_ADC_SAMPLES; i++) {
            status = adc_oneshot_read(&adc1_handle, DIAL_ADC_CHANNEL, &adc_raw);
            if (status != ESP_OK) {
                printf("ADC Oneshot Read Error: %s\r\n", esp_err_to_name(status));
                break;
            }
            adc_raw_avg += adc_raw;
        }
        adc_raw_avg /= DIAL_READER_NUM_ADC_SAMPLES;
        
        #if (DIAL_READER_PRINT_ADC_AVGS == 1)
        printf("Raw ADC Average: %d\r\n", adc_raw_avg);
        #endif

        // Has the dial moved?
        if (adc_raw_avg + DIAL_MOVEMENT_THRESHOLD > last_adc_reading || adc_raw_avg - DIAL_MOVEMENT_THRESHOLD < last_adc_reading) {
            // Update the dial volume, which is normalized to between 0-1.
            dial_volume = adc_raw_avg/4095.0f;
        }
        last_adc_reading = adc_raw_avg;
        adc_raw_avg = 0;


        vTaskDelay(pdMS_TO_TICKS(DIAL_READER_SLEEP_MS));
    }

    (void)pvParameters;
}

void Triac_Driver(void *pvParameters) {
    float power_delivered = 1.0f;    
    // Get the starting half-period of the mains

    while (1) {
        xEventGroupWaitBits(externals_eventgroup, ZERO_CROSS_DETECTED_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
        xQueueReceive(triac_driver_queue, &power_delivered, 0);

        // Start hardware timer based off of power delivered
        
    }
    (void)pvParameters;
}

/* Interrupts */
void line_detect_isr_handler(void *arg) {
    BaseType_t task_woken = pdFALSE;
    xEventGroupSetBitsFromISR(externals_eventgroup, ZERO_CROSS_DETECTED_BIT, &task_woken);

    if (task_woken) {
        portYIELD_FROM_ISR();
    }
}

void mains_timer_isr_handler(void *arg) {
    
}

/* Private Function */
void led_init(void) {
    // Applies the same settings to the green and red LEDs
    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GREEN_GPIO) | (1ULL << LED_RED_GPIO),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));
}

void dial_init(void) {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, DIAL_ADC_CHANNEL, &config));

    BaseType_t status = xTaskCreate(
        Dial_Reader, "Dial Reader Task",
        DIAL_READER_STACK_DEPTH, 0,
        DIAL_READER_PRIORITY, &dial_reader_task
    );
    if (status != pdPASS) {
        printf("Dial Reader Task Create Error: 0x%X\r\n", status);
    }
}

void relay_init(void) {
    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << RELAY_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));
}

void line_detect_init(void) {
    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LINE_DETECT_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));

    // Calling GPIO install ISR here since I think this is the only ISR in the system
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LINE_DETECT_GPIO, line_detect_isr_handler, (void *)LINE_DETECT_GPIO));
}

void triac_drive_init(void) {
    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TRIAC_DRIVE_GPIO),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));
    
    timer_config_t tim_config = {
        .divider = 80,  // 1us per hardware timer tick
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = pdFALSE
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &tim_config));
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, mains_timer_isr_handler, 0, 0));

    triac_driver_queue = xQueueCreate(10, sizeof(float));
    if (triac_driver_queue == NULL) {
        printf("Triac Driver Queue Create Error\r\n");
    }

    BaseType_t status = pdPASS;
    status = xTaskCreate(
        Triac_Driver, "Triac Driver Task",
        TRIAC_DRIVER_STACK_DEPTH, 0,
        TRIAC_DRIVER_PRIORTIY, &triac_driver_task
    );
    if (status != pdPASS) {
        printf ("Triac Driver Task Create Error: 0x%X\r\n", status);
    }
}


#include "Externals.h"
#include "Priorities.h"
#include "WebServer.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gptimer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* Private Resources */
static const char *TAG = "EXTERNALS";

static float power_delivered = 1.0f;
static volatile TickType_t last_zc_tick = 0;
static int line_status = 0;

static adc_oneshot_unit_handle_t adc1_handle;

static TaskHandle_t dial_reader_task;
static TaskHandle_t line_monitor_task;
static TaskHandle_t led_blinker_task;

static QueueHandle_t led_blinker_queue;

static volatile Triac_Timer_State_t next_triac_timer_state = IDLE;
static gptimer_handle_t triac_timer = NULL;

static void led_init(void);
static void relay_init(void);
static void line_detect_init(void);
static void triac_drive_init(void);
static void dial_init(void);

static inline void triac_pulse_gate(void);
static inline void start_zc_delay_timer(uint32_t delay_us);

static void gpio_isr_handler(void *arg);
static bool triac_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);

static void Dial_Reader(void *pvParameters);
static void Line_Monitor(void *pvParameters);
static void LED_Blinker(void *pvParamters);

/* Public Functions */
void Externals_Init(void) {
    led_init();
    dial_init();

    relay_init();
    line_detect_init();
    triac_drive_init();
}

void Blink_LED(LED_t led, uint8_t num_blinks) {
    Blink_LED_Args_t args = {
        .led = led,
        .num_blinks = num_blinks
    };

    xQueueSend(led_blinker_queue, &args, 0);
}

void Toggle_Relay(void) {
    static uint8_t relay_state = 0;

    relay_state = !relay_state;
    gpio_set_level(RELAY_GPIO, relay_state);
}

void Set_PowerDelivered(float percentage) {
    percentage = MIN(percentage, 1);
    percentage = MAX(percentage, 0);

    power_delivered = percentage;
}

float Get_PowerDelivered(void) {
    return power_delivered;
}

int Get_LineStatus(void) {
    return line_status;
}

/* Tasks */
void Dial_Reader(void *pvParameters) {
    int adc_raw;
    int adc_raw_avg = 0;
    int last_adc_reading = 0;
    esp_err_t status = ESP_OK;
    while (1) {
        for (int i = 0; i < DIAL_READER_NUM_ADC_SAMPLES; i++) {
            status = adc_oneshot_read(adc1_handle, DIAL_ADC_CHANNEL, &adc_raw);
            if (status != ESP_OK) {
                ESP_LOGD(TAG, "ADC Oneshot Read Error: %s", esp_err_to_name(status));
                break;
            }
            adc_raw_avg += adc_raw;
        }
        adc_raw_avg /= DIAL_READER_NUM_ADC_SAMPLES;
        
        #if (DIAL_READER_PRINT_ADC_AVGS == 1)
        ESP_LOGI(TAG, "Raw ADC Average: %d", adc_raw_avg);
        #endif

        // Has the dial moved?
        if (adc_raw_avg + DIAL_MOVEMENT_THRESHOLD > last_adc_reading || adc_raw_avg - DIAL_MOVEMENT_THRESHOLD < last_adc_reading) {
            // Update the power delivered, which is normalized to between 0-1.
            Set_PowerDelivered(adc_raw_avg/4095.0f);
            if (IsClientConnected()) {
                Update_PowerDelivered_OnWeb(Get_PowerDelivered());  // Update on Web
            }
        }
        last_adc_reading = adc_raw_avg;
        adc_raw_avg = 0;

        vTaskDelay(pdMS_TO_TICKS(DIAL_READER_SLEEP_MS));
    }

    (void)pvParameters;
}

void Line_Monitor(void *pvParamters) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (1) {
        if (pdTICKS_TO_MS(xTaskGetTickCount() - last_zc_tick) > LINE_MONITOR_NO_ZC_DETECTED_THRESHOLD_MS) {
            if (line_status != 0) {
                // Change the line status to 0, or inactive
                #if (LINE_MONITOR_PRINT_LINE_STATUS == 1)
                ESP_LOGI(TAG, "Line is Inactive");
                #endif
                line_status = 0;
                Blink_LED(GREEN_LED, 1);

                if (IsClientConnected()) {
                    Update_LineStatus_OnWeb(Get_LineStatus());
                }
            }
        } else {
            if (line_status != 1) {
                // Change line status to 1, or active
                #if (LINE_MONITOR_PRINT_LINE_STATUS == 1)
                ESP_LOGI(TAG, "Line is Active");
                #endif
                line_status = 1;
                Blink_LED(RED_LED, 1);

                if (IsClientConnected()) {
                    Update_LineStatus_OnWeb(Get_LineStatus());
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LINE_MONITOR_SLEEP_MS));
    }
    (void)pvParamters;
}

void LED_Blinker(void *pvParameters) {
    Blink_LED_Args_t args;
    while (1) {
        xQueueReceive(led_blinker_queue, &args, portMAX_DELAY);

        gpio_num_t led_gpio = LED_RED_GPIO;
        switch (args.led) {
            case RED_LED:
                led_gpio = LED_RED_GPIO;
                break;
            case GREEN_LED:
                led_gpio = LED_GREEN_GPIO;
                break;
        }

        for (uint8_t i = 0; i < args.num_blinks; i++) {
            gpio_set_level(led_gpio, 1);
            vTaskDelay(pdMS_TO_TICKS(LED_BLINK_MS));
            gpio_set_level(led_gpio, 0);
            vTaskDelay(pdMS_TO_TICKS(LED_BLINK_MS));
        }
    }
    (void)pvParameters;
}

/* Interrupts and Callbacks */
void gpio_isr_handler(void *arg) {
    last_zc_tick = xTaskGetTickCountFromISR();

    if (power_delivered >= 0.99f) {
        // Drive the Triac instantly
        triac_pulse_gate();
    } else if (power_delivered <= 0.01f) {
        // Don't drive the triac
    } else {
        // Ex.To deliver 60% power, we need to delay for 40% of the half ac period.
        uint32_t delay_us = (uint32_t)((1.0f - power_delivered) * HALF_PERIOD_US);
        start_zc_delay_timer(delay_us);
    }

    (void)arg;
}

bool triac_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    switch (next_triac_timer_state) {
        case ZC_DELAY_EXPIRED:
            // Zero Cross delay is over, so pulse the triac gate
            triac_pulse_gate();
            break;
        case PULSE_ON_EXPIRED:
            // Pulse ON time is over, so turn off. This completes the triac pulse gate functionality
            gpio_set_level(TRIAC_DRIVE_GPIO, 0);
            next_triac_timer_state = IDLE;
            gptimer_stop(triac_timer);
            break;
        default:
            break;
    }

    (void)timer;
    (void)edata;
    (void)user_ctx;
    return false;
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
    gpio_config(&io_config);

    led_blinker_queue = xQueueCreate(10, sizeof(Blink_LED_Args_t));
    if (led_blinker_queue == NULL) {
        ESP_LOGE(TAG, "LED Blinker queue create could not allocate required memory");
    }

    BaseType_t status = xTaskCreate(
        LED_Blinker, "LED Blinker Task",
        LED_BLINKER_STACK_DEPTH, 0,
        LED_BLINKER_PRIORITY, &led_blinker_task
    );
    if (status != pdPASS) {
        ESP_LOGE(TAG, "LED Blinker task create could not allocate required memory");
    }
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
        ESP_LOGE(TAG, "Dial Reader task create could not allocated required memory");
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
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LINE_DETECT_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));
    
    // Calling GPIO install ISR here since I think this is the only ISR in the system
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LINE_DETECT_GPIO, gpio_isr_handler, 0));

    BaseType_t status = xTaskCreate(
        Line_Monitor, "Line Monitor Task",
        LINE_MONITOR_STACK_DEPTH, 0,
        LINE_MONITOR_PRIORITY, &line_monitor_task
    );
    if (status != pdPASS) {
        ESP_LOGE(TAG, "Line Monitor task create could not allocated required memory");
    }
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

    // GPTimer configuration
    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000000, // 1 MHz → 1 tick = 1 us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &triac_timer));

    // Alarm configuration
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 0,        // will set dynamically
        .flags.auto_reload_on_alarm = false,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(triac_timer, &alarm_cfg));

    // Register callback function
    gptimer_event_callbacks_t callbacks = {
        .on_alarm = triac_timer_callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(triac_timer, &callbacks, 0));
    ESP_ERROR_CHECK(gptimer_enable(triac_timer));
}

void triac_pulse_gate(void) {
    gpio_set_level(TRIAC_DRIVE_GPIO, 1);

    gptimer_stop(triac_timer);
    gptimer_set_raw_count(triac_timer, 0);
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TRIAC_GATE_PULSE_US,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(triac_timer, &alarm_config);

    next_triac_timer_state = PULSE_ON_EXPIRED;
    gptimer_start(triac_timer);
}

void start_zc_delay_timer(uint32_t delay_us) {
    gptimer_stop(triac_timer);
    gptimer_set_raw_count(triac_timer, 0);
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = delay_us,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(triac_timer, &alarm_config);

    next_triac_timer_state = ZC_DELAY_EXPIRED;
    gptimer_start(triac_timer);
}

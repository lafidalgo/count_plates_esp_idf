#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_sleep.h"
#include "esp_log.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "soc/soc_caps.h"
#include "soc/rtc.h"

#include <time.h>
#include <sys/time.h>

#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "esp32/ulp.h"
#include "ulp_main.h"

#include "sdkconfig.h"

#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)

#define weightReference 2000 // 2 kg
#define maxUnitDifference 2
#define measureSignalReference -1 // O valor diminuir quando aumenta o peso
#define ulpWakeUpPeriod 1000000   // Em us (1 s)
#define ulpWakeUpPeriodFast 1000  // Em us (1 ms)

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

static RTC_DATA_ATTR uint32_t tare = 1;
static RTC_DATA_ATTR float calibration = 1;
static RTC_DATA_ATTR int32_t unitWeight = 1;

const int ext_wakeup_pin_1 = 2;
const int ext_wakeup_pin_2 = 4;
const int ext_wakeup_pin_3 = 13;

gpio_num_t led_pin = GPIO_NUM_14;

/*Variáveis para armazenamento do handle das tasks, queues, semaphores, timers e event groups*/
TaskHandle_t taskEnterDeepSleepHandle = NULL;
TaskHandle_t taskTareHandle = NULL;
TaskHandle_t taskCalibrateHandle = NULL;
TaskHandle_t taskSetUnitHandle = NULL;

EventGroupHandle_t xEventGroupDeepSleep;

//******************** FUNCTIONS ********************
static void init_ulp_program(void);

static uint32_t readWeight(int repeatRead);

static void blinkLED(void);

//******************** TASKS ********************
void enterDeepSleepTask(void *pvParameters)
{
    while (1)
    {
        xEventGroupWaitBits(
            xEventGroupDeepSleep,  /* The event group being tested. */
            BIT_0 | BIT_1 | BIT_2, /* The bits within the event group to wait for. */
            pdTRUE,                /* BIT_0 & BIT_1 & BIT_2 should be cleared before returning. */
            pdTRUE,                /* Wait for both bits. */
            portMAX_DELAY);        /* Wait a maximum of 100ms for either bit to be set. */

        const int wakeup_time_sec = 60;
        // printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
        esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

        const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
        const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;
        const uint64_t ext_wakeup_pin_3_mask = 1ULL << ext_wakeup_pin_3;

        // printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2, ext_wakeup_pin_3);
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask | ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

        // printf("Enabling ULP wakeup\n");
        ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

        /* Disconnect GPIO12 and GPIO15 to remove current drain through
         * pullup/pulldown resistors.
         * GPIO12 may be pulled high to select flash voltage.
         */
        rtc_gpio_isolate(GPIO_NUM_12);
        rtc_gpio_isolate(GPIO_NUM_15);
        esp_deep_sleep_disable_rom_logging(); // suppress boot messages

        printf("Entering deep sleep\n");
        esp_deep_sleep_start();
    }
}

void tareTask(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("Tare Task.\n");
        uint32_t HX711Total = readWeight(5);
        if (HX711Total)
        {
            printf("Valor Total: %d\n", HX711Total);
            tare = HX711Total;
            printf("Valor Tara: %d\n", tare);
            blinkLED();
        }
        xEventGroupSetBits(xEventGroupDeepSleep, BIT_0);
    }
}

void calibrateTask(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("Calibrate Task.\n");
        uint32_t HX711Total = readWeight(5);
        if (HX711Total)
        {
            printf("Valor Total: %d\n", HX711Total);
            printf("Valor Tara: %d\n", tare);
            calibration = ((float)HX711Total - (float)tare) / (float)weightReference;
            printf("Valor Calibração: %.2f\n", calibration);
            blinkLED();
        }
        xEventGroupSetBits(xEventGroupDeepSleep, BIT_1);
    }
}

void setUnitTask(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("Set Unit Task.\n");
        uint32_t HX711Total = readWeight(5);
        if (HX711Total)
        {
            printf("Valor Total: %d\n", HX711Total);
            unitWeight = (HX711Total - tare);
            printf("Valor Unitário: %d\n", unitWeight);
            float weightGrams = (float)unitWeight / calibration;
            printf("Peso Unitário: %.2f g\n", weightGrams);

            uint32_t weightDifference = unitWeight * (maxUnitDifference + 0.5) * measureSignalReference;
            // Acorda quando o valor medido é maior que o definido por Over
            ulp_trshHoldOverADMSB = (tare + weightDifference) >> 16;
            ulp_trshHoldOverADLSB = (tare + weightDifference) & 0xFFFF;
            // Acorda quando o valor medido é menor que o definido por Under
            ulp_trshHoldUnderADMSB = (tare - weightDifference) >> 16;
            ulp_trshHoldUnderADLSB = (tare - weightDifference) & 0xFFFF;
            blinkLED();
        }
        xEventGroupSetBits(xEventGroupDeepSleep, BIT_2);
    }
}

//******************** App Main ********************
void app_main(void)
{
    /*Criação Event Groups*/
    xEventGroupDeepSleep = xEventGroupCreate();
    if (xEventGroupDeepSleep == NULL)
    {
        printf("The event group was not created.");
    }
    xEventGroupSetBits(xEventGroupDeepSleep, BIT_0 | BIT_1 | BIT_2);

    /*Criação Tasks*/
    xTaskCreate(tareTask, "tareTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, &taskTareHandle);
    xTaskCreate(calibrateTask, "calibrateTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, &taskCalibrateHandle);
    xTaskCreate(setUnitTask, "setUnitTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, &taskSetUnitHandle);

    /* Initialize selected GPIO as RTC IO, enable output, disable pullup and pulldown */
    rtc_gpio_init(led_pin);
    rtc_gpio_set_direction(led_pin, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(led_pin);
    rtc_gpio_pullup_dis(led_pin);
    rtc_gpio_set_level(led_pin, 0);

    switch (esp_sleep_get_wakeup_cause())
    {
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from GPIO %d\n", pin);
            switch (pin)
            {
            case ext_wakeup_pin_1:
                xEventGroupClearBits(xEventGroupDeepSleep, BIT_0);
                xTaskNotifyGive(taskTareHandle);
                break;
            case ext_wakeup_pin_2:
                xEventGroupClearBits(xEventGroupDeepSleep, BIT_1);
                xTaskNotifyGive(taskCalibrateHandle);
                break;
            case ext_wakeup_pin_3:
                xEventGroupClearBits(xEventGroupDeepSleep, BIT_2);
                xTaskNotifyGive(taskSetUnitHandle);
                break;
            }
        }
        else
        {
            printf("Wake up from GPIO\n");
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_TIMER:
    {
        printf("Wake up from timer.\n");
        break;
    }
    case ESP_SLEEP_WAKEUP_ULP:
    {
        printf("ULP wakeup\n");
        uint32_t HX711Total = readWeight(5);
        if (HX711Total)
        {
            printf("Valor Total: %d\n", HX711Total);
            uint32_t thresholdType_ulp = (ulp_thresholdType & UINT16_MAX);
            printf("Tipo threshold: %d\n", thresholdType_ulp);
            float weightGrams = ((float)HX711Total - (float)tare) / calibration;
            printf("Peso: %.2f g\n", weightGrams);
            float quantityUnits = ((float)HX711Total - (float)tare) / (float)unitWeight;
            printf("Quantidade: %.2f\n", quantityUnits);

            uint32_t weightDifference = unitWeight * (maxUnitDifference + 0.5) * measureSignalReference;
            // Acorda quando o valor medido é maior que o definido por Over
            ulp_trshHoldOverADMSB = (HX711Total + weightDifference) >> 16;
            ulp_trshHoldOverADLSB = (HX711Total + weightDifference) & 0xFFFF;
            // Acorda quando o valor medido é menor que o definido por Under
            ulp_trshHoldUnderADMSB = (HX711Total - weightDifference) >> 16;
            ulp_trshHoldUnderADLSB = (HX711Total - weightDifference) & 0xFFFF;
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        printf("Not a deep sleep reset, initializing ULP\n");
        init_ulp_program();
    }

    /*Criação Task Deep Sleep*/
    xTaskCreate(enterDeepSleepTask, "enterDeepSleepTask", configMINIMAL_STACK_SIZE * 5, NULL, 5, &taskEnterDeepSleepHandle);
}

static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* GPIO used for do input. */
    gpio_num_t gpio_num_addo = GPIO_NUM_25;
    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(gpio_num_addo);
    rtc_gpio_set_direction(gpio_num_addo, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num_addo);
    rtc_gpio_pullup_dis(gpio_num_addo);
    rtc_gpio_hold_en(gpio_num_addo);

    /* GPIO used for sk output. */
    gpio_num_t gpio_num_adsk = GPIO_NUM_26;
    /* Initialize selected GPIO as RTC IO, enable output, disable pullup and pulldown */
    rtc_gpio_init(gpio_num_adsk);
    rtc_gpio_set_direction(gpio_num_adsk, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num_adsk);
    rtc_gpio_pullup_dis(gpio_num_adsk);
    rtc_gpio_hold_en(gpio_num_adsk);

    // Acorda quando o valor medido é maior que o definido por Over
    ulp_trshHoldOverADMSB = 211;
    ulp_trshHoldOverADLSB = 30196;
    // Acorda quando o valor medido é menor que o definido por Under
    ulp_trshHoldUnderADMSB = 208;
    ulp_trshHoldUnderADLSB = 60000;
    // Seta se quiser só ler peso, limpa se quiser fazer a comparação
    ulp_onlyReadWeight = 0;
    ulp_enterSleepHX711 = 1;

    ulp_set_wakeup_period(0, ulpWakeUpPeriod); // Set ULP wake up period T = 1s

    /* Start the program */
    err = ulp_run(&ulp_main - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

static uint32_t readWeight(int repeatRead)
{
    uint32_t accumulatedTotal = 0;

    ulp_onlyReadWeight = 1;
    ulp_enterSleepHX711 = 0;
    ulp_set_wakeup_period(0, ulpWakeUpPeriodFast);

    for (int count = 0; count < repeatRead; count++)
    {
        while (!(ulp_thresholdType & UINT16_MAX))
            ;
        uint32_t HX711HiWord_ulp = (ulp_HX711HiWord & UINT16_MAX);
        uint32_t HX711LoWord_ulp = (ulp_HX711LoWord & UINT16_MAX);
        uint32_t HX711Total = (HX711HiWord_ulp << 16) + HX711LoWord_ulp;
        ulp_thresholdType = 0;
        if (!HX711Total)
        {
            ulp_set_wakeup_period(0, ulpWakeUpPeriod);
            ulp_onlyReadWeight = 0;
            ulp_enterSleepHX711 = 1;
            return 0;
        }
        accumulatedTotal += HX711Total;
    }
    ulp_enterSleepHX711 = 1;
    accumulatedTotal = accumulatedTotal / repeatRead;

    ulp_set_wakeup_period(0, ulpWakeUpPeriod);
    ulp_onlyReadWeight = 0;

    return accumulatedTotal;
}

void blinkLED(void)
{
    rtc_gpio_set_level(led_pin, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    rtc_gpio_set_level(led_pin, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    rtc_gpio_set_level(led_pin, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    rtc_gpio_set_level(led_pin, 0);
}
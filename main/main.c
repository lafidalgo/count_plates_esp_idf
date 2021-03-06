#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_adc_cal.h"

#include "nvs.h"
#include "nvs_flash.h"

#include <time.h>
#include <sys/time.h>

#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "soc/soc_caps.h"
#include "soc/rtc.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"

#include "esp32/ulp.h"
#include "ulp_main.h"

#include "sdkconfig.h"

#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)

const int TARE_BIT = BIT_0;
const int CALIBRATE_BIT = BIT_1;
const int SET_UNIT_BIT = BIT_2;

#define weightReference 2000 // 2 kg
#define minUnitDifference 0
#define UnitDifferenceLowPriority 1
#define UnitDifferenceMediumPriority 2
#define UnitDifferenceHighPriority 5
#define measureSignalReference -1 // O valor diminuir quando aumenta o peso
#define ulpWakeUpPeriod 1000000   // Em us (1 s)
#define ulpWakeUpPeriodFast 1000  // Em us (1 ms)

#define repeatMeasureQuantity 5

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

static RTC_DATA_ATTR uint32_t tare = 1;
static RTC_DATA_ATTR float calibration = 1;
static RTC_DATA_ATTR int32_t unitWeight = 1;
static RTC_DATA_ATTR float lastQuantity = 0;
static RTC_DATA_ATTR uint32_t lastHX711Total = 0;
static RTC_DATA_ATTR float quantityDifferenceAccumulate = 0;
static RTC_DATA_ATTR int wakeup_time_sec = 0;
static RTC_DATA_ATTR struct timeval sleep_enter_time;

const int ext_wakeup_pin_1 = 2;
const int ext_wakeup_pin_2 = 4;
const int ext_wakeup_pin_3 = 13;

static const char *TAG = "App";

gpio_num_t led_pin = GPIO_NUM_14;

/*Variáveis para armazenamento do handle das tasks, queues, semaphores, timers e event groups*/
TaskHandle_t taskEnterDeepSleepHandle = NULL;
TaskHandle_t taskTareHandle = NULL;
TaskHandle_t taskCalibrateHandle = NULL;
TaskHandle_t taskSetUnitHandle = NULL;

EventGroupHandle_t xEventGroupDeepSleep;

//******************** FUNCTIONS ********************
static void init_ulp_program(void);

static uint32_t readWeight(void);

static void blinkLED(void);

static void nvsWriteUnsigned(const char *key, uint32_t value);

static uint32_t nvsReadUnsigned(const char *key);

static void nvsWriteSigned(const char *key, int32_t value);

static int32_t nvsReadSigned(const char *key);

static void initVariablesFromNVS(void);

static uint32_t measure_battery(int number_samples);

//******************** TASKS ********************
void enterDeepSleepTask(void *pvParameters)
{
    while (1)
    {
        xEventGroupWaitBits(
            xEventGroupDeepSleep,                    /* The event group being tested. */
            TARE_BIT | CALIBRATE_BIT | SET_UNIT_BIT, /* The bits within the event group to wait for. */
            pdTRUE,                                  /* BIT_0 & BIT_1 & BIT_2 should be cleared before returning. */
            pdTRUE,                                  /* Wait for both bits. */
            portMAX_DELAY);                          /* Wait a maximum of 100ms for either bit to be set. */

        if (wakeup_time_sec)
        {
            struct timeval now;
            gettimeofday(&now, NULL);
            int sleep_time_s = now.tv_sec - sleep_enter_time.tv_sec;
            int remaining_wake_up = wakeup_time_sec - sleep_time_s;
            if (remaining_wake_up < 0)
            {
                remaining_wake_up = 0;
            }
            ESP_LOGI(TAG, "Wake up remaining time: %ds", remaining_wake_up);
            esp_sleep_enable_timer_wakeup(remaining_wake_up * 1000000);
        }

        const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
        const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;
        const uint64_t ext_wakeup_pin_3_mask = 1ULL << ext_wakeup_pin_3;

        // ESP_LOGI(TAG, "Enabling EXT1 wakeup on pins GPIO%d, GPIO%d, GPIO%d", ext_wakeup_pin_1, ext_wakeup_pin_2, ext_wakeup_pin_3);
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask | ext_wakeup_pin_3_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

        // ESP_LOGI(TAG, "Enabling ULP wakeup");
        ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

        /* Disconnect GPIO12 and GPIO15 to remove current drain through
         * pullup/pulldown resistors.
         * GPIO12 may be pulled high to select flash voltage.
         */
        rtc_gpio_isolate(GPIO_NUM_12);
        rtc_gpio_isolate(GPIO_NUM_15);
        esp_deep_sleep_disable_rom_logging(); // suppress boot messages

        ESP_LOGI(TAG, "Entering deep sleep");
        esp_deep_sleep_start();
    }
}

void tareTask(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Tare Task.");
        ulp_setRepeatMeasure = 1;
        ulp_set_wakeup_period(0, ulpWakeUpPeriodFast);
        uint32_t HX711Total = readWeight();
        if (HX711Total)
        {
            ESP_LOGI(TAG, "Valor Total: %d", HX711Total);
            tare = HX711Total;
            ESP_LOGI(TAG, "Valor Tara: %d", tare);
            nvsWriteUnsigned("tareValue", tare);
            blinkLED();
        }
        xEventGroupSetBits(xEventGroupDeepSleep, TARE_BIT);
    }
}

void calibrateTask(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Calibrate Task.");
        ulp_setRepeatMeasure = 1;
        ulp_set_wakeup_period(0, ulpWakeUpPeriodFast);
        uint32_t HX711Total = readWeight();
        if (HX711Total)
        {
            ESP_LOGI(TAG, "Valor Total: %d", HX711Total);
            ESP_LOGI(TAG, "Valor Tara: %d", tare);
            calibration = ((float)HX711Total - (float)tare) / (float)weightReference;
            ESP_LOGI(TAG, "Valor Calibração: %.2f", calibration);
            nvsWriteSigned("calibrateValue", (calibration * 10000));
            blinkLED();
        }
        xEventGroupSetBits(xEventGroupDeepSleep, CALIBRATE_BIT);
    }
}

void setUnitTask(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Set Unit Task.");
        ulp_setRepeatMeasure = 1;
        ulp_set_wakeup_period(0, ulpWakeUpPeriodFast);
        uint32_t HX711Total = readWeight();
        if (HX711Total)
        {
            ESP_LOGI(TAG, "Valor Total: %d", HX711Total);
            unitWeight = (HX711Total - tare);
            ESP_LOGI(TAG, "Valor Unitário: %d", unitWeight);
            float weightGrams = (float)unitWeight / calibration;
            ESP_LOGI(TAG, "Peso Unitário: %.2f g", weightGrams);
            nvsWriteSigned("unitWeightValue", unitWeight);

            uint32_t weightDifference = unitWeight * (minUnitDifference + 0.5) * measureSignalReference;
            // Acorda quando o valor medido é maior que o definido por Over
            ulp_trshHoldOverADMSB = (tare + weightDifference) >> 16;
            ulp_trshHoldOverADLSB = (tare + weightDifference) & 0xFFFF;
            // Acorda quando o valor medido é menor que o definido por Under
            ulp_trshHoldUnderADMSB = (tare - weightDifference) >> 16;
            ulp_trshHoldUnderADLSB = (tare - weightDifference) & 0xFFFF;
            blinkLED();
        }
        xEventGroupSetBits(xEventGroupDeepSleep, SET_UNIT_BIT);
    }
}

//******************** App Main ********************
void app_main(void)
{
    /*Criação Event Groups*/
    xEventGroupDeepSleep = xEventGroupCreate();
    if (xEventGroupDeepSleep == NULL)
    {
        ESP_LOGE(TAG, "The event group was not created.");
    }
    xEventGroupSetBits(xEventGroupDeepSleep, TARE_BIT | CALIBRATE_BIT | SET_UNIT_BIT);

    /*Criação Tasks*/
    xTaskCreate(tareTask, "tareTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, &taskTareHandle);
    xTaskCreate(calibrateTask, "calibrateTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, &taskCalibrateHandle);
    xTaskCreate(setUnitTask, "setUnitTask", configMINIMAL_STACK_SIZE * 3, NULL, 5, &taskSetUnitHandle);

    /* Initialize LED GPIO */
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
            ESP_LOGI(TAG, "Wake up from GPIO %d", pin);

            switch (pin)
            {
            case ext_wakeup_pin_1:
                xEventGroupClearBits(xEventGroupDeepSleep, TARE_BIT);
                xTaskNotifyGive(taskTareHandle);
                break;
            case ext_wakeup_pin_2:
                xEventGroupClearBits(xEventGroupDeepSleep, CALIBRATE_BIT);
                xTaskNotifyGive(taskCalibrateHandle);
                break;
            case ext_wakeup_pin_3:
                xEventGroupClearBits(xEventGroupDeepSleep, SET_UNIT_BIT);
                xTaskNotifyGive(taskSetUnitHandle);
                break;
            }
        }
        else
        {
            ESP_LOGI(TAG, "Wake up from GPIO");
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_TIMER:
    {
        ESP_LOGI(TAG, "Wake up from timer");
        ESP_LOGI(TAG, "Diferença acumulada na quantidade: %.2f", quantityDifferenceAccumulate);
        uint32_t HX711Total = lastHX711Total;
        if (HX711Total)
        {
            ESP_LOGI(TAG, "Valor Total: %d", HX711Total);
            float weightGrams = ((float)HX711Total - (float)tare) / calibration;
            ESP_LOGI(TAG, "Peso: %.2f g", weightGrams);
            float quantityUnits = ((float)HX711Total - (float)tare) / (float)unitWeight;
            ESP_LOGI(TAG, "Quantidade: %.2f", quantityUnits);
        }
        uint32_t voltage_total = measure_battery(100);
        ESP_LOGI(TAG, "Battery Voltage: %dmV", voltage_total);
        quantityDifferenceAccumulate = 0;
        wakeup_time_sec = 0;
        break;
    }
    case ESP_SLEEP_WAKEUP_ULP:
    {
        ESP_LOGI(TAG, "ULP wakeup");
        uint32_t HX711Total = readWeight();
        if (HX711Total)
        {
            ESP_LOGI(TAG, "Valor Total: %d", HX711Total);
            float weightGrams = ((float)HX711Total - (float)tare) / calibration;
            ESP_LOGI(TAG, "Peso: %.2f g", weightGrams);
            float quantityUnits = ((float)HX711Total - (float)tare) / (float)unitWeight;
            ESP_LOGI(TAG, "Quantidade: %.2f", quantityUnits);

            uint32_t weightDifference = unitWeight * (minUnitDifference + 0.5) * measureSignalReference;
            // Acorda quando o valor medido é maior que o definido por Over
            ulp_trshHoldOverADMSB = (HX711Total + weightDifference) >> 16;
            ulp_trshHoldOverADLSB = (HX711Total + weightDifference) & 0xFFFF;
            // Acorda quando o valor medido é menor que o definido por Under
            ulp_trshHoldUnderADMSB = (HX711Total - weightDifference) >> 16;
            ulp_trshHoldUnderADLSB = (HX711Total - weightDifference) & 0xFFFF;

            if (quantityDifferenceAccumulate <= (minUnitDifference + 0.5))
            {
                gettimeofday(&sleep_enter_time, NULL);
            }

            float quantityDifference = (quantityUnits - lastQuantity);
            if (quantityDifference < 0)
            {
                quantityDifference *= -1;
            }
            quantityDifferenceAccumulate += quantityDifference;

            ESP_LOGI(TAG, "Diferença acumulada na quantidade: %.2f", quantityDifferenceAccumulate);

            if (quantityDifferenceAccumulate <= (minUnitDifference + 0.5)) // Até 0.5 de diferença
            {
                wakeup_time_sec = 0; // Não envia
            }
            else if (quantityDifferenceAccumulate <= (minUnitDifference + UnitDifferenceLowPriority + 0.5)) // De 0.5 a 1.5 de diferença
            {
                wakeup_time_sec = 90; // 15 min
            }
            else if (quantityDifferenceAccumulate <= (minUnitDifference + UnitDifferenceMediumPriority + 0.5)) // De 1.5 a 2.5 de diferença
            {
                wakeup_time_sec = 60; // 10 min
            }
            else if (quantityDifferenceAccumulate <= (minUnitDifference + UnitDifferenceHighPriority + 0.5)) // De 2.5 a 5.5 de diferença
            {
                wakeup_time_sec = 30; // 5 min
            }
            else if (quantityDifferenceAccumulate > (minUnitDifference + UnitDifferenceHighPriority + 0.5)) // Maior que 5.5
            {
                wakeup_time_sec = 6; // 1 min
            }

            lastQuantity = quantityUnits;
            lastHX711Total = HX711Total;
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        ESP_LOGI(TAG, "Not a deep sleep reset, initializing ULP");
        initVariablesFromNVS();
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

    uint32_t weightDifference = unitWeight * (minUnitDifference + 0.5) * measureSignalReference;
    // Acorda quando o valor medido é maior que o definido por Over
    ulp_trshHoldOverADMSB = (tare + weightDifference) >> 16;
    ulp_trshHoldOverADLSB = (tare + weightDifference) & 0xFFFF;
    // Acorda quando o valor medido é menor que o definido por Under
    ulp_trshHoldUnderADMSB = (tare - weightDifference) >> 16;
    ulp_trshHoldUnderADLSB = (tare - weightDifference) & 0xFFFF;

    /*// Acorda quando o valor medido é maior que o definido por Over
    ulp_trshHoldOverADMSB = 211;
    ulp_trshHoldOverADLSB = 30196;
    // Acorda quando o valor medido é menor que o definido por Under
    ulp_trshHoldUnderADMSB = 208;
    ulp_trshHoldUnderADLSB = 60000;*/
    // Seta se quiser só ler peso, limpa se quiser fazer a comparação
    ulp_repeatMeasureCount = 0;
    ulp_repeatMeasureQuantity = repeatMeasureQuantity + 1;
    ulp_setRepeatMeasure = 0;
    ulp_wakeUp = 1;

    ulp_set_wakeup_period(0, ulpWakeUpPeriod); // Set ULP wake up period T = 1s

    /* Start the program */
    err = ulp_run(&ulp_main - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

static uint32_t readWeight(void)
{
    while (!(ulp_dataReady & UINT16_MAX))
        ;
    ulp_set_wakeup_period(0, ulpWakeUpPeriod);

    /*uint32_t HX711HiWord_ulp = (ulp_HX711HiWord & UINT16_MAX);
    uint32_t HX711LoWord_ulp = (ulp_HX711LoWord & UINT16_MAX);
    uint32_t HX711Total = (HX711HiWord_ulp << 16) + HX711LoWord_ulp;*/

    uint32_t HX711HiWordAcc_ulp = (ulp_HX711HiWordAcc & UINT16_MAX);
    uint32_t HX711LoWordAcc_ulp = (ulp_HX711LoWordAcc & UINT16_MAX);
    uint32_t HX711TotalAcc = (HX711HiWordAcc_ulp << 16) + HX711LoWordAcc_ulp;
    HX711TotalAcc /= repeatMeasureQuantity;

    return HX711TotalAcc;
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

void nvsWriteUnsigned(const char *key, uint32_t value)
{
    nvs_handle handler_particao_nvs;
    esp_err_t err = nvs_flash_init_partition("nvs");

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar partição NVS.");
        return;
    }

    err = nvs_open_from_partition("nvs", "ns_nvs", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao abrir NVS como escrita/leitura");
        return;
    }

    /* Atualiza valor */
    err = nvs_set_u32(handler_particao_nvs, key, value);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro ao gravar");
        nvs_close(handler_particao_nvs);
        return;
    }
    else
    {
        ESP_LOGI(TAG, "Dado gravado com sucesso!");
        nvs_commit(handler_particao_nvs);
        nvs_close(handler_particao_nvs);
    }
}

uint32_t nvsReadUnsigned(const char *key)
{
    nvs_handle handler_particao_nvs;
    uint32_t value;
    esp_err_t err = nvs_flash_init_partition("nvs");

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar partição NVS.");
        return 0;
    }

    err = nvs_open_from_partition("nvs", "ns_nvs", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao abrir NVS como escrita/leitura");
        return 0;
    }

    /* Faz a leitura do dado associado a chave definida em CHAVE_NVS */
    err = nvs_get_u32(handler_particao_nvs, key, &value);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao fazer leitura do dado");
        nvs_close(handler_particao_nvs);
        return 0;
    }
    else
    {
        // ESP_LOGI(TAG, "Dado lido com sucesso!");
        nvs_close(handler_particao_nvs);
        return value;
    }
}

void nvsWriteSigned(const char *key, int32_t value)
{
    nvs_handle handler_particao_nvs;
    esp_err_t err = nvs_flash_init_partition("nvs");

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar partição NVS.");
        return;
    }

    err = nvs_open_from_partition("nvs", "ns_nvs", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao abrir NVS como escrita/leitura");
        return;
    }

    /* Atualiza valor */
    err = nvs_set_i32(handler_particao_nvs, key, value);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro ao gravar");
        nvs_close(handler_particao_nvs);
        return;
    }
    else
    {
        ESP_LOGI(TAG, "Dado gravado com sucesso!");
        nvs_commit(handler_particao_nvs);
        nvs_close(handler_particao_nvs);
    }
}

int32_t nvsReadSigned(const char *key)
{
    nvs_handle handler_particao_nvs;
    int32_t value;
    esp_err_t err = nvs_flash_init_partition("nvs");

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar partição NVS.");
        return 0;
    }

    err = nvs_open_from_partition("nvs", "ns_nvs", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao abrir NVS como escrita/leitura");
        return 0;
    }

    /* Faz a leitura do dado associado a chave definida em CHAVE_NVS */
    err = nvs_get_i32(handler_particao_nvs, key, &value);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao fazer leitura do dado");
        nvs_close(handler_particao_nvs);
        return 0;
    }
    else
    {
        // ESP_LOGI(TAG, "Dado lido com sucesso!");
        nvs_close(handler_particao_nvs);
        return value;
    }
}

void initVariablesFromNVS(void)
{
    tare = nvsReadUnsigned("tareValue");
    if (!tare)
    {
        nvsWriteUnsigned("tareValue", 1);
        tare = nvsReadUnsigned("tareValue");
    }
    ESP_LOGI(TAG, "Valor Tara: %d", tare);

    calibration = ((float)nvsReadSigned("calibrateValue") / 10000);
    if (!calibration)
    {
        nvsWriteSigned("calibrateValue", 1);
        calibration = ((float)nvsReadSigned("calibrateValue") / 10000);
    }
    ESP_LOGI(TAG, "Valor Calibração: %.2f", calibration);

    unitWeight = nvsReadSigned("unitWeightValue");
    if (!unitWeight)
    {
        nvsWriteSigned("unitWeightValue", 1);
        unitWeight = nvsReadSigned("unitWeightValue");
    }
    ESP_LOGI(TAG, "Valor Unitário: %d", unitWeight);
}

uint32_t measure_battery(int number_samples)
{
    esp_adc_cal_characteristics_t *adc_chars;
    const adc_channel_t channel = ADC_CHANNEL_6; // GPIO34 if ADC1
    const adc_bits_width_t width = ADC_WIDTH_BIT_12;
    const adc_atten_t atten = ADC_ATTEN_DB_0;
    const adc_unit_t unit = ADC_UNIT_1;
    gpio_num_t gpio_num_battery = GPIO_NUM_32;

    uint32_t default_vref = 1100;
    uint32_t r2_value = 100;
    uint32_t r3_value = 10;

    // Config gpio_bat
    rtc_gpio_init(gpio_num_battery);
    rtc_gpio_set_direction(gpio_num_battery, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num_battery);
    rtc_gpio_pullup_dis(gpio_num_battery);
    rtc_gpio_set_level(gpio_num_battery, 1);

    // Configure ADC1
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, default_vref, adc_chars);

    // Sample ADC1
    uint32_t adc_reading = 0;
    // Multisampling
    for (int i = 0; i < number_samples; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    rtc_gpio_set_level(gpio_num_battery, 0);
    adc_reading /= number_samples;
    // Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    uint32_t voltage_total = (voltage * (r2_value + r3_value)) / r3_value;

    return voltage_total;
}
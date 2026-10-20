/*
 * RETO 05 - CC3108 Internet of Things
 * Cerradura inteligente (RFID + servo) + monitoreo de humo (MQ-2)
 * Telemetría local por MQTT hacia Mosquitto en la Raspberry Pi
 *
 * Conexiones (ESP32):
 *   RC522  : SDA=GPIO5  SCK=GPIO18  MISO=GPIO19  MOSI=GPIO23  RST=GPIO22  3V3/GND
 *   MQ-2   : AO=GPIO34 (ADC1_CH6)  VCC=5V (VIN)  GND
 *   Servo  : señal=GPIO13  VCC=5V (VIN)  GND
 *   LED rojo: ánodo=GPIO2 (con resistencia) -> GND
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include <stdlib.h>

#include "mqtt_client.h"

#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"
#include "cJSON.h"


static const char *TAG = "RETO05";

/* ======================= CONFIGURACIÓN DEL USUARIO ======================= */

// Wi-Fi (la misma red/hotspot a la que está conectada la Raspberry Pi)
#define WIFI_SSID        "iPhone de Rodrigo"
#define WIFI_PASSWORD    "rok$1001"

// IP de la Raspberry Pi (resultado de `hostname -I`)
#define MQTT_BROKER_URI  "mqtt://172.20.10.2:1883"

// Topic y nombre del equipo (formato seccion/equipo/dato)
#define MQTT_TOPIC       "seccion10/equipo02"
#define EQUIPO           "equipo02"

// UID de la tarjeta blanca (autorizada). Escanéenla una vez, copien el UID
// que sale en el monitor serial ("Tarjeta detectada: XX XX XX XX") y péguenlo aquí.
#define UID_TARJETA_BLANCA "0B D5 8C CB"
  
/* ============================== PINES ==================================== */

#define RFID_SDA_GPIO   5      // CS del RC522
#define RFID_SCK_GPIO   18
#define RFID_MISO_GPIO  19
#define RFID_MOSI_GPIO  23
#define RFID_RST_GPIO   22

#define LED_ROJO_GPIO   2
#define SERVO_GPIO      13

// MQ-2 AO -> GPIO 34 = ADC1 canal 6
#define MQ2_ADC_CHANNEL ADC_CHANNEL_6

/* ============================== MQ-2 ===================================== */
/*
 * Conversión ADC -> PPM.
 * IMPORTANTE: si el docente les dio ecuaciones de calibración, reemplacen
 * el contenido de mq2_calcular_ppm() con ellas. Lo de abajo es una curva
 * típica de humo del MQ-2 (ppm = A * (Rs/R0)^B) y necesita su R0 real
 * (medido en aire limpio).
 */
#define MQ2_VCC          5.0f      // alimentación del módulo
#define MQ2_RL_KOHM      5.0f      // resistencia de carga del módulo (kΩ)
#define MQ2_R0_KOHM      10.0f     // R0: CALIBRAR en aire limpio
#define MQ2_CURVA_A      3616.1f
#define MQ2_CURVA_B      (-2.675f)
// Si usan divisor de voltaje en AO (recomendado), poner el factor. Ej: R1=R2 -> 2.0
#define MQ2_DIVISOR_FACTOR 1.0f

#define UMBRAL_HUMO_PPM  200.0f
#define PERIODO_MQ2_MS   5000

/* ============================== SERVO ==================================== */

#define SERVO_TIMER      LEDC_TIMER_0
#define SERVO_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_CHANNEL    LEDC_CHANNEL_0
#define SERVO_RES        LEDC_TIMER_14_BIT
#define SERVO_FREQ_HZ    50
#define SERVO_MIN_US     500       // 0 grados
#define SERVO_MAX_US     2500      // 180 grados
#define TIEMPO_ABIERTO_MS 5000

/* ============================== ESTADO =================================== */

static adc_oneshot_unit_handle_t adc_handle;
static esp_mqtt_client_handle_t mqtt_client = NULL;

static char ultimo_rfid_uid[32] = "NINGUNA";
static portMUX_TYPE rfid_mutex = portMUX_INITIALIZER_UNLOCKED;

static volatile bool mqtt_conectado   = false;
static volatile bool mqtt_iniciado    = false;
static volatile bool alerta_humo      = false;
static volatile bool acceso_denegado  = false;
static volatile bool puerta_ocupada   = false;

/* ============================== MQTT ===================================== */

static void mqtt_publicar(const char *payload)
{
    if (mqtt_client == NULL || !mqtt_conectado) {
        ESP_LOGW(TAG, "MQTT no conectado");
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return;
    }

    cJSON_AddStringToObject(root, "device_id", EQUIPO);
    cJSON_AddStringToObject(root, "evento", payload);

    char *json = cJSON_PrintUnformatted(root);
    if (json != NULL) {
        esp_mqtt_client_publish(
            mqtt_client, MQTT_TOPIC, json, 0, 1, 0);
        ESP_LOGI(TAG, "MQTT JSON -> %s", json);
        free(json);
    }

    cJSON_Delete(root);
}

// Nueva funcion ejer 06
static void enviar_telemetria_aws(esp_mqtt_client_handle_t cliente, long mq2_ppm,
                                  const char *rfid_uid, int angulo_servo)
{
    if (cliente == NULL || !mqtt_conectado) {
        ESP_LOGW(TAG, "MQTT no conectado, no se envió telemetría");
        return;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device_id", EQUIPO);

    cJSON *telemetria = cJSON_CreateObject();
    cJSON_AddNumberToObject(telemetria, "gas_mq2_ppm", mq2_ppm);
    cJSON_AddStringToObject(telemetria, "acceso_rfid_uid", rfid_uid);
    cJSON_AddNumberToObject(telemetria, "servo_angulo", angulo_servo);
    cJSON_AddItemToObject(root, "telemetria", telemetria);

    char *json_string = cJSON_PrintUnformatted(root);
    if (json_string != NULL) {
        esp_mqtt_client_publish(cliente, MQTT_TOPIC, json_string, 0, 1, 0);
        ESP_LOGI(TAG, "MQTT JSON -> %s : %s", MQTT_TOPIC, json_string);
        free(json_string);
    }

    cJSON_Delete(root);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    if (event_id == MQTT_EVENT_CONNECTED) {
        mqtt_conectado = true;
        ESP_LOGI("MQTT", "Conectado exitosamente al broker en la Raspberry Pi");
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        mqtt_conectado = false;
        ESP_LOGW("MQTT", "Desconectado del broker MQTT");
    }
}

static void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
}

/* ============================== WI-FI ==================================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW("WIFI", "Wi-Fi desconectado, reintentando...");
        esp_wifi_connect();

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI("WIFI", "Wi-Fi Conectado. Iniciando MQTT...");
        if (mqtt_client != NULL && !mqtt_iniciado) {
            esp_mqtt_client_start(mqtt_client);
            mqtt_iniciado = true;
        }
    }
}

static void wifi_init(void)
{
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi iniciado");
}

/* ============================== LED ====================================== */

static void actualizar_led_rojo(void)
{
    gpio_set_level(LED_ROJO_GPIO, (alerta_humo || acceso_denegado) ? 1 : 0);
}

static void led_init(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(LED_ROJO_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(LED_ROJO_GPIO, GPIO_MODE_OUTPUT));
    gpio_set_level(LED_ROJO_GPIO, 0);

    ESP_LOGI(TAG, "LED rojo listo");
}

/* ============================== SERVO ==================================== */

static void servo_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_MODE,
        .duty_resolution = SERVO_RES,
        .timer_num       = SERVO_TIMER,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    ESP_LOGI(TAG, "Servo listo");
}

static void servo_set_angle(int angulo)
{
    if (angulo < 0)   angulo = 0;
    if (angulo > 180) angulo = 180;

    uint32_t pulso_us = SERVO_MIN_US +
                        (uint32_t)angulo * (SERVO_MAX_US - SERVO_MIN_US) / 180;
    uint32_t periodo_us = 1000000 / SERVO_FREQ_HZ;              // 20000 us
    uint32_t duty = (pulso_us * (1u << 14)) / periodo_us;

    ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));
}

/* ============================== MQ-2 ===================================== */

static void mq2_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MQ2_ADC_CHANNEL,
                                               &channel_config));

    ESP_LOGI(TAG, "MQ-2 listo");
}

static float mq2_calcular_ppm(int lectura_adc)
{
    // ADC de 12 bits (0-4095) ~ 0-3.1 V con atenuación de 12 dB
    float v_adc = ((float)lectura_adc / 4095.0f) * 3.1f;
    float v_sensor = v_adc * MQ2_DIVISOR_FACTOR;   // voltaje real en AO

    if (v_sensor < 0.05f) {
        return 0.0f;
    }
    if (v_sensor > MQ2_VCC - 0.05f) {
        v_sensor = MQ2_VCC - 0.05f;
    }

    float rs = MQ2_RL_KOHM * (MQ2_VCC - v_sensor) / v_sensor;
    float ratio = rs / MQ2_R0_KOHM;

    return MQ2_CURVA_A * powf(ratio, MQ2_CURVA_B);
}

static void mq2_task(void *arg)
{
    int lectura_adc;
    

    while (1) {
        esp_err_t r = adc_oneshot_read(adc_handle, MQ2_ADC_CHANNEL, &lectura_adc);

        if (r == ESP_OK) {
            float ppm = mq2_calcular_ppm(lectura_adc);

            ESP_LOGI(TAG, "MQ-2 ADC: %d | PPM: %.2f", lectura_adc, ppm);

            alerta_humo = (ppm > UMBRAL_HUMO_PPM);
            if (alerta_humo) {
                ESP_LOGW(TAG, "ALERTA DE HUMO");
            }
            actualizar_led_rojo();

            char uid_copia[32];

        portENTER_CRITICAL(&rfid_mutex);
        snprintf(uid_copia, sizeof(uid_copia), "%s", ultimo_rfid_uid);
        portEXIT_CRITICAL(&rfid_mutex);

        enviar_telemetria_aws(mqtt_client, (long)ppm, uid_copia,
                            puerta_ocupada ? 90 : 0);
        } else {
            ESP_LOGE(TAG, "Error leyendo MQ-2");
        }

        // Esta es una tarea independiente: este delay NO bloquea al lector RFID
        vTaskDelay(pdMS_TO_TICKS(PERIODO_MQ2_MS));
    }
}

/* ============================ ACCESO RFID ================================ */

static void acceso_permitido_task(void *arg)
{
    char payload[64];

    ESP_LOGI(TAG, "Abriendo puerta (servo 90 grados)");
    servo_set_angle(90);

    snprintf(payload, sizeof(payload), "actuador/%s/ON", EQUIPO);
    mqtt_publicar(payload);

    vTaskDelay(pdMS_TO_TICKS(TIEMPO_ABIERTO_MS));

    servo_set_angle(0);
    ESP_LOGI(TAG, "Puerta cerrada (servo 0 grados)");

    snprintf(payload, sizeof(payload), "actuador/%s/OFF", EQUIPO);
    mqtt_publicar(payload);

    puerta_ocupada = false;
    vTaskDelete(NULL);
}

static void acceso_denegado_task(void *arg)
{
    char payload[64];

    acceso_denegado = true;
    actualizar_led_rojo();
    ESP_LOGW(TAG, "LED ROJO ENCENDIDO");

    snprintf(payload, sizeof(payload), "acceso/%s/DENEGADO", EQUIPO);
    mqtt_publicar(payload);

    vTaskDelay(pdMS_TO_TICKS(2000));

    acceso_denegado = false;
    actualizar_led_rojo();

    vTaskDelete(NULL);
}

static void rc522_handler(void *arg, esp_event_base_t base,
                          int32_t event_id, void *event_data)
{
    rc522_picc_state_changed_event_t *event =
        (rc522_picc_state_changed_event_t *)event_data;
    rc522_picc_t *picc = event->picc;

    if (picc->state != RC522_PICC_STATE_ACTIVE) {
        return;
    }

    char uid_texto[32];
    rc522_picc_uid_to_str(&picc->uid, uid_texto, sizeof(uid_texto));

        portENTER_CRITICAL(&rfid_mutex);
    snprintf(ultimo_rfid_uid, sizeof(ultimo_rfid_uid), "%s", uid_texto);
    portEXIT_CRITICAL(&rfid_mutex);

    ESP_LOGI(TAG, "Tarjeta detectada: %s", uid_texto);

    if (strcmp(uid_texto, UID_TARJETA_BLANCA) == 0) {
        ESP_LOGI(TAG, "ACCESO PERMITIDO");

        if (puerta_ocupada) {
            ESP_LOGW(TAG, "La puerta ya está abierta, se ignora la lectura");
            return;
        }
        puerta_ocupada = true;
        xTaskCreate(acceso_permitido_task, "acceso_permitido", 3072, NULL, 5, NULL);

    } else {
        ESP_LOGW(TAG, "ACCESO DENEGADO");
        xTaskCreate(acceso_denegado_task, "acceso_denegado", 3072, NULL, 5, NULL);
    }
}

static void rfid_init(void)
{
    spi_bus_config_t bus_config = {
        .miso_io_num   = RFID_MISO_GPIO,
        .mosi_io_num   = RFID_MOSI_GPIO,
        .sclk_io_num   = RFID_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    rc522_spi_config_t spi_config = {
        .host_id    = SPI2_HOST,
        .bus_config = &bus_config,
        .dev_config = {
            .spics_io_num = RFID_SDA_GPIO,
        },
        .rst_io_num = RFID_RST_GPIO,
    };

    rc522_driver_handle_t driver;
    ESP_ERROR_CHECK(rc522_spi_create(&spi_config, &driver));
    ESP_ERROR_CHECK(rc522_driver_install(driver));

    rc522_config_t scanner_config = {
        .driver = driver,
    };

    rc522_handle_t scanner;
    ESP_ERROR_CHECK(rc522_create(&scanner_config, &scanner));
    ESP_ERROR_CHECK(rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED,
                                          rc522_handler, NULL));
    ESP_ERROR_CHECK(rc522_start(scanner));

    ESP_LOGI(TAG, "RFID listo");
}

/* ============================== MAIN ===================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando RETO 05");

    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(r);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    led_init();
    servo_init();
    servo_set_angle(0);       // puerta cerrada al iniciar
    mq2_init();

    mqtt_init();              // el cliente se crea; arranca cuando haya IP
    wifi_init();

    rfid_init();

    xTaskCreate(mq2_task, "mq2_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Sistema listo");
}
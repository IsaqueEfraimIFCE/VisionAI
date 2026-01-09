// main.c
/**
 * ESP32-CAM 
 * Captura e envia imagens JPEG via WebSocket a cada 250 ms
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "esp_camera.h"
#include "esp_websocket_client.h"

#define BOARD_ESP32CAM_AITHINKER
static const char *TAG = "cam_ws_fast";

/* ============== CONFIGURAÇÃO ============== */
#define WIFI_SSID   "jonathan"
#define WIFI_PASS   "123456jon"
#define SERVER_IP   "192.168.246.140"
#define SERVER_PORT 8765

#define CAPTURE_INTERVAL_MS 250
/* ========================================= */

static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

#include "camera_pinout.h"  

/* ========== Configuração da câmera ========== */
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 10,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* ===== Wi-Fi events ===== */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi desconectado, tentando reconectar...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi conectado e IP obtido");
    }
}

static esp_err_t wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Conectando ao WiFi: %s ...", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(15000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Conectado ao WiFi!");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Falha ao conectar ao WiFi");
        return ESP_FAIL;
    }
}

/* ===== Inicializa câmera ===== */
static esp_err_t init_camera(void)
{
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar câmera: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "Câmera inicializada com sucesso!");
    return ESP_OK;
}

/* ===== WebSocket events ===== */
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
        break;
    case WEBSOCKET_EVENT_DATA:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_DATA, len=%d, opcode=%d", data->data_len, data->op_code);
        break;
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "WEBSOCKET_EVENT_ERROR");
        if (data && data->data_ptr) {
            ESP_LOGE(TAG, "Erro: %s", (char*)data->data_ptr);
        }
        break;
    default:
ESP_LOGI(TAG, "WEBSOCKET event id: %" PRId32, event_id);
        break;
    }
}

/* ========== APP MAIN ========== */
void app_main(void)
{
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Wi-Fi
    if (wifi_init_sta() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed, abortando");
        return;
    }

    // Câmera
    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed, abortando");
        return;
    }

    char ws_uri[64];
    snprintf(ws_uri, sizeof(ws_uri), "ws://%s:%d/", SERVER_IP, SERVER_PORT);
    ESP_LOGI(TAG, "Conectando ao WebSocket em %s", ws_uri);

    esp_websocket_client_config_t ws_cfg = {
        .uri = ws_uri,
    };

    esp_websocket_client_handle_t client = esp_websocket_client_init(&ws_cfg);
    if (!client) {
        ESP_LOGE(TAG, "Falha ao inicializar cliente WebSocket");
        return;
    }

    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, NULL);

    esp_err_t r = esp_websocket_client_start(client);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar cliente WebSocket: 0x%x", r);
        esp_websocket_client_destroy(client);
        return;
    }

    // espera conexão
    int wait = 0;
    while (!esp_websocket_client_is_connected(client) && wait < 100) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait++;
    }

    if (!esp_websocket_client_is_connected(client)) {
        ESP_LOGE(TAG, "Falha ao conectar ao WebSocket depois da espera");
        esp_websocket_client_stop(client);
        esp_websocket_client_destroy(client);
        return;
    }

    ESP_LOGI(TAG, "Conectado ao WebSocket, iniciando loop de captura");

    // Loop principal
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Falha ao capturar imagem");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (fb->format != PIXFORMAT_JPEG) {
            ESP_LOGW(TAG, "Frame não está em JPEG (format=%d). Ignorando.", fb->format);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(CAPTURE_INTERVAL_MS));
            continue;
        }

        if (esp_websocket_client_is_connected(client)) {
            int sent = esp_websocket_client_send_bin(client, (const char *)fb->buf, fb->len, portMAX_DELAY);
            if (sent > 0) {
                ESP_LOGI(TAG, "Imagem enviada: %d bytes", sent);
            } else {
                ESP_LOGE(TAG, "Erro ao enviar imagem (ret=%d). Reiniciando cliente...", sent);
                // tenta reiniciar o cliente de forma segura
                esp_websocket_client_stop(client);
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_websocket_client_start(client);
            }
        } else {
            ESP_LOGW(TAG, "WebSocket desconectado. Tentando reconectar...");
            esp_websocket_client_stop(client);
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_websocket_client_start(client);
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(CAPTURE_INTERVAL_MS));
    }

    // cleanup (nunca chega aqui normalmente)
    esp_websocket_client_stop(client);
    esp_websocket_client_destroy(client);
}

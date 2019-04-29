#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "esp_event_loop.h"
#include "esp_http_server.h"

#include "quirc.h"
#include "qr_recoginize.h"

static const char* TAG = "camera";

#define M5_CAM_KIND 2 // 1 --> A model, 2 --> B model
// #define FISH_EYE_CAM  // fish eye need flip image
#define CAM_USE_WIFI

// #if M5_CAM_KIND == 1
// #define CAM_PIN_SIOD    25
// #define CAM_PIN_VSYNC   22
// #else
// #define CAM_PIN_SIOD    22
// #define CAM_PIN_VSYNC   25
// #endif

//M5STACK_CAM PIN Map
// #define CAM_PIN_RESET   15 //software reset will be performed
// #define CAM_PIN_XCLK    27
// #define CAM_PIN_SIOC    23

// #define CAM_PIN_D7      19
// #define CAM_PIN_D6      36
// #define CAM_PIN_D5      18
// #define CAM_PIN_D4      39
// #define CAM_PIN_D3      5
// #define CAM_PIN_D2      34
// #define CAM_PIN_D1      35
// #define CAM_PIN_D0      32

// #define CAM_PIN_HREF    26
// #define CAM_PIN_PCLK    21

// #define PWDN_GPIO_NUM    -1
// #define CAM_PIN_RESET   -1
// #define CAM_PIN_XCLK    4
// #define CAM_PIN_SIOD    18
// #define CAM_PIN_SIOC    23

// #define CAM_PIN_D7      36
// #define CAM_PIN_D6      37
// #define CAM_PIN_D5      38
// #define CAM_PIN_D4      39
// #define CAM_PIN_D3      35
// #define CAM_PIN_D2      14
// #define CAM_PIN_D1      13
// #define CAM_PIN_D0      34
// #define CAM_PIN_VSYNC   5
// #define CAM_PIN_HREF    27
// #define CAM_PIN_PCLK    25

#define PWDN_GPIO_NUM    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    21
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      19
#define CAM_PIN_D2      18
#define CAM_PIN_D1       5
#define CAM_PIN_D0       4
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

#define CAM_XCLK_FREQ   10000000

#define ESP_WIFI_SSID "M5Psram_Cam"
#define ESP_WIFI_PASS ""
#define MAX_STA_CONN  1

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static EventGroupHandle_t s_wifi_event_group;
static ip4_addr_t s_ip_addr;
const int CONNECTED_BIT = BIT0;
extern void led_brightness(int duty);
static camera_config_t camera_config = {
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

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

    //XCLK 20MHz or 10MHz
    .xclk_freq_hz = CAM_XCLK_FREQ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    //.pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    //.frame_size = FRAMESIZE_HQVGA,//QQVGA-UXGA Do not use sizes above QVGA when not JPEG
    .pixel_format = PIXFORMAT_GRAYSCALE,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QQVGA,//QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 3 //if more than one, i2s runs in continuous mode. Use only with JPEG
};

static void wifi_init_softap();
static esp_err_t http_server_init();

#define PRINT_QR 0

static const char *data_type_str(int dt)
{
    switch (dt) {
    case QUIRC_DATA_TYPE_NUMERIC:
        return "NUMERIC";
    case QUIRC_DATA_TYPE_ALPHA:
        return "ALPHA";
    case QUIRC_DATA_TYPE_BYTE:
        return "BYTE";
    case QUIRC_DATA_TYPE_KANJI:
        return "KANJI";
    }
    return "unknown";
}

static void dump_cells(const struct quirc_code *code)
{
    int u = 0, v = 0;

    printf("    %d cells, corners:", code->size);
    for (u = 0; u < 4; u++) {
        printf(" (%d,%d)", code->corners[u].x, code->corners[u].y);
    }
    printf("\n");

    for (v = 0; v < code->size; v++) {
        printf("    ");
        for (u = 0; u < code->size; u++) {
            int p = v * code->size + u;

            if (code->cell_bitmap[p >> 3] & (1 << (p & 7))) {
                printf("[]");
            } else {
                printf("  ");
            }
        }
        printf("\n");
    }
}

static void dump_data(const struct quirc_data *data)
{
    printf("    Version: %d\n", data->version);
    printf("    ECC level: %c\n", "MLHQ"[data->ecc_level]);
    printf("    Mask: %d\n", data->mask);
    printf("    Data type: %d (%s)\n", data->data_type,
           data_type_str(data->data_type));
    printf("    Length: %d\n", data->payload_len);
    printf("    Payload: %s\n", data->payload);

    if (data->eci) {
        printf("    ECI: %d\n", data->eci);
    }
}

static void dump_info(struct quirc *q, uint8_t count)
{
    printf("%d QR-codes found:\n\n", count);
    for (int i = 0; i < count; i++) {
        struct quirc_code code;
        struct quirc_data data;

        // Extract the QR-code specified by the given index.
        quirc_extract(q, i, &code);

        //Decode a QR-code, returning the payload data.
        quirc_decode_error_t err = quirc_decode(&code, &data);

#if PRINT_QR
        dump_cells(&code);
        printf("\n");
#endif

        if (err) {
            printf("  Decoding FAILED: %s\n", quirc_strerror(err));
        } else {
            printf("  Decoding successful:\n");
            printf("    %d cells, corners:", code.size);
            for (uint8_t u = 0; u < 4; u++) {
                printf(" (%d,%d)", code.corners[u].x, code.corners[u].y);
            }
            printf("\n");
            dump_data(&data);
        }
        printf("\n");
    }
}

// void qr_recoginze(void *parameter)
// {
//     camera_config_t *camera_config = (camera_config_t *)parameter;
//     // Use VGA Size currently, but quirc can support other frame size.(eg: FRAMESIZE_SVGA,FRAMESIZE_VGA，
//     // FRAMESIZE_CIF,FRAMESIZE_QVGA,FRAMESIZE_HQVGA,FRAMESIZE_QCIF,FRAMESIZE_QQVGA2,FRAMESIZE_QQVGA,etc)
//     if (camera_config->frame_size > FRAMESIZE_VGA) {
//         ESP_LOGE(TAG, "Camera Frame Size err %d", (camera_config->frame_size));
//         vTaskDelete(NULL);
//     }

//     // Save image width and height, avoid allocate memory repeatly.
//     uint16_t old_width = 0;
//     uint16_t old_height = 0;

//     // Construct a new QR-code recognizer.
//     ESP_LOGI(TAG, "Construct a new QR-code recognizer(quirc).");
//     struct quirc *qr_recognizer = quirc_new();
//     if (!qr_recognizer) {
//         ESP_LOGE(TAG, "Can't create quirc object");
//     }
//     camera_fb_t *fb = NULL;
//     uint8_t *image = NULL;
//     int id_count = 0;
//     UBaseType_t uxHighWaterMark;

//     /* 入口处检测一次 */
//     ESP_LOGI(TAG, "uxHighWaterMark = %d", uxTaskGetStackHighWaterMark( NULL ));

//     while (1) {
//         ESP_LOGI(TAG, "uxHighWaterMark = %d", uxTaskGetStackHighWaterMark( NULL ));
//         // Capture a frame
//         fb = esp_camera_fb_get();
//         if (!fb) {
//             ESP_LOGE(TAG, "Camera capture failed");
//             continue;
//         }

//         if (old_width != fb->width || old_height != fb->height) {
//             ESP_LOGD(TAG, "Recognizer size change w h len: %d, %d, %d", fb->width, fb->height, fb->len);
//             ESP_LOGI(TAG, "Resize the QR-code recognizer.");
//             // Resize the QR-code recognizer.
//             if (quirc_resize(qr_recognizer, fb->width, fb->height) < 0) {
//                 ESP_LOGE(TAG, "Resize the QR-code recognizer err.");
//                 continue;
//             } else {
//                 old_width = fb->width;
//                 old_height = fb->height;
//             }
//         }

//         /** These functions are used to process images for QR-code recognition.
//          * quirc_begin() must first be called to obtain access to a buffer into
//          * which the input image should be placed. Optionally, the current
//          * width and height may be returned.
//          *
//          * After filling the buffer, quirc_end() should be called to process
//          * the image for QR-code recognition. The locations and content of each
//          * code may be obtained using accessor functions described below.
//          */
//         image = quirc_begin(qr_recognizer, NULL, NULL);
//         memcpy(image, fb->buf, fb->len);
//         quirc_end(qr_recognizer);

//         // Return the number of QR-codes identified in the last processed image.
//         id_count = quirc_count(qr_recognizer);
//         if (id_count == 0) {
//             ESP_LOGE(TAG, "Error: not a valid qrcode");
//             esp_camera_fb_return(fb);
//             continue;
//         }

//         // Print information of QR-code
//         dump_info(qr_recognizer, id_count);
//         esp_camera_fb_return(fb);
//     }
//     // Destroy QR-Code recognizer (quirc)
//     quirc_destroy(qr_recognizer);
//     ESP_LOGI(TAG, "Deconstruct QR-Code recognizer(quirc)");
//     vTaskDelete(NULL);
// }

void app_qr_recognize(void *pdata)
{
    // xTaskCreate(qr_recoginze, "qr_recoginze", 1024 * 116, pdata, 5, NULL);
    camera_fb_t * fb = NULL;        
    while(true){
        vTaskDelay(100);
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }

        qr_recoginze(fb);
        esp_camera_fb_return(fb);
    }
    
}

void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_INFO);
    
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }

    err = esp_camera_init(&camera_config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        for(;;) {
            vTaskDelay(10);
        }
    } else {
        led_brightness(20);
    }

#ifdef FISH_EYE_CAM
    // flip img, other cam setting view sensor.h
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
#endif

    

#ifdef CAM_USE_WIFI
    wifi_init_softap();

    vTaskDelay(100 / portTICK_PERIOD_MS);
    http_server_init();
#endif

    // app_qr_recognize(&camera_config);
}

#ifdef CAM_USE_WIFI

esp_err_t jpg_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t fb_len = 0;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    res = httpd_resp_set_type(req, "image/jpeg");
    if(res == ESP_OK){
        res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    }

    if(res == ESP_OK){
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    }

    //qr_recoginze(fb);
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    ESP_LOGI(TAG, "JPG: %uKB %ums", (uint32_t)(fb_len/1024), (uint32_t)((fr_end - fr_start)/1000));
    return res;
}
 

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

   
    while(true){
        ESP_LOGI(TAG, "uxHighWaterMark = %d", uxTaskGetStackHighWaterMark( NULL ));
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
        } else {
            if(fb->format != PIXFORMAT_JPEG){
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                if(!jpeg_converted){
                    ESP_LOGE(TAG, "JPEG compression failed");
                    esp_camera_fb_return(fb);
                    res = ESP_FAIL;
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(fb->format != PIXFORMAT_JPEG){
            free(_jpg_buf);
        }
        qr_recoginze(fb);
        esp_camera_fb_return(fb);
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        //ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)",
            //(uint32_t)(_jpg_buf_len/1024),
            //(uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    }

    last_frame = 0;
    return res;
}

static esp_err_t http_server_init(){
    httpd_handle_t server;
    httpd_uri_t jpeg_uri = {
        .uri = "/jpg",
        .method = HTTP_GET,
        .handler = jpg_httpd_handler,
        .user_ctx = NULL
    };

    httpd_uri_t jpeg_stream_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = jpg_stream_httpd_handler,
        .user_ctx = NULL
    };

    httpd_config_t http_options = HTTPD_DEFAULT_CONFIG();
    //!add
    http_options.stack_size = 111500;

    ESP_ERROR_CHECK(httpd_start(&server, &http_options));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &jpeg_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &jpeg_stream_uri));

    return ESP_OK;
}

static esp_err_t event_handler(void* ctx, system_event_t* event) 
{
  switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
      s_ip_addr = event->event_info.got_ip.ip_info.ip;
      xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d", MAC2STR(event->event_info.sta_connected.mac),
               event->event_info.sta_connected.aid);
      xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d", MAC2STR(event->event_info.sta_disconnected.mac),
               event->event_info.sta_disconnected.aid);
      xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      esp_wifi_connect();
      xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    default:
      break;
  }
  return ESP_OK;
}

static void wifi_init_softap() 
{
  s_wifi_event_group = xEventGroupCreate();

  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  wifi_config_t wifi_config = {
      .ap = {.ssid = ESP_WIFI_SSID,
             .ssid_len = strlen(ESP_WIFI_SSID),
             .password = ESP_WIFI_PASS,
             .max_connection = MAX_STA_CONN,
             .authmode = WIFI_AUTH_WPA_WPA2_PSK},
  };
  if (strlen(ESP_WIFI_PASS) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  uint8_t addr[4] = {192, 168, 4, 1};
  s_ip_addr = *(ip4_addr_t*)&addr;

  ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
           ESP_WIFI_SSID, ESP_WIFI_PASS);
}

#endif

/*
  * ESPRESSIF MIT License
  *
  * Copyright (c) 2017 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
  *
  * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
  * it is free of charge, to any person obtaining a copy of this software and associated
  * documentation files (the "Software"), to deal in the Software without restriction, including
  * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all copies or
  * substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
  * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
  * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  */
#include <stdio.h>
#include <string.h>
#include "quirc_internal.h"
#include "qr_recoginize.h"
#include "esp_camera.h"
#include "quirc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

static char *TAG = "QR";

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

void dump_cells(const struct quirc_code *code)
{
    int u, v;

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

void dump_data(const struct quirc_data *data)
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
        quirc_decode_error_t err;

        quirc_extract(q, i, &code);
        err = quirc_decode(&code, &data);

        // dump_cells(&code);
        // printf("\n");

        if (err) {
            printf("  Decoding FAILED: %s\n", quirc_strerror(err));
        } else {
            printf("  Decoding successful:\n");
            dump_data(&data);
        }

        printf("\n");
    }
}

void qr_recoginze(void *parameter)
{
    camera_config_t *camera_config = (camera_config_t *)parameter;
    if (camera_config->frame_size > FRAMESIZE_VGA) {
        ESP_LOGE(TAG, "Camera Size err %d", (camera_config->frame_size));
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "begin to qr_recoginze\r\n");
    struct quirc *q;
    q = quirc_new();
    if (!q) {
        ESP_LOGE(TAG, "can't create quirc object\r\n");
    }
    ESP_LOGD(TAG, "begin to quirc_resize\r\n");
    while (1) {

        //capture a frame
        camera_fb_t *fb = esp_camera_fb_get();
        ESP_LOGE(TAG, "w h %d, %d, %d\r\n", fb->width, fb->height, fb->len);
        if (quirc_resize(q, fb->width, fb->height) < 0) {
            ESP_LOGE(TAG, "quirc_resize err\r\n");
            continue;
        }

        uint8_t *image = quirc_begin(q, NULL, NULL);
        memcpy(image, fb->buf, fb->len);
        quirc_end(q);
        int id_count = quirc_count(q);
        if (id_count == 0) {
            ESP_LOGE(TAG, "Error: not a valid qrcode\n");
        	esp_camera_fb_return(fb);
            continue;
        }

        dump_info(q, id_count);
        esp_camera_fb_return(fb);
    }
    quirc_destroy(q);
    ESP_LOGI(TAG, "finish recoginize\r\n");
    vTaskDelete(NULL);
}

void debug_buf(uint8_t * buf, size_t width, size_t height)
{
    for (uint16_t i = 0; i < height; i++) {
        for(uint16_t j = 0; j < width; j++)
            printf("%02x", *(buf+i*width+j));
        printf("\n");
    }
}

void app_qr_recognize(void *pdata)
{
    xTaskCreate(qr_recoginze, "qr_recoginze", 111500, pdata, 5, NULL);
}
/***********************************************************************************************************************
 *
 * @file    main.c
 * @brief   Main Source File.
 * @date    27/07/2023
 * @author  Fernando Fontes
 * @note    Platform Target: ESP32-WROOM-32D
 *
 * TAB SIZE: 4 SPACES
 *
 ***********************************************************************************************************************/

/****************************************************** Includes ******************************************************/
#include <stdio.h>
#include <stdio.h>
#include <stdbool.h>
#include "string.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "math.h"
#include "time.h"

#include "simcom.h"
#include "howsmyssl_cert.h"
#include "postdata_cert.h"
#include "dummy_cert.h"

/****************************************************** Defines *******************************************************/

/****************************************************** Typedefs ******************************************************/

/***************************************************** Constants ******************************************************/
static const char *TAG = "main";

/***************************************************** Variables ******************************************************/

/*********************************************** Internal Declarations ************************************************/

/***************************************************** Functions ******************************************************/
void app_main()
{
    uint8_t error_counter = 0;

    /* Initialize Driver Related */
    if (simcom_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize simcom module!");
        error_counter++;
    }

    if (error_counter > 0)
    {
        ESP_LOGE(TAG, "Entering in faulty mode!\n");
        for (;;)
        {
        }
    }
    else
    {
        ESP_LOGI(TAG, "All done! Init OK!\n");
    }

    ///// TEST
    for (;;)
    {
        if (simcom_alive())
        {
            ////////////////////////////////////////
            /////////////// GET EXAMPLE ////////////
            ////////////////////////////////////////
            SIMCOM_SSL_Request_t get_ssl_request;
            SIMCOM_SSL_Response_t get_ssl_response;
            char get_endpoint[] = "/a/check";
            char get_host[] = "www.howsmyssl.com";
            char get_root_cert_filename[] = "howsmyssl.pem";

            get_ssl_request.endpoint = get_endpoint;
            get_ssl_request.host_server = get_host;
            get_ssl_request.headers = NULL;
            get_ssl_request.root_cert_filename = get_root_cert_filename;
            get_ssl_request.root_cert = howsmyssl_root_cert_pem;
            get_ssl_request.force_write_cert = false;
            get_ssl_request.force_insecure = false;
            get_ssl_request.ssl_request_method = SIMCOM_SSL_GET;

            ESP_LOGI(TAG, "GET Example.");
            if (simcom_ssl_call(&get_ssl_request, &get_ssl_response) == ESP_OK)
            {

                ESP_LOGI(TAG, "Status Code: %u Data: %.*s Len: %d\r\n", get_ssl_response.status_code, get_ssl_response.data_length, (char *)get_ssl_response.data, get_ssl_response.data_length);

                if (get_ssl_response.data != NULL)
                {
                    vPortFree(get_ssl_response.data);
                }
            }

            // Pause 1 second between examples
            vTaskDelay(pdMS_TO_TICKS(1000));

            ////////////////////////////////////////
            ////////////// POST EXAMPLE ////////////
            ////////////////////////////////////////
            SIMCOM_SSL_Request_t post_ssl_request;
            SIMCOM_SSL_Response_t post_ssl_response;
            char post_endpoint[] = "/add/simcom";
            char post_host[] = "api.postdata.cloud";
            char post_root_cert_filename[] = "postDataRoot.pem";
            char post_payload[] = "{\"temperature\":22.5}";
            // char headers[] = "authorization: token";

            post_ssl_request.endpoint = post_endpoint;
            post_ssl_request.host_server = post_host;
            // post_ssl_request.headers = headers;
            post_ssl_request.headers = NULL;
            post_ssl_request.root_cert_filename = post_root_cert_filename;
            post_ssl_request.root_cert = post_data_root_cert_pem;
            post_ssl_request.force_write_cert = false;
            post_ssl_request.force_insecure = false;
            post_ssl_request.ssl_request_method = SIMCOM_SSL_POST;
            post_ssl_request.payload = post_payload;

            ESP_LOGI(TAG, "POST Example.");
            if (simcom_ssl_call(&post_ssl_request, &post_ssl_response) == ESP_OK)
            {

                ESP_LOGI(TAG, "Status Code: %u Data: %.*s Len: %d\r\n", post_ssl_response.status_code, post_ssl_response.data_length, (char *)post_ssl_response.data, post_ssl_response.data_length);

                if (post_ssl_response.data != NULL)
                {
                    vPortFree(post_ssl_response.data);
                }
            }

            break;
        }

        // delay 5 sec
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/************************************************* Internal Functions *************************************************/

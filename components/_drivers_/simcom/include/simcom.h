/***********************************************************************************************************************
 *
 * @file    simcom.h
 * @brief   SIMCOM A7xx modem header.
 * @date    27/07/2023
 * @author  Fernando Fontes
 * @note    Platform Target: ESP32-WROOM-32D
 *
 * TAB SIZE: 4 SPACES
 *
 ***********************************************************************************************************************/

#ifndef SIMCOM_A76XX_H_
#define SIMCOM_A76XX_H_

/****************************************************** Includes ******************************************************/
/* General includes. */
#include <stdint.h>

/****************************************************** Defines *******************************************************/
#define SIMCOM_SIM_CARD_APN "onomondo"
#define SIMCOM_SIM_CARD_APN_MAX_SIZE 30
#define SIMCOM_NTP_SYNC_SERVER "time.google.com"
#define SIMCOM_NTP_NAMESPACE_MAX_SIZE 30
#define SIMCOM_CERTFILE_NAME_MAX_SIZE 30

/****************************************************** Typedefs ******************************************************/
typedef enum 
{
    SIMCOM_SSL_GET = 0,
    SIMCOM_SSL_POST = 1
} SIMCOM_SSL_Request_Methods_t;

typedef struct
{
    char *root_cert_filename;
    const char *root_cert;
    bool force_write_cert;
    bool force_insecure;
    SIMCOM_SSL_Request_Methods_t ssl_request_method;
    char *host_server;
    char *endpoint;
    char *payload;
    char * headers;
} SIMCOM_SSL_Request_t;

typedef struct 
{
    char *data;
    int data_length;
    int status_code;
} SIMCOM_SSL_Response_t;

/***************************************************** Constants ******************************************************/

/***************************************************** Variables ******************************************************/

/***************************************************** Functions ******************************************************/
/**
 *
 * @brief   Creates a freertos task to handle simcom initialization.
 *          This task should check the health of the simcom module.
 * @param   void
 * @return  esp_err_t - ESP_OK
 *                    - ESP_FAIL
 *
 **/
int simcom_init(void);

/**
 *
 * @brief   Check if the simcom is alive.
 * @param   void
 * @return  bool - true  : alive
 *               - false : not-alive
 *
 **/
bool simcom_alive(void);

/**
 *
 * @brief   Execute a TLS/SSL call to a given server.
 *          At this moment it supports both GET and POST requests.
 * @param   void
 * @return  int  - ESP_OK 
 *               - ESP_FAIL
 *
 **/
int simcom_ssl_call(SIMCOM_SSL_Request_t *ssl_request, SIMCOM_SSL_Response_t *response);

#endif
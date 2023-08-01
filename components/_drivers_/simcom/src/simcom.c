/***********************************************************************************************************************
 *
 * @file    simcom.c
 * @brief   SIMCOM A7xx modem API.
 * @date    27/07/2023
 * @author  Fernando Fontes
 * @note    Platform Target: ESP32-WROOM-32D
 *
 * TAB SIZE: 4 SPACES
 *
 ***********************************************************************************************************************/

/****************************************************** Includes ******************************************************/
/* General includes. */
#include <stdio.h>
#include <stdio.h>
#include <stdbool.h>
#include "string.h"
#include "esp_system.h"
#include "esp_log.h"
#include "math.h"
#include "time.h"
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "simcom.h"
#include "board_config.h"
#include "tasks_config.h"

/****************************************************** Defines *******************************************************/
#define RX_BUF_SIZE 2048
#define NUM_RESPONSES 26

#define RET_NONE 0x000000000 /* RET_NONE shall be 0x0: don't change this value! */
#define RET_CRLF 0x000000001
#define RET_OK 0x000000002 /* do not change this value */
#define RET_SIM_READY 0x000000004
#define RET_ARROW 0x000000008
#define RET_SENT 0x000000010      /* do not change this value */
#define RET_ERROR 0x000000020     /* do not change this value */
#define RET_BUF_FULL 0x000000040  /* do not change this value */
#define RET_CME_ERROR 0x000000080 /* do not change this value */
#define RET_RSSI_SIGNAL 0x000000100
#define RET_CREG 0x000000200
#define RET_CGREG 0x000000400
#define RET_CEREG 0x000000800
#define RET_CERTLIST 0x000001000
#define RET_CHSTART 0x000002000
#define RET_CHOPEN 0x000004000
#define RET_CHSTOP 0x000008000
#define RET_CHSEND 0x000010000
#define RET_NTP 0x000020000
#define RET_CLK 0x000040000
#define RET_CHSET 0x000080000
#define RET_CHSSLCFG 0x000100000
#define RET_CHRECVDATA 0x000200000
#define RET_CHCLOSE 0x000400000
#define RET_CHOPEN_CONNECT 0x000800000

#define SIMCOM_MUTEX_LOCK_TIMEOUT 1000
#define UART_TX_READY_TIMEOUT 3000
#define SIMCOM_WAIT_SIM_READY_ATTEMPTS 10
#define SIMCOM_DEFAULT_CONTEXT 1
#define SIMCOM_DEFAULT_SSL_CONTEXT 0
#define SIMCOM_DEFAULT_SSL_SESSION_ID 0
#define SIMCOM_WAIT_CELLULAR_CONNECTION_ATTEMPTS 10
#define SIMCOM_GET_RSSI_ATTEMPTS 5

/****************************************************** Typedefs ******************************************************/
typedef struct
{
    uint32_t retval;
    char retstr[100];
} SIMCOM_RetKeywords_t;

typedef enum
{
    CFUN_MINIMUM_FUNCTIONALITY = 0,
    CFUN_FULL_FUNCTIONALITY = 1,
    CFUN_DISABLE_PHONE_TX_RX_CIRCUITS = 4,
    CFUN_FACTORY_TEST = 5,
    CFUN_RESET = 6,
    CFUN_OFFLINE_MODE = 7
} SIMCOM_CFUN_Values_t;

typedef enum
{
    SIMCOM_Response_Time_Short = 100,
    SIMCOM_Response_Time_120s = 120000,
    SIMCOM_Response_Time_9s = 9000,
    SIMCOM_Response_Time_15s = 15000,
    SIMCOM_Response_Time_150s = 150000
} SIMCOM_MAX_Response_Time_t;
/***************************************************** Constants ******************************************************/
static const char *TAG = "SIMCOM";

/***************************************************** Variables ******************************************************/
// simcom receive buffer
static char _buffer[RX_BUF_SIZE + 1] = {'\0'};

// simcom send/receive related keywords
const SIMCOM_RetKeywords_t ReturnKeywords[] = {
    {RET_SENT, "SEND OK\r\n"},
    {RET_ARROW, ">"},
    {RET_ERROR, "ERROR\r\n"},
    {RET_CME_ERROR, "+CME ERROR: "},
    {RET_BUF_FULL, "SEND FAIL\r\n"},
    {RET_OK, "OK\r\n"},
    {RET_SIM_READY, "+CPIN: READY\r\n"},
    {RET_RSSI_SIGNAL, "+CSQ: "},
    {RET_CREG, "+CREG: "},
    {RET_CGREG, "+CGREG: "},
    {RET_CGREG, "+CEREG: "},
    {RET_CERTLIST, "+CCERTLIST: "},
    {RET_CHSTART, "+CCHSTART"},
    {RET_CHOPEN, "+CCHOPEN: "},
    {RET_CHSTOP, "+CCHSTOP: "},
    {RET_CHSEND, "+CCHSEND: "},
    {RET_NTP, "+CNTP: "},
    {RET_CLK, "+CCLK: "},
    {RET_CHSET, "+CCHSET: "},
    {RET_CHSSLCFG, "+CCHSSLCFG: "},
    {RET_CHRECVDATA, "+CCHRECV: DATA"},
    {RET_CHCLOSE, "+CCHCLOSE: "},
    {RET_CHOPEN_CONNECT, "CONNECT "},
    {RET_CRLF, "\r\n"}, /* keep RET_CRLF last !!! */
};

// simcom status
static bool _simcom_alive = false;

// mutex
static SemaphoreHandle_t simcom_semaphore = NULL; // for locking while requesting from modem

/*********************************************** Internal Declarations ************************************************/
// Task related
static void simcom_handler_task(void *pvParameters);

// UART related
// Default baudrate will be 115200 bps
static int simcom_uart_init(void);
// Handlers to send and receive data
static int simcom_send_data(const char *data);
static int simcom_rcv_data(char *buffer, int buffer_len, int scan_resp, int timeout);
// GPIO related
static int simcom_gpio_init(void);

// SIMCOM related
// Configure modem
static int simcom_configure(void);
// Commands
static int simcom_set_cfun(SIMCOM_CFUN_Values_t value);
static int simcom_query_signal(int *rssi);
static int simcom_set_apn(int context, char *apn);
static int simcom_activate_pdp_context(int context);
static int simcom_query_creg(int *stat);
static int simcom_query_cgreg(int *stat);
static int simcom_query_cereg(int *stat);
static int simcom_add_certfile(char *filename, const char *cert);
static int simcom_delete_certfile(char *filename);
static int simcom_sync_ntp(char *ntp_server);
static int simcom_change_echo(bool enable);
static int simcom_restore_factory_settings(void);
static int simcom_get_clock(time_t *t_of_day);
static int simcom_configure_ssl(char *parameter, int ssl_ctx_index, char *value);
static int simcom_configure_report_mode(bool enabled);
static int simcom_start_ssl_service(void);
static int simcom_set_ssl_context(int session_id, int ssl_ctx_index);
static int simcom_ssl_connect_to_server(int session_id, char *host, int port, int client_type);
static int simcom_ssl_send_data_to_server(int session_id, SIMCOM_SSL_Request_t *ssl_request);
static int simcom_ssl_recv_data_from_server(int session_id, SIMCOM_SSL_Response_t *response);
static int simcom_disconnect_from_server(int session_id);
static int simcom_stop_ssl_service(void);
static int simcom_configure_ssl_transparent_mode(bool enabled);

// Checkers
static bool simcom_sim_card_ready(void);
static bool simcom_check_certfile(char *filename);
// Parsers
static int parseInt(char *buffer, char terminator);
// Mutex related
static int simcom_mutex_take(int ms_to_wait);
static int simcom_mutex_give(void);
/***************************************************** Functions ******************************************************/

int simcom_init(void)
{
    // Creates the task.
    BaseType_t xReturned = xTaskCreate(
        simcom_handler_task,
        COMPONENT_NAME_SIMCOM,
        TASK_STACK_SIZE_SIMCOM,
        NULL,
        TASK_PRIORITY_SIMCOM,
        NULL);

    // In case the task creation fails.
    if (xReturned != pdPASS)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

bool simcom_alive(void)
{
    return _simcom_alive;
}

int simcom_ssl_call(SIMCOM_SSL_Request_t *ssl_request, SIMCOM_SSL_Response_t *response)
{
    esp_err_t retval = ESP_OK;

    // Check cert file and add if needed
    if (!ssl_request->force_insecure && (ssl_request->force_write_cert || !simcom_check_certfile(ssl_request->root_cert_filename)))
    {
        if (ssl_request->force_write_cert)
        {
            if (simcom_delete_certfile(ssl_request->root_cert_filename) != ESP_OK)
            {
                ESP_LOGE(TAG, "Error while trying to delete certfile!");
                return ESP_FAIL;
            }
        }

        if (simcom_add_certfile(ssl_request->root_cert_filename, ssl_request->root_cert) != ESP_OK)
        {
            ESP_LOGE(TAG, "Error while trying to add certfile!");
            return ESP_FAIL;
        }
    }

    // Set the SSL version of the first SSL context
    if (simcom_configure_ssl("sslversion", SIMCOM_DEFAULT_SSL_CONTEXT, "4") != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL sslversion configure Failed!");
        return ESP_FAIL;
    }

    // Set the authentication mode
    // 0 - insecure (don't verify the server)
    // 1 - secure (verify server)
    if (simcom_configure_ssl("authmode", SIMCOM_DEFAULT_SSL_CONTEXT, ssl_request->force_insecure ? "0" : "1") != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL authmode configure Failed!");
        return ESP_FAIL;
    }

    // Set the server root CA of the first SSL context
    if (!ssl_request->force_insecure)
    {
        if (simcom_configure_ssl("cacert", SIMCOM_DEFAULT_SSL_CONTEXT, ssl_request->root_cert_filename) != ESP_OK)
        {
            ESP_LOGE(TAG, "SSL cacert configure Failed!");
            return ESP_FAIL;
        }
    }

    // Enable server name identification
    if (simcom_configure_ssl("enableSNI", SIMCOM_DEFAULT_SSL_CONTEXT, "1") != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL enable SNI configure Failed!");
        return ESP_FAIL;
    }

    // Enable reporting +CCHSEND result
    if (simcom_configure_report_mode(true) != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL Report configure Failed!");
        return ESP_FAIL;
    }

    // Disable transparent mode for SSL
    // Transparent mode is not supported
    if (simcom_configure_ssl_transparent_mode(false) != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL Transparent mode Failed!");
        return ESP_FAIL;
    }

    // Start SSL service, activate PDP context
    if (simcom_start_ssl_service() != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL Start Failed!");
        return ESP_FAIL;
    }

    // Set the first SSL context to be used in the SSL connection
    if (simcom_set_ssl_context(SIMCOM_DEFAULT_SSL_SESSION_ID, SIMCOM_DEFAULT_SSL_CONTEXT) != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL Set Context Failed!");
        retval = ESP_FAIL;
        goto close_ssl_and_leave;
    }

    // connect to SSL/TLS server
    if (simcom_ssl_connect_to_server(SIMCOM_DEFAULT_SSL_SESSION_ID, ssl_request->host_server, 443, 2) != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL connect to server Failed!");
        retval = ESP_FAIL;
        goto close_ssl_and_leave;
    }

    // send data to server
    if (simcom_ssl_send_data_to_server(SIMCOM_DEFAULT_SSL_SESSION_ID, ssl_request) != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL send data to server Failed!");
        retval = ESP_FAIL;
        goto close_connection_and_leave;
    }

    // receive response from server
    if (simcom_ssl_recv_data_from_server(SIMCOM_DEFAULT_SSL_SESSION_ID, response) != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL receive data from server Failed!");
        retval = ESP_FAIL;
        goto close_connection_and_leave;
    }

close_connection_and_leave:
    // Disconnect from the Service
    if (simcom_disconnect_from_server(SIMCOM_DEFAULT_SSL_SESSION_ID) != ESP_OK)
    {
        ESP_LOGI(TAG, "SSL disconnected from service already.");
    }

close_ssl_and_leave:
    // stop SSL Service
    if (simcom_stop_ssl_service() != ESP_OK)
    {
        ESP_LOGE(TAG, "SSL stop Failed!");
        retval = ESP_FAIL;
    }

    return retval;
}

/************************************************* Internal Functions *************************************************/
static void simcom_handler_task(void *pvParameters)
{
#ifdef DEBUG_STACK_SIZE
    int uxHighWaterMark_counter = 0;
    UBaseType_t uxHighWaterMark;

    /* Inspect our own high water mark on entering the task. */
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGW(TAG, "pre-exec uxHighWaterMark: %d \r\n", uxHighWaterMark);
#endif

    // Initializes 'last_wake_time' with the current time.
    TickType_t last_wake_time;

    // Variables
    bool gpio_inited = false;
    bool uart_inited = false;

    for (;;)
    {
        // update last_wake_time
        last_wake_time = xTaskGetTickCount();

        if (!gpio_inited)
        {
            if (simcom_gpio_init() == ESP_OK)
            {
                gpio_inited = true;
            }
            else
            {
                ESP_LOGE(TAG, "Failed to init GPIO!");
            }
        }

        // init uart if not already
        if (gpio_inited && !uart_inited)
        {
            if (simcom_uart_init() == ESP_OK)
            {
                uart_inited = true;
            }
            else
            {
                ESP_LOGE(TAG, "UART init failed!");
            }
        }

        // configure modem if not configured yet
        if (uart_inited && !_simcom_alive)
        {
            if (simcom_configure() == ESP_OK)
            {
                _simcom_alive = true;
            }
            else
            {
                ESP_LOGE(TAG, "SIMCOM configure failed!");
            }
        }

        // TODO: Periodic calls to verify if modem is still alive

        // Waits for the end of the current time period. This considers the code execution time
        // since *last_wake_time* and waits until time reaches the TASK_PERIOD_SIMCOM ms.
        vTaskDelayUntil(&last_wake_time, TASK_PERIOD_SIMCOM / portTICK_PERIOD_MS);

#ifdef DEBUG_STACK_SIZE
        if (uxHighWaterMark_counter++ > (DEBUG_STACK_SIZE_PERIODICITY / TASK_PERIOD_PUBLISHER))
        {
            uxHighWaterMark_counter = 0;
            /* Calling the function will have used some stack space, we would
                therefore now expect uxTaskGetStackHighWaterMark() to return a
                value lower than when it was called on entering the task. */
            uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGW(TAG, "exec uxHighWaterMark: %d \r\n", uxHighWaterMark);
        }
#endif
    }

    vTaskDelete(NULL);
}

static int simcom_gpio_init(void)
{
    esp_err_t err;

    //++ Set GPIO Pin Directions
    err = gpio_set_direction(SIMCOM_PWKEY_IO, GPIO_MODE_OUTPUT);

    // TODO: Add here everything else that you need from the modem

    return err;
}

static int simcom_uart_init(void)
{
    esp_err_t retval = ESP_OK;

    const uart_config_t uart_config =
        {
            .baud_rate = SIMCOM_DEFAULT_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

    // We won't use a buffer for sending data.
    retval = uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (retval != ESP_OK)
    {
        return retval;
    }

    // Apply UART configurations
    retval = uart_param_config(UART_NUM_1, &uart_config);
    if (retval != ESP_OK)
    {
        return retval;
    }

    // Set UART pins
    retval = uart_set_pin(UART_NUM_1, SIMCOM_TTL_TX_IO, SIMCOM_TTL_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (retval != ESP_OK)
    {
        return retval;
    }

    return ESP_OK;
}

static int simcom_send_data(const char *data)
{
    // FLUSH RX RING BUFFER
    uart_flush(UART_NUM_1);

    const int len = strlen(data);
    const int retval = uart_write_bytes(UART_NUM_1, data, len);
    if (retval != len)
    {
        return ESP_FAIL;
    }

    // wait until tx done
    if (uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(UART_TX_READY_TIMEOUT) == ESP_OK))
    {
        ESP_LOGD(TAG, "Transmitted: %s", data);
        return ESP_OK;
    }

    return ESP_FAIL;
}

static int simcom_rcv_data(char *buffer, int buffer_len, int scan_resp, int timeout)
{
    TickType_t tickstart = xTaskGetTickCount();
    int16_t ReadData = 0;
    uint16_t x;
    static bool firstTime = true;
    uint16_t index[NUM_RESPONSES];
    static uint16_t lens[NUM_RESPONSES];
    uint16_t pos;
    uint8_t c;

    // generate lens and index arrays only once
    if (firstTime)
    {
        firstTime = false;
        for (x = 0; x < NUM_RESPONSES; x++)
        {
            lens[x] = strlen(ReturnKeywords[x].retstr);
        }
    }

    // reset index
    memset(index, 0, sizeof(index));

    // reset buffer
    buffer[0] = '\0';

    if (scan_resp == RET_NONE)
    {
        return RET_NONE; /* to avoid waiting a RET_VAL in case the parsed_lenth of payload is zero */
                         /* but no code needs to be retrieved */
    }

    while ((xTaskGetTickCount() - tickstart) <= pdMS_TO_TICKS(timeout))
    {

        // try to receive every 1 ms
        if (uart_read_bytes(UART_NUM_1, (char *)&c, 1, 10 / portTICK_PERIOD_MS))
        {
            // add to the buffer
            if (ReadData < buffer_len)
            {
                buffer[ReadData++] = c;
                buffer[ReadData] = '\0';
            }
            else
            {
                // buffer overflow
                goto buffer_overflow;
            }

            /* Check whether we hit an ESP return values */
            for (x = 0; x < NUM_RESPONSES; x++)
            {
                if (c != ReturnKeywords[x].retstr[index[x]])
                {
                    index[x] = 0;
                }

                if (c == ReturnKeywords[x].retstr[index[x]])
                {
                    pos = ++(index[x]);
                    if (pos >= lens[x])
                    {
                        if (scan_resp & ReturnKeywords[x].retval)
                        {
                            ESP_LOGD(TAG, "Data: %.*s Len: %d\r\n", ReadData, (char *)buffer, ReadData);
                            // for (int k= 0; k < ReadData; k++) {
                            //     printf(" %u ",buffer[k]);
                            // }
                            // printf("\r\n");
                            return ReturnKeywords[x].retval;
                        }
                    }
                }
            }
        }
    }

    ESP_LOGE(TAG, "Timeout while waiting for respose from UART!");
    return ESP_FAIL;

buffer_overflow:
    ESP_LOGE(TAG, "Buffer overflow while trying to receive from UART!");
    return ESP_FAIL;
}

static int simcom_configure(void)
{
    gpio_set_level(SIMCOM_PWKEY_IO, 1); //++ Restarting Simcom via ENABLE pin
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(SIMCOM_PWKEY_IO, 0);
    vTaskDelay(5000 / portTICK_PERIOD_MS); // wait modem power up

    // check if the sim is ready
    bool sim_ready = false;
    int attempts = SIMCOM_WAIT_SIM_READY_ATTEMPTS;
    do
    {
        sim_ready = simcom_sim_card_ready();
        if (sim_ready)
        {
            break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // check again in 1 sec
    } while (!sim_ready && (attempts-- > 0));

    if (!sim_ready)
    {
        ESP_LOGE(TAG, "Sim not ready! Please check and restart!");
        return ESP_FAIL;
    }

    // restore factory settings
    if (simcom_restore_factory_settings() != ESP_OK)
    {
        ESP_LOGE(TAG, "Factory settings command failed!");
        return ESP_FAIL;
    }

    // disable echo
    if (simcom_change_echo(false) != ESP_OK)
    {
        ESP_LOGE(TAG, "Disabling echo command failed!");
        return ESP_FAIL;
    }

    // set function quality to max value
    if (simcom_set_cfun(CFUN_FULL_FUNCTIONALITY) != ESP_OK)
    {
        ESP_LOGE(TAG, "Set phone functionality command failed!");
        return ESP_FAIL;
    }

    // check rssi
    int rssi;
    attempts = SIMCOM_GET_RSSI_ATTEMPTS;
    do
    {
        if (simcom_query_signal(&rssi) != ESP_OK)
        {
            return ESP_FAIL;
        }
        if (rssi != 99)
        {
            break;
        }
        // try again in 1 second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } while ((rssi == 99) && (attempts-- > 0));
    ESP_LOGI(TAG, "Got RSSI: %d.", rssi);

    // query cs service
    int stat = 0;
    for (int i = 0; i < SIMCOM_WAIT_CELLULAR_CONNECTION_ATTEMPTS; i++)
    {
        if (simcom_query_creg(&stat) != ESP_OK)
        {
            ESP_LOGE(TAG, "Query CREG command failed!");
            return ESP_FAIL;
        }
        if (stat == 1 || stat == 5)
        {
            break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // query again in 1 sec
    }
    if (!(stat == 1 || stat == 5))
    {
        ESP_LOGE(TAG, "CREG activation failed!");
        return ESP_FAIL;
    }

    // query ps service
    for (int i = 0; i < SIMCOM_WAIT_CELLULAR_CONNECTION_ATTEMPTS; i++)
    {
        if (simcom_query_cgreg(&stat) != ESP_OK)
        {
            ESP_LOGE(TAG, "Query CGREG command failed!");
            return ESP_FAIL;
        }
        if (stat == 1 || stat == 5)
        {
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (simcom_query_cereg(&stat) != ESP_OK)
        {
            ESP_LOGE(TAG, "Query CEREG command failed!");
            return ESP_FAIL;
        }
        if (stat == 1 || stat == 5)
        {
            break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // query again in 1 sec
    }
    if (!(stat == 1 || stat == 5))
    {
        ESP_LOGE(TAG, "CGREG/CEREG activation failed!");
        return ESP_FAIL;
    }

    // set apn configs
    if (simcom_set_apn(SIMCOM_DEFAULT_CONTEXT, SIMCOM_SIM_CARD_APN) != ESP_OK)
    {
        ESP_LOGE(TAG, "Set APN command failed!");
        return ESP_FAIL;
    }

    // activate pdp
    if (simcom_activate_pdp_context(SIMCOM_DEFAULT_CONTEXT) != ESP_OK)
    {
        ESP_LOGE(TAG, "Activate PDP context command failed!");
        return ESP_FAIL;
    }

    // sync ntp
    if (simcom_sync_ntp(SIMCOM_NTP_SYNC_SERVER) != ESP_OK)
    {
        ESP_LOGE(TAG, "NTP server sync failed!");
        return ESP_FAIL;
    }

    // get clock and sync esp32 time
    time_t t_of_day;
    if (simcom_get_clock(&t_of_day) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error while retrieving Clock time!");
        return ESP_FAIL;
    }
    else
    {
        struct timeval tv;
        tv.tv_sec = t_of_day;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);

        ESP_LOGI(TAG, "Seconds since the Epoch: %ld", t_of_day);
    }

    return ESP_OK;
}

static int simcom_change_echo(bool enable)
{
    char command[50];
    esp_err_t retval = ESP_FAIL;
    snprintf(command, sizeof(command), "ATE%u\r\n", enable);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        if (simcom_send_data(command) == ESP_OK)
        {
            // receive confirmation
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_120s) == RET_OK)
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_restore_factory_settings(void)
{
    esp_err_t retval = ESP_FAIL;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // 	Restore factory settings
        if (simcom_send_data("AT&F\r\n") == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_9s) == RET_OK)
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static bool simcom_sim_card_ready(void)
{
    bool retval = false;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CPIN
        if (simcom_send_data("AT+CPIN?\r\n") == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_SIM_READY | RET_ERROR, SIMCOM_Response_Time_9s) == RET_SIM_READY)
            {
                retval = true;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_set_cfun(SIMCOM_CFUN_Values_t value)
{
    char command[50];
    esp_err_t retval = ESP_FAIL;
    snprintf(command, sizeof(command), "AT+CFUN=%u\r\n", value);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        if (simcom_send_data(command) == ESP_OK)
        {
            // receive confirmation
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_9s) == RET_OK)
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_query_signal(int *rssi)
{
    esp_err_t retval = ESP_FAIL;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CSQ
        if (simcom_send_data("AT+CSQ\r\n") == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_RSSI_SIGNAL | RET_ERROR, SIMCOM_Response_Time_9s) == RET_RSSI_SIGNAL)
            {
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    *rssi = parseInt(_buffer, ',');
                    retval = ESP_OK;
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_set_apn(int context, char *apn)
{
    char command[sizeof("AT+CGDCONT=99,\"IP\",\"\"\r\n") + SIMCOM_SIM_CARD_APN_MAX_SIZE + 1];
    esp_err_t retval = ESP_FAIL;
    snprintf(command, sizeof(command), "AT+CGDCONT=%u,\"IP\",\"%s\"\r\n", context, apn);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CGDCONT
        if (simcom_send_data(command) == ESP_OK)
        {
            // receive confirmation
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_9s) == RET_OK)
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_activate_pdp_context(int context)
{
    char command[50];
    esp_err_t retval = ESP_FAIL;
    snprintf(command, sizeof(command), "AT+CGACT=1,%u\r\n", context);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CGACT
        if (simcom_send_data(command) == ESP_OK)
        {
            // receive confirmation
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR | RET_CME_ERROR, SIMCOM_Response_Time_150s) == RET_OK)
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_query_creg(int *stat)
{
    esp_err_t retval = ESP_FAIL;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CREG?
        if (simcom_send_data("AT+CREG?\r\n") == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CREG | RET_ERROR, SIMCOM_Response_Time_9s) == RET_CREG)
            {
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    char *ptr = _buffer;
                    // jump to <stat>
                    while (*ptr != ',')
                        ptr++;
                    // parse it
                    ptr++;
                    *stat = parseInt(ptr, ',');
                    retval = ESP_OK;
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_query_cgreg(int *stat)
{
    esp_err_t retval = ESP_FAIL;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CGREG?
        if (simcom_send_data("AT+CGREG?\r\n") == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CGREG | RET_ERROR, SIMCOM_Response_Time_9s) == RET_CGREG)
            {
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    char *ptr = _buffer;
                    // jump to <stat>
                    while (*ptr != ',')
                        ptr++;
                    // parse it
                    ptr++;
                    *stat = parseInt(ptr, ',');
                    retval = ESP_OK;
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_query_cereg(int *stat)
{
    esp_err_t retval = ESP_FAIL;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CEREG?
        if (simcom_send_data("AT+CEREG?\r\n") == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CEREG | RET_ERROR, SIMCOM_Response_Time_9s) == RET_CEREG)
            {
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    char *ptr = _buffer;
                    // jump to <stat>
                    while (*ptr != ',')
                        ptr++;
                    // parse it
                    ptr++;
                    *stat = parseInt(ptr, ',');
                    retval = ESP_OK;
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_sync_ntp(char *ntp_server)
{
    esp_err_t retval = ESP_FAIL;
    // Set the server for syncing
    char cmd[sizeof("AT+CNTP=\"") + SIMCOM_NTP_NAMESPACE_MAX_SIZE + sizeof("\",0\r\n") + 1] = {'\0'};
    snprintf(cmd, sizeof(cmd), "AT+CNTP=\"%s\",0\r\n", ntp_server);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        if (simcom_send_data(cmd) == ESP_OK)
        {
            // receive confirmation
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_15s) == RET_OK)
            {
                // Sync NTP
                if (simcom_send_data("AT+CNTP\r\n") == ESP_OK)
                {
                    // receive confirmation
                    if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_NTP | RET_ERROR, SIMCOM_Response_Time_15s) == RET_NTP)
                    {
                        // get return code
                        if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                        {
                            char *ptr = _buffer;
                            // operation succeed
                            if (parseInt(ptr, '\r') == 0)
                            {
                                retval = ESP_OK;
                            }
                        }
                    }
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_get_clock(time_t *t_of_day)
{
    esp_err_t retval = ESP_FAIL;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCLK?
        if (simcom_send_data("AT+CCLK?\r\n") == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CLK | RET_ERROR, SIMCOM_Response_Time_9s) == RET_CLK)
            {
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    // 23/07/28,10:21:07+00
                    // yy/MM/dd,hh:mm:ss±zz
                    struct tm t;
                    int timezone;
                    char *ptr = _buffer;
                    ptr++;
                    t.tm_year = 2000 + parseInt(ptr, '/') - 1900;
                    ptr += 3;
                    t.tm_mon = parseInt(ptr, '/') - 1;
                    ptr += 3;
                    t.tm_mday = parseInt(ptr, ',');
                    ptr += 3;
                    t.tm_hour = parseInt(ptr, ':');
                    ptr += 3;
                    t.tm_min = parseInt(ptr, ':');
                    ptr += 3;
                    t.tm_sec = parseInt(ptr, '+');
                    ptr += 3;
                    timezone = parseInt(ptr, '"');
                    t.tm_isdst = -1;
                    *t_of_day = mktime(&t);
                    ESP_LOGD(TAG, "%02d/%02d/%02d %02d:%02d:%02d TZ:%02d", t.tm_mday, t.tm_mon + 1, t.tm_year + 1900, t.tm_hour, t.tm_min, t.tm_sec, timezone);
                    retval = ESP_OK;
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static bool simcom_check_certfile(char *filename)
{
    bool result = false;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCERTLIST
        if (simcom_send_data("AT+CCERTLIST\r\n") == ESP_OK)
        {
            int retval = RET_NONE;
            do
            {
                retval = simcom_rcv_data(_buffer, sizeof(_buffer), RET_CERTLIST | RET_OK | RET_ERROR, SIMCOM_Response_Time_120s);
                if ((retval == RET_CERTLIST) && simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    if (strstr(_buffer, filename) != NULL)
                    {
                        result = true;
                    }
                }
            } while (retval != ESP_FAIL && retval != RET_OK && retval != RET_ERROR);
        }
    }

    // release sempahore
    simcom_mutex_give();

    return result;
}

static int simcom_add_certfile(char *filename, const char *cert)
{
    esp_err_t retval = ESP_FAIL;
    char command[sizeof("AT+CCERTDOWN=\r\n") + SIMCOM_CERTFILE_NAME_MAX_SIZE + 1];
    snprintf(command, sizeof(command), "AT+CCERTDOWN=%s,%u\r\n", filename, strlen(cert));

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCERTDOWN
        if (simcom_send_data(command) == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_ARROW | RET_ERROR, SIMCOM_Response_Time_9s) == RET_ARROW)
            {
                if (simcom_send_data(cert) == ESP_OK)
                {
                    if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_120s) == RET_OK)
                    {
                        retval = ESP_OK;
                    }
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_delete_certfile(char *filename)
{
    esp_err_t retval = ESP_FAIL;
    char command[sizeof("AT+CCERTDELE=\r\n") + SIMCOM_CERTFILE_NAME_MAX_SIZE + 1];
    snprintf(command, sizeof(command), "AT+CCERTDELE=%s\r\n", filename);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        if (simcom_send_data(command) == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_120s) == RET_OK)
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_configure_ssl(char *parameter, int ssl_ctx_index, char *value)
{
    // size based on max command lenght
    char command[sizeof("AT+CSSLCFG=\"ignorelocaltime\",99,\"\"\r\n") + SIMCOM_CERTFILE_NAME_MAX_SIZE + 1];
    esp_err_t retval = ESP_FAIL;
    snprintf(command, sizeof(command), "AT+CSSLCFG=\"%s\",%u,%s\r\n", parameter, ssl_ctx_index, value);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CSSLCFG
        if (simcom_send_data(command) == ESP_OK)
        {
            // receive confirmation
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_9s) == RET_OK)
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_configure_ssl_transparent_mode(bool enabled)
{
    esp_err_t retval = ESP_FAIL;
    char command[50];

    // set the command
    if (enabled)
    {
        snprintf(command, sizeof(command), "AT+CCHMODE=1\r\n");
    }
    else
    {
        snprintf(command, sizeof(command), "AT+CCHMODE=0\r\n");
    }

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCHMODE
        if (simcom_send_data(command) == ESP_OK)
        {
            // receive confirmation
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_9s) == RET_OK)
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_configure_report_mode(bool enabled)
{
    esp_err_t retval = ESP_FAIL;
    int report_send_result = -1;
    char command[50];

    // set the command
    if (enabled)
    {
        snprintf(command, sizeof(command), "AT+CCHSET=1,0\r\n");
    }
    else
    {
        snprintf(command, sizeof(command), "AT+CCHSET=0,0\r\n");
    }

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCHSET?
        if (simcom_send_data("AT+CCHSET?\r\n") == ESP_OK)
        {
            // receive confirmation
            bool change_settings = false;
            int retKeyword = simcom_rcv_data(_buffer, sizeof(_buffer), RET_CHSET | RET_OK | RET_ERROR, SIMCOM_Response_Time_120s);

            // configuration set
            if (retKeyword == RET_CHSET)
            {
                // get current mode
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    char *ptr = _buffer;
                    report_send_result = parseInt(ptr, ',');

                    if (enabled && (report_send_result == 0))
                    {
                        change_settings = true;
                    }
                    else if (!enabled && (report_send_result == 1))
                    {
                        change_settings = true;
                    }
                }
            }

            // no configuration set
            if (retKeyword == RET_OK)
            {
                change_settings = true;
            }

            // apply new settings
            if (change_settings)
            {
                // send AT+CCHSET
                if (simcom_send_data(command) == ESP_OK)
                {
                    // receive confirmation
                    if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_9s) == RET_OK)
                    {
                        retval = ESP_OK;
                    }
                }
            }
            else
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_start_ssl_service(void)
{
    esp_err_t retval = ESP_FAIL;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCHSTART
        if (simcom_send_data("AT+CCHSTART\r\n") == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CHSTART | RET_ERROR, SIMCOM_Response_Time_120s) == RET_CHSTART)
            {
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    if (parseInt(_buffer, '\0') == 0)
                    {
                        retval = ESP_OK;
                    }
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_set_ssl_context(int session_id, int ssl_ctx_index)
{
    esp_err_t retval = ESP_FAIL;
    char command[50];
    snprintf(command, sizeof(command), "AT+CCHSSLCFG=%d,%d\r\n", session_id, ssl_ctx_index);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCHSSLCFG
        if (simcom_send_data(command) == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_OK | RET_ERROR, SIMCOM_Response_Time_120s) == RET_OK)
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_ssl_connect_to_server(int session_id, char *host, int port, int client_type)
{
    esp_err_t retval = ESP_FAIL;
    int resp = RET_NONE;
    char command[sizeof("AT+CCHOPEN=0,\"\",99999,99\r\n") + strlen(host) + 1];
    snprintf(command, sizeof(command), "AT+CCHOPEN=%d,\"%s\",%d,%d\r\n", session_id, host, port, client_type);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCHOPEN
        if (simcom_send_data(command) == ESP_OK)
        {
            resp = simcom_rcv_data(_buffer, sizeof(_buffer), RET_CHOPEN | RET_CHOPEN_CONNECT | RET_ERROR, SIMCOM_Response_Time_120s);
            // non-transparent mode
            if (resp == RET_CHOPEN)
            {
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    char *ptr = _buffer;
                    int res_session_id = parseInt(ptr, ',');
                    while (*ptr != ',')
                        ptr++;
                    int err = parseInt(ptr, '\0');

                    if (res_session_id == session_id && err == 0)
                    {
                        retval = ESP_OK;
                    }
                }
            }
            // transparent mode
            else if (resp == RET_CHOPEN_CONNECT)
            {
                retval = ESP_OK;
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_ssl_send_data_to_server(int session_id, SIMCOM_SSL_Request_t *ssl_request)
{
    esp_err_t retval = ESP_FAIL;
    char *data = NULL;
    char command[50];
    int array_size;
    int len;

    if (ssl_request->ssl_request_method == SIMCOM_SSL_GET)
    {
        array_size = sizeof("GET ") + strlen(ssl_request->endpoint) + sizeof(" HTTP/1.1\r\n") +
                     sizeof("Host: ") + strlen(ssl_request->host_server) + sizeof("\r\n") +
                     sizeof("Connection: close\r\n") +
                     sizeof("User-Agent: esp-idf/1.0 esp32\r\n") +
                     sizeof("\r\n") + 1;

        if (ssl_request->headers != NULL)
        {
            array_size += strlen(ssl_request->headers) + sizeof("\r\n");
        }

        data = pvPortMalloc(array_size);
        len = snprintf(
            data,
            array_size,
            "GET %s HTTP/1.1\r\n"
            "Host: %s \r\n"
            "%s%s"
            "Connection: close\r\n"
            "User-Agent: esp-idf/1.0 esp32\r\n"
            "\r\n",
            ssl_request->endpoint,
            ssl_request->host_server,
            ssl_request->headers != NULL ? ssl_request->headers : "",
            ssl_request->headers != NULL ? "\r\n" : "");
    }
    else if (ssl_request->ssl_request_method == SIMCOM_SSL_POST)
    {
        array_size = sizeof("POST ") + strlen(ssl_request->endpoint) + sizeof(" HTTP/1.1\r\n") +
                     sizeof("Host: ") + strlen(ssl_request->host_server) + sizeof("\r\n") +
                     sizeof("Connection: close\r\n") +
                     sizeof("User-Agent: esp-idf/1.0 esp32\r\n") +
                     sizeof("Content-Type: application/json\r\n") + strlen(ssl_request->payload) + sizeof("\r\n\r\n") +
                     sizeof("Content-Length: 99999999\r\n") + sizeof("\r\n") + 1;

        if (ssl_request->headers != NULL)
        {
            array_size += strlen(ssl_request->headers) + sizeof("\r\n");
        }

        data = pvPortMalloc(array_size);
        len = snprintf(
            data,
            array_size,
            "POST %s HTTP/1.1\r\n"
            "Host: %s \r\n"
            "%s%s"
            "User-Agent: esp-idf/1.0 esp32\r\n"
            "Connection: close\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %u\r\n"
            "\r\n"
            "%s\r\n"
            "\r\n",
            ssl_request->endpoint,
            ssl_request->host_server,
            ssl_request->headers != NULL ? ssl_request->headers : "",
            ssl_request->headers != NULL ? "\r\n" : "",
            strlen(ssl_request->payload),
            ssl_request->payload);
    }
    else
    {
        ESP_LOGE(TAG, "Unsupported method!");
        return ESP_FAIL;
    }

    // create command
    snprintf(command, sizeof(command), "AT+CCHSEND=%d,%d\r\n", session_id, len);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCHSEND
        if (simcom_send_data(command) == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_ARROW | RET_ERROR, SIMCOM_Response_Time_9s) == RET_ARROW)
            {
                if (simcom_send_data(data) == ESP_OK)
                {
                    if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CHSEND, SIMCOM_Response_Time_120s) == RET_CHSEND)
                    {
                        if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                        {
                            char *ptr = _buffer;
                            int res_session_id = parseInt(ptr, ',');
                            while (*ptr != ',')
                                ptr++;
                            int pending_bytes = parseInt(ptr, '\0');

                            if (res_session_id == session_id && pending_bytes == 0)
                            {
                                retval = ESP_OK;
                            }
                        }
                    }
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    if (data != NULL)
    {
        vPortFree(data);
    }

    return retval;
}

static int simcom_ssl_recv_data_from_server(int session_id, SIMCOM_SSL_Response_t *response)
{
    esp_err_t retval = ESP_FAIL;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        char *status_code = NULL;
        char *content_length = NULL;
        bool isDataSection = false;
        bool mallocFailed = false;
        int recv_data_length = 0;

        do
        {
            // discard '\r\n' of the last line
            // when payload is divided
            if (recv_data_length > 0 && response->data[recv_data_length - 2] == '\r' && response->data[recv_data_length - 1] == '\n')
            {
                recv_data_length -= 2;
            }

            // wait +CCHRECV:DATA indication
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CHRECVDATA | RET_ERROR, SIMCOM_Response_Time_120s) == RET_CHRECVDATA)
            {
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    char *ptr = _buffer;
                    ptr++;
                    int res_session_id = parseInt(ptr, ',');

                    // number of bytes to read
                    while (*ptr != ',')
                        ptr++;
                    int bytes_to_read = parseInt(ptr, '\0');

                    // ESP_LOGI(TAG, "Session: %u with %u to receive.", res_session_id, bytes_to_read);

                    if (res_session_id == session_id)
                    {
                        int recv_bytes = 0;

                        while (recv_bytes < bytes_to_read)
                        {
                            bool err = simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_120s) == ESP_FAIL;
                            // ESP_LOGI(TAG, "Err: %u Bytes to copy = %u", err, strlen(_buffer));

                            // increment
                            recv_bytes += strlen(_buffer);

                            if (!isDataSection)
                            {
                                // get status code
                                if ((status_code = strstr(_buffer, "HTTP/1.1")) != NULL)
                                {
                                    status_code += strlen("HTTP/1.1") + 1;
                                    response->status_code = parseInt(status_code, ' ');
                                    ESP_LOGD(TAG, "Recv status code: %u", response->status_code);
                                }

                                // get content length
                                else if ((content_length = strstr(_buffer, "Content-Length:")) != NULL ||
                                         (content_length = strstr(_buffer, "content-length:")) != NULL)
                                {
                                    content_length += strlen("Content-Length:") + 1;
                                    response->data_length = parseInt(content_length, '\0');
                                    ESP_LOGD(TAG, "Recv content length: %u", response->data_length);
                                    response->data = pvPortMalloc(response->data_length + 1);
                                    if (response->data == NULL)
                                    {
                                        ESP_LOGE(TAG, "Malloc failed!");
                                        mallocFailed = true;
                                        break;
                                    }
                                }

                                // get data section
                                else if (strcmp(_buffer, "\r\n") == 0)
                                {
                                    isDataSection = true;
                                    continue; // move to data section
                                }
                            }

                            else
                            {
                                int bytes_to_copy = strlen(_buffer);

                                if (recv_data_length + bytes_to_copy <= response->data_length)
                                {
                                    memcpy(&response->data[recv_data_length], _buffer, bytes_to_copy);
                                    recv_data_length += bytes_to_copy;
                                }
                                else
                                {
                                    memcpy(&response->data[recv_data_length], _buffer, response->data_length - recv_data_length);
                                    ESP_LOGI(TAG, "Truncating from: %u to: %u.", bytes_to_copy, response->data_length - recv_data_length);
                                    recv_data_length += (response->data_length - recv_data_length);
                                }
                            }

                            // nothing else to receive
                            if (err)
                            {
                                break;
                            }
                        }
                    }
                }
            }
        } while (!mallocFailed && recv_data_length < response->data_length);

        // Nothing received ?
        if (recv_data_length > 0)
        {
            retval = ESP_OK;
        }

        // update length
        response->data_length = recv_data_length;
        response->data[recv_data_length] = '\0';
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_disconnect_from_server(int session_id)
{
    esp_err_t retval = ESP_FAIL;
    char command[50];
    snprintf(command, sizeof(command), "AT+CCHCLOSE=%d\r\n", session_id);

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCHCLOSE
        if (simcom_send_data(command) == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CHCLOSE | RET_ERROR, SIMCOM_Response_Time_120s) == RET_CHCLOSE)
            {
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    char *ptr = _buffer;
                    int res_session_id = parseInt(ptr, ',');
                    while (*ptr != ',')
                        ptr++;
                    int err = parseInt(ptr, '\0');

                    if (res_session_id == session_id && err == 0)
                    {
                        retval = ESP_OK;
                    }
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int simcom_stop_ssl_service(void)
{
    esp_err_t retval = ESP_FAIL;

    // try to acquire the semaphore
    if (simcom_mutex_take(SIMCOM_MUTEX_LOCK_TIMEOUT) == ESP_OK)
    {
        // send AT+CCHSTOP
        if (simcom_send_data("AT+CCHSTOP\r\n") == ESP_OK)
        {
            if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CHSTOP | RET_ERROR, SIMCOM_Response_Time_120s) == RET_CHSTOP)
            {
                if (simcom_rcv_data(_buffer, sizeof(_buffer), RET_CRLF, SIMCOM_Response_Time_Short) == RET_CRLF)
                {
                    if (parseInt(_buffer, '\0') == 0)
                    {
                        retval = ESP_OK;
                    }
                }
            }
        }
    }

    // release sempahore
    simcom_mutex_give();

    return retval;
}

static int parseInt(char *buffer, char terminator)
{
    int i, res = 0;
    for (i = 0; (buffer[i] != '\0') && (buffer[i] != terminator); i++)
    {
        if (buffer[i] >= '0' && buffer[i] <= '9')
        {
            res = res * 10 + buffer[i] - '0';
        }
    }
    return res;
}

static int simcom_mutex_take(int ms_to_wait)
{
    if (simcom_semaphore == NULL)
    {
        simcom_semaphore = xSemaphoreCreateMutex();
        if (simcom_semaphore == NULL)
        {
            ESP_LOGE(TAG, "Error while trying to create semaphore!");
            return ESP_FAIL;
        }
    }

    if (xSemaphoreTake(simcom_semaphore, pdMS_TO_TICKS(ms_to_wait)) == pdTRUE)
    {
        return ESP_OK;
    }

    return ESP_FAIL;
}

static int simcom_mutex_give(void)
{
    if (simcom_semaphore == NULL)
    {
        return ESP_FAIL;
    }

    // Release safely now
    if (xSemaphoreGive(simcom_semaphore) == pdTRUE)
    {
        return ESP_OK;
    }

    return ESP_FAIL;
}

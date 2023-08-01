# SIMCOM A76xxx ESP32 HTTPS/SSL/TLS Driver

![SimcomHardware](/images/simcom_breadboard.jpeg "A preview of the hardware setup.")

This repository contains a device driver for the SIMCOM module. The driver is located under `components/_driver_/simcom` folder. There is a small example for both GET and POST methods. The example is using the [PostData API](https://postdata.cloud/) which is a very simple and intuitive way to publish and visualize sensor data.

## Using the driver
Attach the modem to the ESP32 and change the `components/_config_/include/board_config.h` to match your hardware setup pins.
Replace or add any necessary root certificate for your application.

## Request options
```
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
```

| Option               | Description
|--------------------- |-----------
| `root_cert_filename` | Name of the certificate. <br>**Example:** `postData.pem`
| `root_cert`          | Pointer to the certificate.
| `force_write_cert`   | Force a write even if the cert file already exists in memory of the simcom.
| `force_insecure`     | Force the connection to be insecure (it will not verify the server).
| `ssl_request_method` | Request method. It can be POST or GET.
| `host_server`        | Host server. <br>**Example:** `api.postdata.cloud`
| `endpoint`           | Endpoint. <br>**Example:** `/add/simcom`
| `payload`            | Payload to post. <br>**Example:** `{"temperature":22.5}`
| `headers`            | Extra required headers. <br>**Example:** `{"authorization":token}`

## How to build PlatformIO based project

1. [Install PlatformIO Core](http://docs.platformio.org/page/core.html)
2. Run these commands:

```shell
# Change directory to example
$ cd a7672e_4g_modem_esp32

# Build project
$ pio run

# Upload firmware to specific port
$ pio run --target upload --upload-port /dev/ttyUSB0

# Build specific environment
$ pio run -e esp32dev

# Upload firmware for the specific environment
$ pio run -e esp32dev --target upload

# Clean build files
$ pio run --target clean
```
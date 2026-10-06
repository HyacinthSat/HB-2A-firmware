#ifndef HB_ESP_LOG_H
#define HB_ESP_LOG_H
/* The application reports sensor status; no blocking logging in the driver. */
#define ESP_LOGE(tag, ...) ((void)(tag))
#endif

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 1: pure test, synthetic position, no GNSS config/wait/forwarding.
   Set to 0 for flight operation with verified GNSS config and 3D/UTC lock. */
#define APP_INDOOR_TEST_MODE          0U
/* One startup-only pin test after radio init failure; adds 4 s, no SPI clocks. */
#define APP_SX1278_NSS_TEST_ON_FAILURE 1U
/* Temporary RF diagnosis: one bounded carrier before startup APRS, indoor only. */
#define APP_SX1278_CARRIER_TEST_MS     0U
/* Images are captured from the 06-minute APRS boundary onward. */
#define APP_STARTUP_SSDV_TEST           0U

#define APP_WSPR_CALL                 "BG7ZDQ"
#define APP_WSPR_DEFAULT_GRID         "RR73" /* Explicit no-fix fallback. */
#define APP_WSPR_ID13                 "03"
#define APP_WSPR_CENTER_HZ            14097140UL
#define APP_WSPR_REGULAR_MINUTE       2U
#define APP_WSPR_TELEMETRY_MINUTE     4U
#define APP_WSPR_DEFAULT_POWER_DBM    10U
#define APP_WSPR_SYMBOL_COUNT         162U
#define APP_WSPR_FRAME_DURATION_MS   110592UL

/* Centre of the OM68FA subsquare; consistent with the WSPR test locator. */
#define APP_TEST_GRID56               "FA"
#define APP_TEST_LAT_E7               380208333L
#define APP_TEST_LON_E7               1124583333L
#define APP_TEST_ALTITUDE_M           850U
#define APP_TEST_SPEED_KNOTS          0U

/* Start-to-start interval: 110.592 s on air, approximately 9.408 s RF off.
   Regular first, then telemetry, repeating independently of UTC indoors. */
#define APP_INDOOR_WSPR_PERIOD_MS     120000UL
/* Due APRS bursts wait for the RF-off gap; missed periods are not replayed. */
#define APP_INDOOR_APRS_PERIOD_MS     30000UL
#define APP_APRS_REPEAT_COUNT         2U
#define APP_APRS_REPEAT_GAP_MS        3000UL
/* Actual encoded length is checked against the remaining time before TX. */
#define APP_APRS_SLOT_GUARD_MS        300UL
#define APP_APRS_POWER_DBM            10U /* PA_BOOST setting, not a measured power */
#if APP_APRS_POWER_DBM < 2U || APP_APRS_POWER_DBM > 17U
#error "Normal PA_BOOST supports 2..17 dBm"
#endif

#define APP_APRS_CALL                 "BG7ZDQ-11"
#define APP_APRS_PATH                 "APZDQ7"
#define APP_APRS_DEFAULT_FREQ_HZ      144640000UL
/* Fixed experimental GMSK channel, separate from regional APRS selection. */
#define APP_GMSK_DOWNLINK_FREQ_HZ     144925000UL
#define APP_GMSK_POWER_DBM            10U
#define APP_GMSK_TELEMETRY_PERIOD_MS  60000UL
/* Packet FIFO RX does not depend on the continuous-mode DCLK output.
   Set to 0 only when comparing the original DIO1/DIO2 bit receiver. */
#ifndef APP_UPLINK_RX_FIFO
#define APP_UPLINK_RX_FIFO            1U
#endif

/* Image and telemetry packets use the same project destination. */
#define APP_SSDV_APRS_DEST            APP_APRS_PATH

/* Test startup: after GNSS lock, capture 256x192 CAM_A then CAM_B over APRS. */
/* Sensor-generated color bars for both cameras. Set to 0 for live images. */
#define APP_CAMERA_COLORBAR_TEST      0U
#define APP_CAMERA_WARMUP_COUNT      20U
#define APP_CAMERA_WARMUP_INTERVAL_MS 300UL
/* ST presets: 0=auto, 1=sunny, 2=office, 4=home, 8=cloudy.
   Indoor test uses the office preset because the measured auto-WB state
   stayed green (G average remained well above R/B after 20 warm-up frames).
   Set back to 0 for flight/unknown lighting once a reference scene is used. */
#if APP_INDOOR_TEST_MODE
#define APP_CAMERA_WB_MODE           2U
#else
#define APP_CAMERA_WB_MODE           0U
#endif
/* Optional diagnostic: manual equal U/V saturation (0x40/0x40). */
#define APP_CAMERA_FIXED_UV          0U
#define APP_TEST_FACTORY_IMAGE       1U
/* Streaming encoder input limit; independent of the DCMI DMA buffer. */
#define APP_SSDV_MAX_JPEG_BYTES       (512UL * 1024UL)
/* u-blox dynamic model: 6 = airborne <1 g, suitable for a balloon payload. */
#define APP_GNSS_DYN_MODEL             6U
#define APP_SSDV_PACKET_SIZE          256U
#define APP_SSDV_FEED_SIZE            128U
#define APP_SSDV_QUALITY              5
/* The internal reference image is diagnostic-only; keep VGA resolution but
   use the smallest SSDV quantisation to avoid a 15-minute APRS transfer. */
#define APP_FACTORY_SSDV_QUALITY      0

#endif

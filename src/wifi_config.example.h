// Copy to wifi_config.h (git-ignored) and fill in your network.
#pragma once
// The ESP32 joins 2.4 GHz networks only (a phone hotspot must be set to 2.4 GHz).
#define WIFI_SSID "your-network-name"
#define WIFI_PASSWORD "your-password"
// Fixed address on the network above (comma separated). Delete both lines to use DHCP.
#define WIFI_STATIC_IP 192, 168, 137, 200
#define WIFI_GATEWAY 192, 168, 137, 1
// Password for software (OTA) uploads; must match upload_flags in platformio.ini.
#define OTA_PASSWORD "imuota"

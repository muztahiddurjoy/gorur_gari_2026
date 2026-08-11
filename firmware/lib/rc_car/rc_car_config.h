#ifndef RC_CAR_CONFIG_H
#define RC_CAR_CONFIG_H

// CREDENTIALS GO IN wifi_secrets.h, NOT HERE. this include wins, so an ssid
// defined further down is silently ignored and you end up on the ap wondering
// why. wifi_secrets.h is gitignored, copy wifi_secrets.example.h over it.
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

// only reached on a fresh clone that has no wifi_secrets.h yet, purely so the
// build still goes through. an empty ssid means the car hosts its own ap.
#ifndef RC_WIFI_SSID
#define RC_WIFI_SSID ""
#endif

#ifndef RC_WIFI_PASSWORD
#define RC_WIFI_PASSWORD ""
#endif

#define RC_HOSTNAME "gorurgari"    // the controller lands on http://gorurgari.local
#define RC_AP_SSID "GorurGari-RC"  // fallback network, used when the join fails
#define RC_AP_PASSWORD "gorurgari" // wpa2 needs at least 8 characters
#define RC_HTTP_PORT 80

const unsigned long RC_WIFI_CONNECT_TIMEOUT_MS = 15000;
const unsigned long RC_FAILSAFE_MS = 500; // silence for this long and the car stops itself
const unsigned long RC_TELEMETRY_INTERVAL_MS = 100;

// full-lock swing and top duty for the websocket joystick, which sends
// normalised -1..1 sticks. only this bench-test controller uses them - on the
// real (mavlink) drive path ROS2 sends raw servo angles and duty, with the
// limits configured in ros2_ws/config/bot_config.yaml.
const int STEERING_MAX_ANGLE = 60; // how far the wheels swing either side of centre
const int MOTOR_MAX_SPEED = 255;

#endif // RC_CAR_CONFIG_H

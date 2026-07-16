#ifndef RC_CAR_H
#define RC_CAR_H

#include <Arduino.h>
#include "steering_control.h"
#include "motor_control.h"
#include "encoder_reader.h"

// kept out of the header so the async stack does not leak into main.cpp
class AsyncWebServer;
class AsyncWebSocket;

// TEST RIG ONLY. joins wifi, hosts the controller page on
// http://gorurgari.local and drives the car from a websocket. nothing else in
// the firmware includes this, so the whole folder can be deleted whenever the
// real link takes over. see README.md.
class RcCar {
private:
    SteeringController &steer;
    MotorController &motor;
    EncoderReader &encoder;
    AsyncWebServer *server;
    AsyncWebSocket *socket;
    // written from the async task, read from loop(). single aligned words, so
    // they are only ever stale by a cycle, never torn.
    volatile float steerInput;    // -1..1, left to right
    volatile float throttleInput; // -1..1, reverse to forward
    volatile unsigned long lastCommandMs;
    float appliedSteer;
    float appliedThrottle;
    unsigned long lastTelemetryMs;
    bool stopped;
    bool accessPoint;
    bool joinWifi();
    void startAccessPoint();
    void startServer();
    void apply();
    void handleCommand(uint8_t *payload, size_t length);
    void sendTelemetry();
public:
    RcCar(SteeringController &steering, MotorController &motor_controller, EncoderReader &encoder_reader);
    void begin();
    void update();
    bool isAccessPoint();
    String host();
};

#endif // RC_CAR_H

#include "rc_car.h"
#include "rc_car_config.h"
#include "rc_car_page.h"
#include "config.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

RcCar::RcCar(SteeringController &steering, MotorController &motor_controller, EncoderReader &encoder_reader)
    : steer(steering), motor(motor_controller), encoder(encoder_reader){
    server = nullptr;
    socket = nullptr;
    steerInput = 0;
    throttleInput = 0;
    lastCommandMs = 0;
    appliedSteer = NAN; // nothing written to the hardware yet
    appliedThrottle = NAN;
    lastTelemetryMs = 0;
    stopped = true;
    accessPoint = false;
}

void RcCar::begin(){
    if(!joinWifi()) startAccessPoint();

    if(MDNS.begin(RC_HOSTNAME)){
        MDNS.addService("http", "tcp", RC_HTTP_PORT);
        Serial.printf("[rc] controller on http://%s.local\n", RC_HOSTNAME);
    } else {
        Serial.println("[rc] mdns failed to start, use the ip instead");
    }
    Serial.printf("[rc] controller on http://%s\n", host().c_str());

    startServer();
    lastCommandMs = millis();
}

bool RcCar::joinWifi(){
    if(strlen(RC_WIFI_SSID) == 0){
        Serial.println("[rc] no ssid in wifi_secrets.h, going straight to ap");
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setHostname(RC_HOSTNAME);
    WiFi.setSleep(false); // a remote control wants latency, not battery life
    WiFi.begin(RC_WIFI_SSID, RC_WIFI_PASSWORD);

    Serial.printf("[rc] joining %s", RC_WIFI_SSID);
    unsigned long started = millis();
    while(WiFi.status() != WL_CONNECTED){
        if(millis() - started > RC_WIFI_CONNECT_TIMEOUT_MS){
            Serial.println(" timed out");
            return false;
        }
        Serial.print(".");
        delay(250);
    }

    accessPoint = false;
    Serial.printf(" joined, %s\n", WiFi.localIP().toString().c_str());
    return true;
}

void RcCar::startAccessPoint(){
    WiFi.mode(WIFI_AP);
    WiFi.softAP(RC_AP_SSID, RC_AP_PASSWORD);
    accessPoint = true;
    Serial.printf("[rc] hosting ap %s, %s\n", RC_AP_SSID, WiFi.softAPIP().toString().c_str());
}

void RcCar::startServer(){
    server = new AsyncWebServer(RC_HTTP_PORT);
    socket = new AsyncWebSocket("/ws");

    socket->onEvent([this](AsyncWebSocket *ws, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t length){
        if(type == WS_EVT_CONNECT){
            Serial.printf("[rc] controller %u connected\n", client->id());
            lastCommandMs = millis();
        } else if(type == WS_EVT_DISCONNECT){
            Serial.printf("[rc] controller %u gone\n", client->id());
            steerInput = 0;
            throttleInput = 0;
        } else if(type == WS_EVT_DATA){
            AwsFrameInfo *frame = (AwsFrameInfo *)arg;
            // only single frame text payloads, which is all the page ever sends
            if(frame->final && frame->index == 0 && frame->len == length && frame->opcode == WS_TEXT){
                handleCommand(data, length);
            }
        }
    });

    server->addHandler(socket);

    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        // no template processor, so the css percent signs are left alone
        request->send(200, "text/html; charset=utf-8", RC_CAR_PAGE);
    });

    // captive-portal-ish: anything else lands on the controller
    server->onNotFound([](AsyncWebServerRequest *request){
        request->redirect("/");
    });

    server->begin();
}

// runs on the async task, so it only records intent. update() is what
// actually touches the servo and the motor.
void RcCar::handleCommand(uint8_t *payload, size_t length){
    JsonDocument command;
    if(deserializeJson(command, payload, length)) return;

    const char *type = command["t"];
    if(type == nullptr) return;

    if(strcmp(type, "drive") == 0){
        float wantSteer = command["s"] | 0.0f;
        float wantThrottle = command["p"] | 0.0f;
        steerInput = constrain(wantSteer, -1.0f, 1.0f);
        throttleInput = constrain(wantThrottle, -1.0f, 1.0f);
        lastCommandMs = millis();
    } else if(strcmp(type, "zero") == 0){
        encoder.reset(); // guarded by the encoder's own critical section
    }
}

void RcCar::update(){
    unsigned long now = millis();
    bool silent = (now - lastCommandMs) > RC_FAILSAFE_MS;

    // controller went away mid-drive, stop before it hits something
    if(silent && !stopped){
        steerInput = 0;
        throttleInput = 0;
        stopped = true;
        Serial.println("[rc] failsafe, controller went quiet");
    }
    if(!silent) stopped = false;

    apply();

    if(now - lastTelemetryMs >= RC_TELEMETRY_INTERVAL_MS){
        lastTelemetryMs = now;
        sendTelemetry();
        // on the same tick, loop() spins far too fast to be grabbing the
        // client list lock out from under the async task every pass
        socket->cleanupClients();
    }
}

// no-op unless a stick actually moved, so we are not rewriting the same
// duty cycle a few thousand times a second.
void RcCar::apply(){
    float wantSteer = steerInput;
    float wantThrottle = throttleInput;
    if(wantSteer == appliedSteer && wantThrottle == appliedThrottle) return;

    steer.to(STEERING_CENTER_ANGLE + (int)lroundf(wantSteer * STEERING_MAX_ANGLE));
    motor.to((int)lroundf(wantThrottle * MOTOR_MAX_SPEED));

    appliedSteer = wantSteer;
    appliedThrottle = wantThrottle;
}

void RcCar::sendTelemetry(){
    if(socket->count() == 0) return;

    JsonDocument telemetry;
    telemetry["rpm"] = encoder.rpm();
    telemetry["ticks"] = encoder.count();
    telemetry["dir"] = encoder.direction();
    telemetry["rssi"] = accessPoint ? 0 : WiFi.RSSI();

    char buffer[128];
    size_t length = serializeJson(telemetry, buffer, sizeof(buffer));
    socket->textAll(buffer, length);
}

bool RcCar::isAccessPoint(){
    return accessPoint;
}

String RcCar::host(){
    return accessPoint ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
}

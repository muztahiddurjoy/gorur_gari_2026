#include "display_control.h"
#include <Wire.h>
#include "pins.h"
#include "debug_serial.h"

DisplayController::DisplayController(): oled(nullptr), oledClockHz(0), ready(false){}

DisplayController::~DisplayController() {
    delete oled;
}

bool DisplayController::begin(uint32_t i2cClockHz) {
    ready = false;

    // Adafruit_SSD1306::begin() reports success as long as it can allocate its
    // frame buffer - it never checks that a panel actually answers - so a
    // disconnected or still-booting screen would come back "ready" and the I2C
    // task would never retry it. Ping the address ourselves first.
    if (!isResponding()) {
        return false;
    }

    // built once and reused across retries: begin() re-sends the whole init
    // sequence and only mallocs the frame buffer if it does not have one yet,
    // so re-running it on the same object is both safe and leak free. Only a
    // change of bus clock rebuilds it, since the driver bakes the clock in.
    if (oled && oledClockHz != i2cClockHz) {
        delete oled;
        oled = nullptr;
    }
    if (!oled) {
        // the same clock during and after each frame: the default would drop
        // the shared bus to 100 kHz after every frame, under the gyro's feet
        oled = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET,
                                    i2cClockHz, i2cClockHz);
        oledClockHz = i2cClockHz;
    }
    // Wire is already up on our own SDA/SCL pins (Wire.setPins + Wire.begin
    // in setup()), so periphBegin=false: the library must not call
    // Wire.begin() itself
    if (!oled->begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, false, false)) {
        DBG_PRINTLN("SSD1306 allocation failed");
        return false;
    }
    oled->clearDisplay();
    oled->display();
    ready = true;
    return true;
}

bool DisplayController::isResponding() {
    Wire.beginTransmission(OLED_ADDRESS);
    return Wire.endTransmission() == 0;
}

void DisplayController::markLost() {
    ready = false;
}

void DisplayController::clear() {
    if(!ready || !oled) return;
    oled->clearDisplay();
}

void DisplayController::displayText(const String& text, int x, int y, int textSize) {
    if(!ready || !oled) return;
    oled->setTextSize(textSize);
    oled->setTextColor(SSD1306_WHITE);
    oled->setCursor(x, y);
    oled->println(text);
}

void DisplayController::display() {
    if(!ready || !oled) return;
    oled->display();
}

void DisplayController::updateScreen() {
    if(!ready || !oled) return;
    oled->display();
}

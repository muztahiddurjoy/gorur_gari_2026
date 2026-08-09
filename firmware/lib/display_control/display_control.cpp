#include "display_control.h"
#include <Wire.h>
#include "pins.h"
#include "debug_serial.h"

DisplayController::DisplayController(): oled(nullptr), ready(false){}

DisplayController::~DisplayController() {
    delete oled;
}

bool DisplayController::begin() {
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
    // so re-running it on the same object is both safe and leak free
    if (!oled) {
        oled = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    }
    // Wire is already up with our own SDA/SCL pins, so tell the library not to
    // call Wire.begin() itself with the core's default pins
    if (!oled->begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, false, false)) {
        DEBUG_SERIAL.println("SSD1306 allocation failed");
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

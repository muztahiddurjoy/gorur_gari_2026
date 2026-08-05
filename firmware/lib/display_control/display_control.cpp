#include "display_control.h"
#include "pins.h"

DisplayController::DisplayController(): oled(nullptr){}

bool DisplayController::begin() {
    oled = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    if (!oled->begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
        return false;
    }
    oled->clearDisplay();
    oled->display();
    return true;
}

void DisplayController::clear() {
    oled->clearDisplay();
}

void DisplayController::displayText(const String& text, int x, int y, int textSize) {
    if(!oled) return;
    oled->setTextSize(textSize);
    oled->setTextColor(SSD1306_WHITE);
    oled->setCursor(x, y);
    oled->println(text);
}

void DisplayController::display() {
    oled->display();
}

void DisplayController::updateScreen() {

    if(oled) oled->display();
}
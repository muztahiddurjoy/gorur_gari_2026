#include "display_control.h"
#include "pins.h"

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

DisplayController::DisplayController() {
        Wire.begin(I2C_SDA, I2C_SCL);
}

void DisplayController::begin() {
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
    }
    oled.clearDisplay();
}

void DisplayController::clear() {
    oled.clearDisplay();
}

void DisplayController::displayText(const String& text, int x, int y, int textSize) {
    oled.setTextSize(textSize);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(x, y);
    oled.println(text);
    oled.display();
}

void DisplayController::display() {
    oled.display();
}
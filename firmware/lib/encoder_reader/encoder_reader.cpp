#include "encoder_reader.h"

// indexed by (previous phase << 2) | current phase, where phase is (A << 1) | B.
// a valid gray code step is +/-1, an impossible double edge reads back 0.
// kept in dram so the isr never has to touch flash.
static DRAM_ATTR const int8_t QUAD_STEPS[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

EncoderReader::EncoderReader(int pin_a, int pin_b, int counts_per_rev){
    pinA = pin_a;
    pinB = pin_b;
    countsPerRev = counts_per_rev;
    sampleIntervalMs = 50;
    ticks = 0;
    lastPhase = 0;
    mux = portMUX_INITIALIZER_UNLOCKED;
    lastTicks = 0;
    lastSampleMs = 0;
    revsPerMinute = 0;
}

void EncoderReader::begin(int sample_interval_ms){
    sampleIntervalMs = sample_interval_ms;

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    lastPhase = (digitalRead(pinA) << 1) | digitalRead(pinB);
    lastSampleMs = millis();

    attachInterruptArg(pinA, onEdge, this, CHANGE);
    attachInterruptArg(pinB, onEdge, this, CHANGE);
}

void ARDUINO_ISR_ATTR EncoderReader::onEdge(void *arg){
    static_cast<EncoderReader *>(arg)->handleEdge();
}

void ARDUINO_ISR_ATTR EncoderReader::handleEdge(){
    uint8_t phase = (digitalRead(pinA) << 1) | digitalRead(pinB);

    portENTER_CRITICAL_ISR(&mux);
    ticks += QUAD_STEPS[(lastPhase << 2) | phase];
    lastPhase = phase;
    portEXIT_CRITICAL_ISR(&mux);
}

void EncoderReader::update(){
    unsigned long now = millis();
    unsigned long elapsed = now - lastSampleMs;
    if(elapsed < (unsigned long)sampleIntervalMs) return;

    long current = count();
    long delta = current - lastTicks;

    revsPerMinute = ((float)delta / (float)countsPerRev) * (60000.0f / (float)elapsed);

    lastTicks = current;
    lastSampleMs = now;
}

long EncoderReader::count(){
    portENTER_CRITICAL(&mux);
    long current = ticks;
    portEXIT_CRITICAL(&mux);
    return current;
}

float EncoderReader::revolutions(){
    return (float)count() / (float)countsPerRev;
}

float EncoderReader::rpm(){
    return revsPerMinute;
}

int EncoderReader::direction(){
    if(revsPerMinute > 0) return 1;
    if(revsPerMinute < 0) return -1;
    return 0;
}

void EncoderReader::reset(){
    portENTER_CRITICAL(&mux);
    ticks = 0;
    portEXIT_CRITICAL(&mux);

    lastTicks = 0;
    lastSampleMs = millis();
    revsPerMinute = 0;
}

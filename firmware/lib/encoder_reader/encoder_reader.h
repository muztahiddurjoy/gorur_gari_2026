#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>

// reads a two channel quadrature encoder. both channels are sampled on
// every edge (x4 decoding) so one shaft turn is countsPerRev ticks.
class EncoderReader {
private:
    int pinA;
    int pinB;
    int countsPerRev;
    int sampleIntervalMs;
    volatile long ticks;
    volatile uint8_t lastPhase;
    portMUX_TYPE mux;
    long lastTicks;
    unsigned long lastSampleMs;
    float revsPerMinute;
    static void ARDUINO_ISR_ATTR onEdge(void *arg);
    void ARDUINO_ISR_ATTR handleEdge();
public:
    EncoderReader(int pin_a, int pin_b, int counts_per_rev);
    void begin(int sample_interval_ms);
    void update(); // call from loop(), refreshes rpm every sample interval
    long count();
    float revolutions();
    float rpm();
    int direction();
    void reset();
};

#endif // ENCODER_READER_H

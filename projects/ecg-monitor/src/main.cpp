#include "mbed.h"
using namespace std::chrono_literals;

// extern declaration for Assembly function (Part F)

extern "C" uint8_t getHR(short* dataArray);

AnalogIn  ecg_output(PA_0);
DigitalIn lo_plus(PB_0);
DigitalIn lo_minus(PB_1);

// Sampling Configuration
// ECG bandwidth ~150Hz -> Nyquist = 300Hz minimum
// Using 360Hz (clinical standard - MIT-BIH dataset)
// Buffer = 360 x 3 seconds = 1080 samples
#define SAMPLE_RATE   360
#define BUFFER_SIZE   (SAMPLE_RATE * 3)   
#define REFRACTORY    36                  

Ticker         sampler;
volatile short ecg_buffer[BUFFER_SIZE];
volatile int   buf_index  = 0;
volatile bool  leads_off  = false;

// Ticker Callback - fires every 2778us (1/360Hz)
void sample_ecg() {
    if (lo_plus.read() == 1 || lo_minus.read() == 1) {
        ecg_buffer[buf_index] = 0;      
        leads_off = true;
    } else {
        leads_off = false;
        ecg_buffer[buf_index] = (short)(ecg_output.read() * 32767.0f);
    }
    buf_index = (buf_index + 1) % BUFFER_SIZE;
}

// Part E - getHR in C (reference implementation)
// Assembly version in getHR.S is the active version (Part F)

// Assumptions:
//  1. dataArray holds 1080 samples at 360Hz (3 seconds)
//  2. R-peaks are dominant high-amplitude features
//  3. Threshold = 60% of max amplitude in buffer
//  4. Refractory period = 36 samples (100ms) prevents
//     double-counting the same R-peak
//  5. If leads off (>10% zeros in buffer) return 0 BPM
//  6. Valid HR range: 40-200 BPM, else return 0 
// -------------------------------------------------------

/*
uint8_t getHR(short* dataArray) {

    // Direct LO signal check - required by Part E
    if (lo_plus.read() == 1 || lo_minus.read() == 1) {
        return 0;
    }

    // 1. Lead-off check via sentinel zeros
    int zeroCount = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (dataArray[i] == 0) zeroCount++;
    }
    if (zeroCount > BUFFER_SIZE / 10) return 0;

    // 2. Find max value for adaptive threshold
    short maxVal = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (dataArray[i] > maxVal) maxVal = dataArray[i];
    }
    short threshold = (short)(maxVal * 0.6f);  // 60% of peak

    // 3. Count R-peaks with refractory period
    int peaks      = 0;
    int refractory = 0;

    for (int i = 1; i < BUFFER_SIZE - 1; i++) {
        if (refractory > 0) {
            refractory--;
            continue;
        }
        if (dataArray[i] > threshold &&
            dataArray[i] >= dataArray[i - 1] &&
            dataArray[i] >= dataArray[i + 1]) {
            peaks++;
            refractory = REFRACTORY;
        }
    }

    // 4. Convert to BPM: peaks in 3 sec * 20 = BPM
    uint8_t bpm = (uint8_t)(peaks * 20);

    // 5. Sanity check
    if (bpm < 40 || bpm > 200) return 0;

    return bpm;
}
*/

// Main
int main() {
    lo_plus.mode(PullDown);
    lo_minus.mode(PullDown);

    printf("ECG Monitor Started - %d Hz sampling\n", SAMPLE_RATE);

    sampler.attach(&sample_ecg, 2778us);

    while (1) {
        thread_sleep_for(3000);

        uint8_t hr = getHR((short*)ecg_buffer);

        if (hr == 0) {
            printf("Invalid reading - check electrode connection\n");
        } else {
            printf("Heart Rate: %d BPM\n", hr);
        }
    }
}
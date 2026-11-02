#include "mbed.h"

I2C i2c(PB_11, PB_10);
BufferedSerial pc(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) { return &pc; }

#define LSM6DSL_ADDR  (0x6A << 1)
#define WHO_AM_I      0x0F
#define CTRL1_XL      0x10
#define CTRL2_G       0x11
#define CTRL3_C       0x12
#define OUTX_L_XL     0x28

DigitalOut led1(LED1);
DigitalOut led2(LED2);
InterruptIn btn(BUTTON1);

#define NUM_GESTURES         3
#define MAX_SAMPLES          200
#define GESTURE_WINDOW_MS    3000
#define SAMPLE_PERIOD_MS     10
#define DTW_BAND             30
#define DTW_THRESHOLD        145000000L
#define BTN_LONG_PRESS_MS    2000
#define MAX_ATTEMPTS         3
#define MOTION_THRESHOLD     150
#define STILL_THRESHOLD      80
#define STILL_DEBOUNCE_MS    300

typedef struct {
    int16_t s[MAX_SAMPLES][3];
    uint16_t len;
} Gesture;

static Gesture key[NUM_GESTURES];
static Gesture attempt[NUM_GESTURES];

volatile bool btn_pressed = false;
void btn_down() { btn_pressed = true; }


void imu_write(uint8_t reg, uint8_t val) {
    char d[2] = {(char)reg, (char)val};
    i2c.write(LSM6DSL_ADDR, d, 2);
}
uint8_t imu_read_reg(uint8_t reg) {
    char r = reg;
    i2c.write(LSM6DSL_ADDR, &r, 1, true);
    i2c.read(LSM6DSL_ADDR, &r, 1);
    return (uint8_t)r;
}
int16_t imu_read16(uint8_t reg) {
    uint8_t lo = imu_read_reg(reg);
    uint8_t hi = imu_read_reg(reg + 1);
    return (int16_t)((hi << 8) | lo);
}
void imu_read_xyz(int16_t xyz[3]) {
    xyz[0] = imu_read16(OUTX_L_XL);
    xyz[1] = imu_read16(OUTX_L_XL + 2);
    xyz[2] = imu_read16(OUTX_L_XL + 4);
}
void imu_init() {
    i2c.frequency(400000);
    uint8_t id = imu_read_reg(WHO_AM_I);
    printf("IMU WHO_AM_I=0x%02X %s\r\n", id, id==0x6A?"OK":"FAIL");
    imu_write(CTRL3_C,  0x44);
    imu_write(CTRL1_XL, 0x40);
    imu_write(CTRL2_G,  0x40);
}

static int16_t lpf[3] = {0, 0, 1000};
void lpf_update(int16_t xyz[3]) {
    lpf[0] = (int16_t)(0.4f*xyz[0] + 0.6f*lpf[0]);
    lpf[1] = (int16_t)(0.4f*xyz[1] + 0.6f*lpf[1]);
    lpf[2] = (int16_t)(0.4f*xyz[2] + 0.6f*lpf[2]);
    xyz[0]=lpf[0]; xyz[1]=lpf[1]; xyz[2]=lpf[2];
}
bool is_moving(int16_t xyz[3]) {
    int32_t ax=xyz[0], ay=xyz[1], az=xyz[2]-1000;
    int32_t m=(ax*ax+ay*ay+az*az)/10000;
    return m > (MOTION_THRESHOLD*MOTION_THRESHOLD/10000);
}
bool is_still(int16_t xyz[3]) {
    int32_t ax=xyz[0], ay=xyz[1], az=xyz[2]-1000;
    int32_t m=(ax*ax+ay*ay+az*az)/10000;
    return m < (STILL_THRESHOLD*STILL_THRESHOLD/10000);
}

/* ── LED patterns ─────────────────────────────────────────── */
void leds_off() { led1=0; led2=0; }

void led_recording_on()  { led1=1; led2=0; }
void led_recording_off() { led1=0; led2=0; }

void led_gesture_saved() {
    printf("  [LED] LED1 double-blinks = gesture saved\r\n");
    for(int i=0;i<2;i++){
        led1=1; ThisThread::sleep_for(80ms);
        led1=0; ThisThread::sleep_for(80ms);
    }
}

void led_all_recorded() {
    printf("  [LED] Both LEDs blink 3x = all gestures saved!\r\n");
    for(int i=0;i<3;i++){
        led1=1; led2=1; ThisThread::sleep_for(150ms);
        led1=0; led2=0; ThisThread::sleep_for(150ms);
    }
}

void led_gesture_match(uint8_t gesture_num) {
    printf("  [LED] LED1 fast blinks = gesture %u matched!\r\n", gesture_num);
    for(int i=0;i<4;i++){
        led1=1; ThisThread::sleep_for(60ms);
        led1=0; ThisThread::sleep_for(60ms);
    }
}

void led_fully_unlocked() {
    printf("  [LED] BOTH LEDs fast blinking = UNLOCKED!\r\n");
    for(int i=0;i<10;i++){
        led1=1; led2=1; ThisThread::sleep_for(80ms);
        led1=0; led2=0; ThisThread::sleep_for(80ms);
    }
}

void led_wrong() {
    printf("  [LED] LED1 slow blinks = wrong gesture\r\n");
    for(int i=0;i<3;i++){
        led1=1; ThisThread::sleep_for(400ms);
        led1=0; ThisThread::sleep_for(400ms);
    }
}

void led_lockout() {
    printf("  [LED] LEDs alternating = LOCKED OUT\r\n");
    for(int i=0;i<10;i++){
        led1=1; led2=0; ThisThread::sleep_for(200ms);
        led1=0; led2=1; ThisThread::sleep_for(200ms);
    }
    leds_off();
}


static int32_t dtw_prev[MAX_SAMPLES+1];
static int32_t dtw_curr[MAX_SAMPLES+1];

int32_t dtw(const Gesture *a, const Gesture *b) {
    uint16_t m=a->len, n=b->len;
    if(!m||!n) return INT32_MAX;
    int32_t INF=INT32_MAX/2;
    for(uint16_t j=0;j<=n;j++) dtw_prev[j]=INF;
    dtw_prev[0]=0;
    for(uint16_t i=1;i<=m;i++){
        for(uint16_t j=0;j<=n;j++) dtw_curr[j]=INF;
        uint16_t jlo=(i>DTW_BAND)?(i-DTW_BAND):1;
        uint16_t jhi=(i+DTW_BAND<n)?(i+DTW_BAND):n;
        for(uint16_t j=jlo;j<=jhi;j++){
            int32_t dx=(a->s[i-1][0]-b->s[j-1][0])/10;
            int32_t dy=(a->s[i-1][1]-b->s[j-1][1])/10;
            int32_t dz=(a->s[i-1][2]-b->s[j-1][2])/10;
            int32_t cost=dx*dx+dy*dy+dz*dz;
            int32_t best=dtw_prev[j-1];
            if(dtw_prev[j]<best)   best=dtw_prev[j];
            if(dtw_curr[j-1]<best) best=dtw_curr[j-1];
            dtw_curr[j]=(best>=INF)?INF:cost+best;
        }
        memcpy(dtw_prev,dtw_curr,(n+1)*sizeof(int32_t));
    }
    return dtw_prev[n];
}


void capture(Gesture *g) {
    g->len = 0;
    printf("  Waiting for motion...\r\n");

    uint32_t wait_start = HAL_GetTick();
    bool motion_detected = false;
    while(HAL_GetTick()-wait_start < GESTURE_WINDOW_MS){
        int16_t xyz[3];
        imu_read_xyz(xyz);
        lpf_update(xyz);
        if(is_moving(xyz)){ motion_detected=true; break; }
        ThisThread::sleep_for(std::chrono::milliseconds(SAMPLE_PERIOD_MS));
    }

    if(!motion_detected){
        printf("  Timeout — no motion detected.\r\n");
        return;
    }

    printf("  Motion detected! Recording...\r\n");
    led_recording_on();

    uint32_t rec_start=HAL_GetTick();
    uint32_t still_since=0;
    bool was_still=false;

    while(HAL_GetTick()-rec_start < GESTURE_WINDOW_MS){
        uint32_t t0=HAL_GetTick();
        int16_t xyz[3];
        imu_read_xyz(xyz);
        lpf_update(xyz);

        if(g->len < MAX_SAMPLES){
            g->s[g->len][0]=xyz[0];
            g->s[g->len][1]=xyz[1];
            g->s[g->len][2]=xyz[2];
            g->len++;
        }

        if(is_still(xyz)){
            if(!was_still){ still_since=HAL_GetTick(); was_still=true; }
            else if(HAL_GetTick()-still_since > STILL_DEBOUNCE_MS) break;
        } else { was_still=false; }

        uint32_t elapsed=HAL_GetTick()-t0;
        if(elapsed<(uint32_t)SAMPLE_PERIOD_MS)
            ThisThread::sleep_for(std::chrono::milliseconds(SAMPLE_PERIOD_MS));
    }

    led_recording_off();
    printf("  Captured %u samples\r\n", g->len);
}


uint32_t wait_btn() {
    btn_pressed = false;
    while(!btn_pressed){}
    uint32_t t0=HAL_GetTick();
    while(btn.read()==0){}
    return HAL_GetTick()-t0;
}

void do_record() {
    printf("\r\n");
    printf("╔══════════════════════════════╗\r\n");
    printf("║        RECORD MODE           ║\r\n");
    printf("╚══════════════════════════════╝\r\n");
    printf("Record 3 gestures to set your key.\r\n");
    printf("Press button before each gesture.\r\n");
    printf("Move the board — recording starts on motion.\r\n");
    printf("Hold still at the end to finish.\r\n\r\n");
    printf("[LED GUIDE - RECORD]\r\n");
    printf("  LED1 solid ON     = recording your gesture\r\n");
    printf("  LED1 double-blink = gesture saved\r\n");
    printf("  Both LEDs 3x      = all 3 gestures saved!\r\n\r\n");

    for(uint8_t i=0; i<NUM_GESTURES; i++){
        printf("──────────────────────────────\r\n");
        printf("Gesture %u of %u\r\n", i+1, NUM_GESTURES);
        printf("Press button when ready...\r\n");

        for(uint8_t b=0;b<=i;b++){
            led1=1; ThisThread::sleep_for(200ms);
            led1=0; ThisThread::sleep_for(200ms);
        }

        wait_btn();
        printf("Go! Move the board now.\r\n");
        capture(&key[i]);

        if(key[i].len == 0){
            printf("Nothing recorded — try again.\r\n");
            i--;
            continue;
        }

        led_gesture_saved();
        printf("Gesture %u saved! (%u samples)\r\n\r\n", i+1, key[i].len);
        ThisThread::sleep_for(500ms);
    }

    led_all_recorded();
    printf("──────────────────────────────\r\n");
    printf("All 3 gestures saved as your key!\r\n");
    printf("\r\nLong press now (>2s) to re-record.\r\n");
    printf("Or wait to go to unlock mode...\r\n\r\n");

    ThisThread::sleep_for(500ms);
    btn_pressed = false;
    uint32_t wait_start = HAL_GetTick();
    while(HAL_GetTick()-wait_start < 4000){
        if(btn_pressed){
            uint32_t t0=HAL_GetTick();
            while(btn.read()==0){}
            if(HAL_GetTick()-t0 >= BTN_LONG_PRESS_MS){
                printf("Re-recording...\r\n");
                do_record();
                return;
            }
        }
        ThisThread::sleep_for(50ms);
    }
}

/* main */
int main() {
    btn.fall(&btn_down);
    imu_init();

    printf("\r\n");
    printf("╔══════════════════════════════╗\r\n");
    printf("║         KinetiKey            ║\r\n");
    printf("║   Gesture Combination Lock   ║\r\n");
    printf("║   ECE-GY 6483  Spring 2026   ║\r\n");
    printf("╚══════════════════════════════╝\r\n\r\n");

    do_record();

    int fail_count = 0;

    while(1){

        /* ── Lockout check ── */
        if(fail_count >= MAX_ATTEMPTS){
            printf("\r\n!!! LOCKED OUT — too many failed attempts !!!\r\n");
            printf("Hold button 2s to reset.\r\n");
            led_lockout();
            while(1){
                uint32_t dur=wait_btn();
                if(dur>=BTN_LONG_PRESS_MS){
                    fail_count=0;
                    printf("Lockout cleared.\r\n");
                    break;
                }
                printf("Keep holding...\r\n");
            }
        }

        /* ── Unlock mode header ── */
        printf("\r\n");
        printf("╔══════════════════════════════╗\r\n");
        printf("║        UNLOCK MODE           ║\r\n");
        printf("╚══════════════════════════════╝\r\n");
        printf("Attempts remaining: %d/%d\r\n",
               MAX_ATTEMPTS-fail_count, MAX_ATTEMPTS);
        printf("Perform your 3-gesture key.\r\n");
        printf("Long press at any time to re-record.\r\n\r\n");
        printf("[LED GUIDE - UNLOCK]\r\n");
        printf("  LED1 solid ON        = recording gesture\r\n");
        printf("  LED1 fast blinks     = that gesture matched!\r\n");
        printf("  BOTH LEDs fast blink = FULLY UNLOCKED!\r\n");
        printf("  LED1 slow blinks     = wrong — try again\r\n\r\n");

        /* Ready signal — double blink */
        for(int b=0;b<2;b++){
            led1=1;led2=1; ThisThread::sleep_for(300ms);
            led1=0;led2=0; ThisThread::sleep_for(300ms);
        }

        bool re_record = false;
        uint8_t matched_so_far = 0;

        /* ── Capture 3 gestures ── */
        for(uint8_t i=0; i<NUM_GESTURES; i++){
            printf("──────────────────────────────\r\n");
            printf("Gesture %u of %u\r\n", i+1, NUM_GESTURES);
            printf("Press button when ready...\r\n");

            led1=1; ThisThread::sleep_for(300ms); led1=0;

            uint32_t dur=wait_btn();
            if(dur>=BTN_LONG_PRESS_MS){
                re_record=true;
                break;
            }

            printf("Go! Move the board now.\r\n");
            capture(&attempt[i]);
        }

        if(re_record){
            fail_count=0;
            do_record();
            continue;
        }

        /* ── Compare ── */
        printf("\r\nComparing your gestures to the key...\r\n");
        printf("──────────────────────────────\r\n");
        uint8_t all_match=1;

        for(uint8_t i=0; i<NUM_GESTURES; i++){
            int32_t d=dtw(&key[i], &attempt[i]);
            bool match=(d<=DTW_THRESHOLD);
            printf("Gesture %u: dist=%-12ld  %s\r\n",
                   i+1, (long)d, match?"MATCH":"NO MATCH");

            if(match){
                matched_so_far++;
                led_gesture_match(i+1);
            } else {
                all_match=0;
            }
        }

        printf("──────────────────────────────\r\n");

        /* ── Result ── */
        if(all_match){
            printf("\r\n");
            printf("╔══════════════════════════════╗\r\n");
            printf("║      *** UNLOCKED! ***       ║\r\n");
            printf("║   All 3 gestures matched!    ║\r\n");
            printf("╚══════════════════════════════╝\r\n\r\n");
            led_fully_unlocked();

            /* Stay unlocked — both LEDs solid until button press */
            printf("System unlocked!\r\n");
            printf("Press button to lock again.\r\n");
            led1=1; led2=1;
            wait_btn();
            led1=0; led2=0;
            fail_count=0;
            printf("Locked again.\r\n");

        } else {
            fail_count++;
            printf("\r\n--- FAILED (%u/3 matched) ---\r\n", matched_so_far);

            if(fail_count < MAX_ATTEMPTS){
                printf("Attempt %d failed. Try again.\r\n", fail_count);
                printf("Attempts remaining: %d/%d\r\n",
                       MAX_ATTEMPTS-fail_count, MAX_ATTEMPTS);
            }
            led_wrong();
        }
    }
}
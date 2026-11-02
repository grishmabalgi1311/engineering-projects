#include "simpletools.h"
#include "servo.h"
#include "fdserial.h"

#define rp_motor   13
#define f_motor    14
#define b_motor    12
#define PIVOT_PIN  15

#define IN1         7
#define IN2         6

#define ACTUATOR_EXTEND_MS   3800
#define ACTUATOR_RETRACT_MS  3800

#define PIVOT_RIGHT_ANGLE    1600
#define PIVOT_LEFT_ANGLE      900

#define TRAVEL_SPEED     200

unsigned int stack1[65];

static volatile int f_arm_state = 1;   // 1 = open, 0 = closed
static volatile int b_arm_state = 1;   // 1 = open, 0 = closed

volatile char current_command = 'S';
fdserial *bt;

void control(void *par);

void forward(void);
void backward(void);
void f_arm(void);
void b_arm(void);

void pivot_right(void);
void pivot_left(void);

void actuator_init(void);
void actuator_stop(void);
void actuator_extend(void);
void actuator_retract(void);
void actuator_retract_checked(void);

void send_done(void);

int main() {
  actuator_init();

  int cog1 = cogstart(control, NULL, stack1, sizeof(stack1));

  pause(1000);

  if (cog1 == -1) {
    while (1) pause(100);
  }

  while (1) {
    if (current_command == 'S') {
      servo_speed(rp_motor, 0);
      pause(100);
    }
    else if (current_command == 'F') {
      current_command = 'S';
      forward();
      current_command = 'S';
      send_done();
    }
    else if (current_command == 'B') {
      current_command = 'S';
      backward();
      current_command = 'S';
      send_done();
    }
    else if (current_command == 'R') {
      current_command = 'S';
      pivot_right();
      current_command = 'S';
    }
    else if (current_command == 'L') {
      current_command = 'S';
      pivot_left();
      current_command = 'S';
    }

    pause(100);
  }
}

void send_done(void) {
  if (bt != NULL) {
    dprint(bt, "D");
  }
}

void actuator_init(void) {
  low(IN1);
  low(IN2);
}

void actuator_stop(void) {
  low(IN1);
  low(IN2);
}

void actuator_extend(void) {
  high(IN1);
  low(IN2);
  pause(ACTUATOR_EXTEND_MS);
  actuator_stop();
}

void actuator_retract(void) {
  low(IN1);
  high(IN2);
  pause(ACTUATOR_RETRACT_MS);
  actuator_stop();
}

void actuator_retract_checked(void) {
  low(IN1);
  high(IN2);

  for (int t = 0; t < ACTUATOR_RETRACT_MS; t += 20) {
    pause(20);
  }

  actuator_stop();
}

void control(void *par) {
  bt = fdserial_open(8, 9, 0, 9600);
  pause(200);

  while (1) {
    if (fdserial_rxReady(bt)) {
      char new_c = fdserial_rxChar(bt);

      if (new_c == 'F' || new_c == 'B' || new_c == 'S' ||
          new_c == 'R' || new_c == 'L') {
        current_command = new_c;
      }
    }

    pause(50);
  }
}

void f_arm(void) {
  if (f_arm_state == 0) {
    servo_speed(f_motor, -150);
    pause(4500);
  } else {
    servo_speed(f_motor, 150);
    pause(3000);
  }

  servo_speed(f_motor, 0);
  servo_stop();
  f_arm_state = !f_arm_state;
}

void b_arm(void) {
  if (b_arm_state == 0) {
    servo_speed(b_motor, -150);
    pause(4000);
  } else {
    servo_speed(b_motor, 150);
    pause(3000);
  }

  servo_speed(b_motor, 0);
  servo_stop();
  b_arm_state = !b_arm_state;
}

void forward(void) {
  if (f_arm_state == 0) f_arm();   // open front  if closed
  if (b_arm_state == 1) b_arm();   // close back  if open

  servo_speed(rp_motor, -TRAVEL_SPEED);
  pause(3200);
  servo_speed(rp_motor, 0);
  servo_stop();

  if (f_arm_state == 1) f_arm();   // close front if open
  if (b_arm_state == 0) b_arm();   // open back   if closed

  servo_speed(rp_motor, TRAVEL_SPEED);
  pause(3200);
  servo_speed(rp_motor, 0);
  servo_stop();

  if (b_arm_state == 1) b_arm();   // close back  if open
  if (f_arm_state == 0) f_arm();   // open front  if closed
}

void backward(void) {
  if (b_arm_state == 0) b_arm();   // open back   if closed
  if (f_arm_state == 1) f_arm();   // close front if open

  servo_speed(rp_motor, -TRAVEL_SPEED);
  pause(2500);
  servo_speed(rp_motor, 0);
  servo_stop();

  if (b_arm_state == 1) b_arm();   // close back  if open
  if (f_arm_state == 0) f_arm();   // open front  if closed

  servo_speed(rp_motor, TRAVEL_SPEED);
  pause(2500);
  servo_speed(rp_motor, 0);
  servo_stop();

  if (f_arm_state == 1) f_arm();   // close front if open
  if (b_arm_state == 0) b_arm();   // open back   if closed
}

void pivot_right(void) {
  // Both arms open before pivot
  if (f_arm_state == 0) f_arm();   // open front if closed
  if (b_arm_state == 0) b_arm();   // open back  if closed

  actuator_retract_checked();
  servo_angle(PIVOT_PIN, 0);
  pause(200);
  servo_set(PIVOT_PIN, 0);
  pause(200);

  actuator_extend();
  
  servo_angle(PIVOT_PIN, PIVOT_RIGHT_ANGLE);
  pause(2000);

  actuator_retract_checked();
  servo_angle(PIVOT_PIN, 0);
  pause(200);
  servo_set(PIVOT_PIN, 0);
  send_done();
}

void pivot_left(void) {
  if (f_arm_state == 0) f_arm();   // open front if closed
  if (b_arm_state == 0) b_arm();   // open back  if closed

  actuator_retract_checked();
  servo_angle(PIVOT_PIN, 0);
  pause(200);
  servo_set(PIVOT_PIN, 0);
  pause(200);

  actuator_extend();

  servo_angle(PIVOT_PIN, PIVOT_LEFT_ANGLE);
  pause(2000);

  actuator_retract_checked();
  servo_angle(PIVOT_PIN, 0);
  pause(200);
  servo_set(PIVOT_PIN, 0);
  send_done();
}
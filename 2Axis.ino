// שליטה על שני מנועים עם AccelStepper + TMC2209 UART + ESP32

#include <TMCStepper.h>
#include <AccelStepper.h>

#define R_SENSE 0.11f
#define DRIVER_ADDRESS_1 0b00
#define DRIVER_ADDRESS_2 0b01

#define STEP_PIN_1 26
#define DIR_PIN_1  25
#define STEP_PIN_2 17
#define DIR_PIN_2  16

#define RXD2     32
#define TXD2     33

#define LIMIT_SWITCH_PIN 21

HardwareSerial TMCSerial(2);
TMC2209Stepper driver1(&TMCSerial, R_SENSE, DRIVER_ADDRESS_1);
TMC2209Stepper driver2(&TMCSerial, R_SENSE, DRIVER_ADDRESS_2);

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

// מנוע 1 (ימינה-שמאלה)
long current_position_1 = 0;
const long STEPS_PER_REV_1 = 12800;
const long ZERO_OFFSET_1 = 1600;
const long MIN_POSITION_1 = -ZERO_OFFSET_1;
const long MAX_POSITION_1 = 19200;

// מנוע 2 (למעלה-למטה)
long current_position_2 = 5000;
const long MIN_POSITION_2 = -5000;
const long MAX_POSITION_2 = 10000;

// מצב מנוע שני (true אם מחובר)
bool ENABLE_TILT = false;

void setup() {
  Serial.begin(115200);
  TMCSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  driver1.begin();
  driver1.toff(5);
  driver1.rms_current(600);
  driver1.microsteps(16);
  driver1.en_spreadCycle(false);
  driver1.pwm_autoscale(true);

  driver2.begin();
  driver2.toff(5);
  driver2.rms_current(600);
  driver2.microsteps(16);
  driver2.en_spreadCycle(false);
  driver2.pwm_autoscale(true);

  stepper1.setMaxSpeed(8000);
  stepper1.setAcceleration(100000);

  if (ENABLE_TILT) {
    stepper2.setMaxSpeed(8000);
    stepper2.setAcceleration(100000);
  }

  Serial.println("System ready. Send two values: rotation (0–12800) and tilt (0–10000), or -1 to home.");
}

void loop() {
  stepper1.run();
  if (ENABLE_TILT) stepper2.run();

  if (Serial.available() > 0) {
    long input1 = Serial.parseInt();
    while (Serial.available() == 0); // מחכה לנתון שני
    long input2 = Serial.parseInt();

    if (input1 == -1 || input2 == -1) {
      home();
    } else {
      moveToRotation(input1);
      if (ENABLE_TILT) moveToTilt(input2);
    }
  }
}

void moveToRotation(long target) {
  long norm_current = current_position_1 % STEPS_PER_REV_1;
  long clockwise_steps = (target - norm_current + STEPS_PER_REV_1) % STEPS_PER_REV_1;
  long counter_steps   = (norm_current - target + STEPS_PER_REV_1) % STEPS_PER_REV_1;

  long pos_after_cw  = current_position_1 + clockwise_steps;
  long pos_after_ccw = current_position_1 - counter_steps;

  bool crosses_block_cw = (current_position_1 < ZERO_OFFSET_1) && (pos_after_cw >= ZERO_OFFSET_1);
  bool clockwise_valid  = !crosses_block_cw && (pos_after_cw <= MAX_POSITION_1);
  bool counter_valid    = (pos_after_ccw >= MIN_POSITION_1);

  bool go_clockwise;

  if (clockwise_valid && counter_valid) {
    go_clockwise = (clockwise_steps <= counter_steps);
  } else if (clockwise_valid) {
    go_clockwise = true;
  } else {
    go_clockwise = false;
  }

  long steps = go_clockwise ? clockwise_steps : counter_steps;
  long target_position = go_clockwise ? (current_position_1 + steps) : (current_position_1 - steps);

  stepper1.moveTo(target_position);
  current_position_1 = target_position;

  Serial.print("Rotation set to: ");
  Serial.println(current_position_1 % STEPS_PER_REV_1);
}

void moveToTilt(long target) {
  target = constrain(target, MIN_POSITION_2, MAX_POSITION_2);
  stepper2.moveTo(target);
  current_position_2 = target;

  Serial.print("Tilt set to: ");
  Serial.println(current_position_2);
}

void home() {
  Serial.println("Homing rotation...");

  digitalWrite(DIR_PIN_1, HIGH);
  while (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
    digitalWrite(STEP_PIN_1, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN_1, LOW);
    delayMicroseconds(1000);
  }

  Serial.println("Switch hit. Moving away...");

  digitalWrite(DIR_PIN_1, LOW);
  for (int i = 0; i < ZERO_OFFSET_1; i++) {
    digitalWrite(STEP_PIN_1, HIGH);
    delayMicroseconds(300);
    digitalWrite(STEP_PIN_1, LOW);
    delayMicroseconds(300);
  }

  current_position_1 = 0;
  stepper1.setCurrentPosition(0);
  Serial.println("Rotation homed.");

  if (ENABLE_TILT) {
    current_position_2 = (MAX_POSITION_2 + MIN_POSITION_2) / 2;
    stepper2.setCurrentPosition(current_position_2);
    Serial.print("Tilt homed to midpoint: ");
    Serial.println(current_position_2);
  }
}

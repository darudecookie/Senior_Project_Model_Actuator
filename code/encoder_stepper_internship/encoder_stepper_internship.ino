#include <AccelStepper.h>
#define FULLSTEP 4
#include "Arduino.h"

const int clk_pin = 2;
const int dt_pin = 3;
const int sw_pin = 4;
int encoder_position = 0;

AccelStepper stepper(FULLSTEP, 11, 9, 10, 8);
const int steps_per_revolution = 64 * 32;

const float reduction_factor = 0.05;

void setup() {
  Serial.begin(9600);
  Serial.println("begin setup!");

  attachInterrupt(digitalPinToInterrupt(clk_pin), updateEncoder, CHANGE);
  pinMode(sw_pin, INPUT);

  stepper.setMaxSpeed(500);
  stepper.setAcceleration(1000);
  stepper.setSpeed(400);

  Serial.println("end setup!");
}

void loop() {
  static int previous_encoder_position = 0;
  static long last_motor_position_update = 0;
  if (millis() - last_motor_position_update > 250) {
    last_motor_position_update = millis();
    stepper.move((encoder_position - previous_encoder_position) * steps_per_revolution * reduction_factor);
    previous_encoder_position = encoder_position;
  }
  
  /*
  static bool should_zero = true;
  if (digitalRead(sw_pin) == LOW) {
    if (should_zero) {
      encoder_position = 0;
    }
  } else {
    should_zero = true;
  }
  */

  while (stepper.distanceToGo() != 0) {
    stepper.run();
    yield();
  }
}

void updateEncoder() {
  static int last_clk = 0;
  int current_clk = digitalRead(clk_pin);

  if (current_clk != last_clk && current_clk == 1) {
    if (digitalRead(dt_pin) != current_clk) {
      encoder_position--;
    } else {
      encoder_position++;
    }
  }
  last_clk = current_clk;
}

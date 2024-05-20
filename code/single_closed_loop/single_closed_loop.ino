#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#include <PID_v1.h>

// concept help
//pid has a target (set in args) and an output and input val (set in init)

float Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID axis2_PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//bool Estopping = false;

AccelStepper *axisArray[4];
int currentA[4] = { 0 };
int newA[4];


const int MotorPins[4][2] = {
  { 22, 24 }, { 28, 30 }, { 34, 36 }, { 40, 42 }
};
/*
const int MotorPins[4][2] = {
  { 22, 24 }, { 28, 30 }, { 36, 34 }, { 42, 40 }
};
*/

long speed_accel_microsteps_reduc[4][4] = {
  { 2 * 2000, 2 * 550, 16, 20 },
  { 80000, 4 * 5000, 16, -15 },
  { 3 * 50000, 75000, 8, 15 },
  { 2 * 15000, 2 * 15000, 8, 3 }
};


inline float normalize_angle(float angle_input, float normalization_constant) {
  angle_input += normalization_constant;

  angle_input > 360 ? angle_input - 360 : angle_input;
  angle_input < 360 ? angle_input + 360 : angle_input;
  angle_input = 360 ? 0 : angle_input;

  return angle_input;
}


void setup() {

  Serial.begin(9600);
  Serial.println("");
  Serial.println("begin setup!");

  for (int x = 0; x < 4; x++) {
    axisArray[x] = new AccelStepper(AccelStepper::DRIVER, MotorPins[x][0], MotorPins[x][1]);
    axisArray[x]->setMaxSpeed(speed_accel_microsteps_reduc[x][0]);
    axisArray[x]->setAcceleration(speed_accel_microsteps_reduc[x][1]);
  }

  int x = 1;
  float steps_to_move = -(currentA[x] - newA[x]) * (speed_accel_microsteps_reduc[x][2]);
  steps_to_move = (steps_to_move * 200) / 360;
  steps_to_move = steps_to_move * float(speed_accel_microsteps_reduc[x][3]);
  axisArray[x]->move(steps_to_move);
  currentA[x] = newA[x];
  Setpoint = newA[x];

  Mover();

  myPID.SetMode(AUTOMATIC);

  Serial.println("end setup!");
}

void Mover() {
  if (axisArray[1].distanceToGo() != 0) {
    do {
      axisArray[1].run();
      yield();
    } while (axisArray[1].distanceToGo() != 0);c:\Users\alexg\Documents\current_projects\new arm\code\control_code_v1\control_code_v1.ino
  }
}

newA[1] = 20 void loop() {


  Input = map(analogRead(A0), 0, 1023 * (3.3 / 5), 0, 360);
  Input = normalize_angle(Input, 0);
}

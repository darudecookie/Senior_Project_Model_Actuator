#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>
#include <Wire.h>

//motor config setup
bool zeroing;

AccelStepper *axisArray[4];
int current_angle = 0;

const int MotorPins[4][2] = {
  { 22, 24 }, { 28, 30 }, { 34, 36 }, { 40, 42 }
};

const int ServoPins[2] = { 46, 50 };

/*
const long speed_accel_microsteps_reduc[4][4] = {
  { 2 * 2000, 2 * 550, 16, 20 },
  { 80000, 4 * 5000, 16, -15 },
  { 3 * 50000, 75000, 8, 15 },
  { 2 * 15000, 2 * 15000, 8, 3 }
};
*/

const long speed_accel_microsteps_reduc[4][4] = {
  { 2 * 2000, 2 * 550, 16, 20 },
  { 80000, 4 * 5000, 16, -15 },
  { 3 * 50000, 75000, 8, 15 },
  { 3 * 50000, 75000, 8, 15 },
};


Servo Axis5;
Servo Axis6;

//gyro setup code
const int MPU_addr = 0x68;
float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int minVal = 265;
int maxVal = 402;


float gyro_coords[3];
float zero_angle;
float high_test_angle;
float low_test_angle;

float zero_error[25];
float forward_error[25];
float backward_error[25];

//-----------------------CONFIG---------------------
//axis-coord pairs: axis 1: ???;axis 2: Y; axis 3: Y; axis 4: Z;
int coord_oi = 1;  //0: X, 1: Y, 2: Z
int axis_oi = 3;
int initial_angle = 0;
int offset_test_angle = 75;

int number_of_tests = 3;
int pause_time = 500;  //time in ms to wait between finishing movement and surveying gyro
//-----------------------CONFIG---------------------



void setup() {
  Serial.begin(19200);
  Serial.println("begin setup!");

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  for (int x = 0; x < 4; x++) {
    axisArray[x] = new AccelStepper(AccelStepper::DRIVER, MotorPins[x][0], MotorPins[x][1]);
    axisArray[x]->setMaxSpeed(speed_accel_microsteps_reduc[x][0]);
    axisArray[x]->setAcceleration(speed_accel_microsteps_reduc[x][1]);
  }

  Axis5.attach(ServoPins[0]);
  Axis6.attach(ServoPins[1]);
  delay(1000);

  move_to(initial_angle);
  delay(pause_time);

  Serial.println("end setup; begin zero!");

  zero_angle = angle_survey(20, 1);

  high_test_angle = zero_angle + offset_test_angle;
  low_test_angle = zero_angle - offset_test_angle;

  Serial.print("end zero; zero angle: ");
  Serial.println(zero_angle);
  delay(pause_time);
}


void loop() {
  int test_angle;
  for (int i = 0; i < number_of_tests; i++) {
    Serial.println("");
    Serial.print("test number ");
    Serial.print(i + 1);

    move_to(initial_angle + offset_test_angle);
    delay(pause_time);
    test_angle = angle_survey(10, 1);
    forward_error[i] = ((zero_angle - test_angle) - offset_test_angle);
    delay(pause_time);

    move_to(initial_angle);
    delay(pause_time);
    test_angle = angle_survey(10, 1);
    zero_error[i] = (test_angle - zero_angle);
    delay(pause_time);

    move_to(initial_angle - offset_test_angle);
    delay(pause_time);
    test_angle = angle_survey(10, 1);
    backward_error[i] = ((zero_angle - test_angle) + offset_test_angle);
    delay(pause_time);

    move_to(initial_angle);
    delay(pause_time);
  }

  delay(pause_time);
  move_to(initial_angle);

  float mean_forward_error, mean_zero_error, mean_backward_error;
  mean_forward_error = mean_zero_error = mean_backward_error = 0;

  for (int i = 0; i < number_of_tests; i++) {
    mean_forward_error += forward_error[i];
    mean_zero_error += zero_error[i];
    mean_backward_error += backward_error[i];
  }

  mean_forward_error /= number_of_tests;
  mean_zero_error /= number_of_tests;
  mean_backward_error /= number_of_tests;

  Serial.println("");
  Serial.print("mean forward error: ");
  Serial.print(mean_forward_error);
  Serial.print(" degrees; mean zero error: ");
  Serial.print(mean_zero_error);
  Serial.print(" degrees; mean backward error: ");
  Serial.print(mean_backward_error);
  Serial.print(" degrees");

  delay(1000000000);
}


float check_current_angles() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  float xAng = map(AcX, minVal, maxVal, -90, 90);
  float yAng = map(AcY, minVal, maxVal, -90, 90);
  float zAng = map(AcZ, minVal, maxVal, -90, 90);

  if (coord_oi == 0) {
    return RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  } else if (coord_oi == 1) {
    return RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  } else if (coord_oi == 2) {
    return RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
  }
}


float angle_survey(int checks_sec, int seconds) {
  long start = millis();
  float survey_result = 0;
    for (int i = 0; i < (checks_sec * seconds); i++) {
      survey_result += check_current_angles();
      delay((float)(checks_sec / 1000));
      Serial.print(i);
      Serial.print(" ");
    }
    survey_result /= checks_sec * seconds;
  

  Serial.println("");
  Serial.print("survey complete with a runtime of ");
  Serial.print(millis() - start);
  Serial.print("ms and an angle of ");
  Serial.print((int)survey_result);
  Serial.print(" degrees");

  return survey_result;
}


void move_to(int new_angle) {

  long steps_to_move = (new_angle - current_angle) * (speed_accel_microsteps_reduc[axis_oi][2] * speed_accel_microsteps_reduc[axis_oi][3] * 200 / 360);

  axisArray[axis_oi]->move(steps_to_move);

  while (axisArray[axis_oi]->distanceToGo() != 0) {
    axisArray[axis_oi]->run();
  }
  axisArray[axis_oi]->stop();

  current_angle = new_angle;
}

#include <Arduino.h>
#include <AccelStepper.h>
#include <FastPID.h>

// number of axes
//******IMPORTANT*********
const int stepper_servo_axes = 2;

// these global angle values manage program's target angle
float actual_angles[stepper_servo_axes];                        // current angles as measured form encoder
float target_angles[stepper_servo_axes] = { 0, 0 };             // angles for each joint ot move to
bool new_target_angles[stepper_servo_axes] = { false, false };  // array of whther each axis has read an updated target angle, true if angle is unread, false otherwise

// these are global status variables and acceptable values
String global_status = "";                                          // global status, used to communicate from serial to various arm functions, eg e-stop
bool new_global_status = false;                                     // whether or not there is global status that hasn't been used, true if global status has not been read by a function, false otherwise
const String e_stop_status_msg = "estop";                           // status message that will trigger e-stop
const String end_effector_open_status_msg = "end_effector:open";    // status message that opens end effector, as of 5-4-24 this is a placeholder
const String end_effector_close_status_msg = "end_effector:close";  // status message that closes end effector, as of 5-4-24 this is a placeholder

// intervals between when functions should run in microsecs
const int serial_update_runtime_microsecs = 0;                                           // interval for serial update func. to loop at
const int e_stop_runtime_microsteps = 250;                                               // interval for e-stop func. to loop at
const int StepperServo_runtime_interval_microsecs[stepper_servo_axes] = { 5000, 5000 };  // interval for joints to loop at. this is an array as opposed to one value because you might want joints that have to move quickly or make fine adjustments like a wrist mechanism to refresh faster, not sure if this is really necessary though

// global time var for syncing
long global_time;

// joint param
const int motor_pins[stepper_servo_axes][2] = {
  // pins motors are plugged into, order {step, direction}
  { 25, 23 },
  { 29, 27 },
};
const float microsteps_reduction_params[stepper_servo_axes][2] = {
  // microsteps and reduction of each gearbox, order {microsteps, reduction}. 1 degree of joint movement = motor_steps_per_rev * microsteps * gearbox_reduction / 360                                                                                                                             // gear stepper microsteps and gear reduction for each axes, order {microstep, reduction}
  { 8, 2.60869 },
  { 8, 2.60869 },
};
const long speed_accel_params[stepper_servo_axes][2] = { { 500, 1000 }, { 500, 1000 } };  // speed and acceleration params of motor, in steps/second (including microsteps), order: {speed, acceleration}
const int joint_lims[stepper_servo_axes][2] = {                                           // joint limits for each axis, the only ones that are really updated are for axis 2 as of 5/3
  { -70, 40 },
  { -180, 180 }
};
const long PID_params[stepper_servo_axes][3] = {  // pid params
  { 8, 0, 0 },
  { 8, 0, 0 }
};
const float angle_offset_sign[stepper_servo_axes][2] = { { 25, 0.383386 }, { -333, 0.383386 } };  // angle offset for encoder input, first value is added and the second is multiplied by the encoder input

// class that combines steppers and encoders into unified servo object
class StepperServo {
public:
  AccelStepper *stepper_motor;

  bool is_running = true;  // whether or not stepper should currently move

  StepperServo(int inp_joint_num)  // init func.
  {

    joint_num = inp_joint_num;                                                                                   // internal joint_num var used to reference global variables
    stepper_motor = new AccelStepper(AccelStepper::DRIVER, motor_pins[joint_num][0], motor_pins[joint_num][1]);  // creates accel stepper object using stepper driver w/ two pins, {step, direction}

    // set stepper acceleration and speed params in steps/sec
    stepper_motor->setMaxSpeed(speed_accel_params[joint_num][0]);
    stepper_motor->setAcceleration(speed_accel_params[joint_num][1]);

    // creates pid object. PID object takes target angles, current encoder-read angles, and a unit-less output value to control stepper
    PID_controller_Object = new FastPID(PID_params[joint_num][0], PID_params[joint_num][1], PID_params[joint_num][2], 100, 16, true);

    check_current_angle();  // function that checks current angle from encoder, interprets it, and sets global current_angle var

    // initialization debug msgs
    Serial.print("joint ");
    Serial.print(joint_num);
    Serial.print(" init success (index starts at 0), current joint angle: ");
    Serial.print(actual_angles[joint_num]);
    Serial.print(" degrees. Moving to ");
    Serial.print(target_angles[joint_num]);
    Serial.println(" degrees");
  }

  void main() {

    if (check_runtime_joint(last_time, StepperServo_runtime_interval_microsecs[joint_num])) {
      check_current_angle();

      output_val = PID_controller_Object->step(target_angles[joint_num], actual_angles[joint_num]);

      /*
      if (joint_num == 1) {
        Serial.print("output: ");
        Serial.print(output_val);
        Serial.print(" actual: ");
        Serial.print(actual_angles[joint_num]);
        Serial.print(" target: ");
        Serial.println(target_angles[joint_num]);
      }
*/
      output_val = (joint_num == 0) ? -output_val : output_val;
      stepper_motor->move(output_val);  // moves to appropriate
                                        // again this code was copied from internet. my understanding is that if stepper should move and it's not paused, func adds stepper->move() command to a queue when arduino processor is free
                                        // this code works well from testing, steppers move quickly and smoothly but other funcs. are not bogged down, unlike previous blocking approach or others
    }
  }

private:
  FastPID *PID_controller_Object;

  int joint_num;
  long last_time = 0;
  float output_val;

  void check_current_angle()  // function reads encoder values and then converts them to angle sw params
  {
    // here is all the math that converts. the 664 is a constant/the max value of the encoder. this doesn't appear consistent from encoder to encoder so should maybe be turned into param

    float angle = ((float)(analogRead(joint_num + 1)) * angle_offset_sign[joint_num][1] + angle_offset_sign[joint_num][0]);
    // normalizes angle to +- 180 value
    if (angle > 180) {
      angle -= 360;
    } else if (angle < -180) {
      angle += 360;
    }

    actual_angles[joint_num] = angle;  // sets appropriate angle var to current angle
  }

  bool check_runtime_joint(long inp_last_time, int inp_runtime_interval_microsecs) {
    // checks if time since last runtime is enough for program to continue
    // if (global_time - inp_last_time > inp_runtime_interval_microsecs) {
    if (micros() - inp_last_time > inp_runtime_interval_microsecs) {

      return true;
    }
    return false;
  }
};

// create joint array
StepperServo *StepperServo_array[stepper_servo_axes];

// these two functions are used  convenience
inline bool check_runtime(long inp_last_time, int inp_runtime_interval_microsecs) {
  // checks if time since last runtime is enough for program to continue
  if (global_time - inp_last_time > inp_runtime_interval_microsecs) {
    return true;
  }
  return false;
}


void update_from_joystick() {
  static long last_time = 0;  // last time func. was run
  const int runtime_interval = 500;

  const int y_pin = 3;
  const int x_pin = 4;
  const int button_pin = 5;


  const float x_reduction_factor = 200;
  const float y_reduction_factor = x_reduction_factor;

  if (check_runtime(last_time, runtime_interval)) {
    last_time = global_time;

    int x_val = map(analogRead(x_pin), 0, 1023, -180, 180);
    int y_val = map(analogRead(y_pin), 0, 1023, -180, 180);

    int button_val = analogRead(button_pin);
    if (abs(x_val) < 10) { x_val = 0; }
    if (abs(y_val) < 10) { y_val = 0; }

    if (button_val > 200) {
      for (int i = 0; i < 2; i++) {
        target_angles[i] = 0;
        new_target_angles[i] = true;
      }
    } else {
      target_angles[0] += x_val / x_reduction_factor;
      target_angles[1] += y_val / y_reduction_factor;
      for (int i = 0; i < 2; i++) {
        new_target_angles[i] = true;
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("begin setup!");

  for (int i = 0; i < stepper_servo_axes; i++) {
    StepperServo_array[i] = new StepperServo(i);  // remember args
  }

  Serial.println("");
  Serial.println("end setup!");
}

void loop() {
  // basic running loop, not very interesting, callas all appropriate functions repeatedly
  global_time = micros();


  StepperServo_array[0]->main();
  StepperServo_array[1]->main();


  update_from_joystick();
  if (StepperServo_array[0]->stepper_motor->distanceToGo() != 0 && StepperServo_array[1]->stepper_motor->distanceToGo()) {
    do {
      StepperServo_array[0]->stepper_motor->run();
      StepperServo_array[1]->stepper_motor->run();
      yield();
    } while (StepperServo_array[0]->stepper_motor->distanceToGo() != 0 && StepperServo_array[1]->stepper_motor->distanceToGo());  // the is running var makes sure that the motor stops immediately when it should
  }
  //float angle = (float)(analogRead(joint_num)) * angle_offset_sign[joint_num][1] + angle_offset_sign[joint_num][0];

  /*
  Serial.print(((float)(analogRead(1)) * angle_offset_sign[1][1]) + angle_offset_sign[1][0]);
  Serial.print("   ");
  //Serial.println(((float)(analogRead(2)) * angle_offset_sign[2][1]) + angle_offset_sign[2][0]);
  Serial.println(((float)(analogRead(2)) * angle_offset_sign[2][1]) + angle_offset_sign[2][0]);


  Serial.print(actual_angles[0]);
  Serial.print("  ");
  Serial.println(actual_angles[1]);


*/
  Serial.print(target_angles[0]);
  Serial.print("  ");
  Serial.println(target_angles[1]);

}

//update_from_serial();
//manage_e_stop();

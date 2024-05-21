#include <Arduino.h>
#include <AccelStepper.h>
#include <FastPID.h>

// number of axes
//******IMPORTANT*********
const int stepper_servo_axes = 2;

// these global angle values manage program's target angle
float actual_angles[stepper_servo_axes];                        // current angles as measured form encoder
float target_angles[stepper_servo_axes] = { 0 };                // angles for each joint ot move to
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
const int StepperServo_runtime_interval_microsecs[stepper_servo_axes] = { 50000, 50000 };  // interval for joints to loop at. this is an array as opposed to one value because you might want joints that have to move quickly or make fine adjustments like a wrist mechanism to refresh faster, not sure if this is really necessary though

// global time var for syncing
long global_time;

// joint param
const int motor_pins[stepper_servo_axes][2] = {
  // pins motors are plugged into, order {step, direction}
  { 23, 25 },
  { 27, 29 },
};
const float microsteps_reduction_params[stepper_servo_axes][2] = {
  // microsteps and reduction of each gearbox, order {microsteps, reduction}. 1 degree of joint movement = motor_steps_per_rev * microsteps * gearbox_reduction / 360                                                                                                                             // gear stepper microsteps and gear reduction for each axes, order {microstep, reduction}
  { 8, 2.60869 },
  { 8, 2.60869 },
};
const long speed_accel_params[stepper_servo_axes][2] = { { 5000, 2500 }, { 5000, 2500 } };  // speed and acceleration params of motor, in steps/second (including microsteps), order: {speed, acceleration}
const int joint_lims[stepper_servo_axes][2] = {                                             // joint limits for each axis, the only ones that are really updated are for axis 2 as of 5/3
  { -180, 180 },
  { -180, 180 }
};
const long PID_params[stepper_servo_axes][3] = {  // pid params
  { 1, 0, 0 },
  { 1, 0, 0 }
};
const float angle_offset_sign[stepper_servo_axes][2] = { { -246, 0.383386 }, { -333, 1 } };  // angle offset for encoder input, first value is added and the second is multiplied by the encoder input

// class that combines steppers and encoders into unified servo object
class StepperServo {
public:
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
      Serial.print(output_val);
      Serial.print(" actual: ");
      Serial.print(actual_angles[joint_num]);
      Serial.print(" target: ");
      Serial.println(target_angles[joint_num]);
      */
      stepper_motor->move(output_val);  // moves to appropriate
      // again this code was copied from internet. my understanding is that if stepper should move and it's not paused, func adds stepper->move() command to a queue when arduino processor is free
      // this code works well from testing, steppers move quickly and smoothly but other funcs. are not bogged down, unlike previous blocking approach or others
      if (stepper_motor->distanceToGo() != 0 && is_running) {
        do {
          stepper_motor->run();
          yield();
        } while (stepper_motor->distanceToGo() != 0 && is_running);  // the is running var makes sure that the motor stops immediately when it should
      }
    }
  }

private:
  AccelStepper *stepper_motor;
  FastPID *PID_controller_Object;

  int joint_num;
  long last_time = 0;
  float output_val;

  void check_current_angle()  // function reads encoder values and then converts them to angle sw params
  {
    // here is all the math that converts. the 664 is a constant/the max value of the encoder. this doesn't appear consistent from encoder to encoder so should maybe be turned into param
    float angle = (float)(analogRead(joint_num)) * angle_offset_sign[joint_num][1] + angle_offset_sign[joint_num][0];

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

bool check_if_current_status(String condition)  // returns true if input condition is an acceptable status, then does status clearing actions,
{
  if (condition == global_status && new_global_status == true) {
    global_status = "";
    new_global_status = false;
    return true;
  }
  return false;
}

// next three functions parse serial input and put it in appropriate var
void update_from_serial() {
  static long last_time = 0;      // last time func. was run
  static bool first_char = true;  // whether or not the most recent character read is the first
  static bool status_update;      // whether or not parsed message is an updates status message or an updated angle message. this doesn't scale well if i want to add more io than just status and angles, should change. maybe use enum

  const char angle_init_char = 'A';   // unique first character in an input angle string
  const char status_init_char = 'S';  // unique first character in an input status string
  const char end_line_char = '\n';    // end character for all types of string

  static String parsed_string = "";  // variable that holds parsed string as it is read from serial

  if (check_runtime(last_time, serial_update_runtime_microsecs)) {
    // last_time =micros();
    if (Serial.available()) {
      // this section of code reads from serial input char by char
      // if char is a designated first character, it records which type of string its reading from serial
      // if char is an end line/msg char it terminates reading and passes string to appropriate handling function
      // if neither start nor ending char (ie part of msg body) char is appended to parsed_string. process continues until endline character. maybe should set maximum length to stop an improperly formatted input string from permanently halting program
      char read_char = Serial.read();
      Serial.println("bruh");
      if (first_char && read_char == status_init_char) {  // check if first char of status update msg
        status_update = true;
      } else if (first_char && read_char == angle_init_char) {  // check if first char of angle update msg
        status_update = false;
      } else if (read_char == end_line_char) {                                                                 // check if end msg char
        status_update ? updated_status_handler(parsed_string) : updated_target_angles_handler(parsed_string);  // call appropriate string handling function
        parsed_string = "";                                                                                    // clear input string
      } else {                                                                                                 // if not others, then its char partway through string so append to inp string
        parsed_string += read_char;
      }
    }
  }
}

void updated_target_angles_handler(String input_string) {
  // no vars need to be static because this function doesn't loop like update_from_serial(), it runs once sequentially

  const char deliminator_char = ',';  // char that delimitates between different angles in input angle string
  String parsed_angle = "";           // variable that holds parsed angle as it's read from string

  uint8_t angle_counter_coord = 0;  // variable that keeps track of which joint angle is currently being read

  for (char angle_char : input_string) {
    // this code goes through angle string and then adds to a substring
    // if char is deliminator char, string is converted to float and set equal to appropriate angle array, angle string is cleared, and joint angle is incremented by 1
    if (angle_char != deliminator_char) {
      parsed_angle += angle_char;  // append char to angle string
    } else {
      target_angles[angle_counter_coord] = atof(parsed_angle.c_str());  // angle string -> target angle float
      parsed_angle = "";                                                // clear angle string
      angle_counter_coord++;                                            // increment target joint by one
    }
  }

  if (angle_counter_coord != stepper_servo_axes - 1)  // this function handles last angle assignment if the string doesn't have an end deliminator char, ie "A10,10,10,10" instead of "A10,10,10,10,"
  {
    target_angles[angle_counter_coord] = atof(parsed_angle.c_str());
    parsed_angle = "";
  }

  for (int i = 0; i < stepper_servo_axes; i++)  // function sets the new target angle value to true
  {
    new_target_angles[i] = true;
  }
}

void updated_status_handler(String input_string) {

  const String acceptable_statuses[4] = { e_stop_status_msg, end_effector_open_status_msg, end_effector_close_status_msg };  // array of acceptable status msgs. maybe should use this array as a global var for scalability instead of multiple strings

  for (String status : acceptable_statuses)  // function checks if string is acceptable. if yes, string outputted and new_global_status flag set
  {
    if (status == input_string) {
      global_status = status;
      new_global_status = true;
      return;
    }
  }

  Serial.print("last status: '");  // if status is not in acceptable list, status is printed to serial
  Serial.print(input_string);
  Serial.println("' not recognized, disregarding");
}

void manage_e_stop() {
  // NOTE: e-stop permanently stops program so until microcontroller is reset.
  // functionally, its same as joint hold but only happens once/doesn't need to manage a toggle because once triggered program is done

  static long last_time = 0;         // last time func. was run
  const int e_stop_button = 15;      // button pin (analog pin) for e-stop button
  const int e_stop_threshold = 500;  // threshold for button to trigger e-stop

  if (check_runtime(last_time, e_stop_runtime_microsteps)) {
    /*last_time = global_time;*/ last_time = micros();
    if (/*(analogRead(e_stop_button) > e_stop_threshold) ||*/ check_if_current_status(e_stop_status_msg)) {
      // when able i should update electronics to allow for digital control of stepper driver enable pins for an actual power-off e stop

      for (int i = 0; i < stepper_servo_axes; i++)  // same joint stop function as estop
      {
        target_angles[i] = actual_angles[i];
        StepperServo_array[i]->is_running = false;
      }

      // printing debug/warning info
      Serial.println("/////////////// E STOP ACTIVATED ///////////////");
      Serial.print("arm stopped, angles: ");
      for (int i = 0; i < stepper_servo_axes; i++) {
        Serial.print(target_angles[i]);
        Serial.print(", ");
      }
      Serial.println("restart microcontroller to continue ");

      while (true)  // permanently stops program
      {
        delay(10);
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


  for (int i = 0; i < stepper_servo_axes; i++) {
    StepperServo_array[i]->main();
  }

  Serial.print(actual_angles[0]);
  Serial.print("   ");
  Serial.println(actual_angles[1]);


  update_from_serial();
  manage_e_stop();
}

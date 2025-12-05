#ifndef __MODE_H
#define __MODE_H

// the below 4 define is the FSM of the motor itself and we will get it from the motor feedback
#define _REST_MODE 0
#define _HALL_CALIBRATE 1
#define _MOTOR_MODE 2
#define _SET_ZERO 3

enum class Mode {
  REST, //echo + RESET mode for motor
  CONFIG, // only using as echo 
  SET_ZERO,
  HALL_CALIBRATE, // should be in config (echo) after enter calibration to wait for cal done
  MOTOR,
  CONTROL,
};

enum class MotorMode {
  REST,
  CONFIG,
  SET_ZERO,
  HALL_CALIBRATE,
  POSITION,
  VELOCITY,
  TORQUE,
};

enum class Behavior { SET_THETA, TCP_SLAVE, CUSTOM_1, CUSTOM_2, CUSTOM_3 };

#endif
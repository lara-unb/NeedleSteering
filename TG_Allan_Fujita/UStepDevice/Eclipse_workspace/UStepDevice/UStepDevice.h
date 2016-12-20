/*
 * UStepDevice.h
 *
 *  Created on: May 18, 2015
 *      Author: andre
 */

#ifndef USTEPDEVICE_H_
#define USTEPDEVICE_H_

#include "StepperMotor.h"
#include <pigpio.h>

class UStepDevice
{
 private:

  /*
   * DEVICE PARAMETERS
   */

  bool telescoping_support_mode_;

  // Actuators
  StepperMotor insertion_;
  StepperMotor rotation_;
  StepperMotor front_gripper_;
  StepperMotor back_gripper_;

  // Sensors
  unsigned emergency_button_;
  unsigned front_switch_;
  unsigned back_switch_;

  // Speed and acceleration parameters of the motors
  double min_constant_speed_;            // old min_base_speed_
  double max_constant_speed_;
  double base_speed_;                    // old max_base_speed_
  double max_ramp_speed_;                // old max_final_speed_
  double max_acceleration_;

  // Standard speed for opening/closing the gripper
  double gripper_default_speed_;

  // The default displacement of the front gripper (in revolutions) necessary to
  // move the gripper from the open position to the firmly grasping position
  double front_gripper_default_displacement_;

  // The default displacement of the back gripper (in revolutions) necessary to
  // move the gripper from the open position to the firmly grasping position
  double back_gripper_default_displacement_;

  // The time delay, the program should sleep for after performing any of the
  // gripper functions
  unsigned micros_gripper_delay_;

  // Insertion length position limits in millimeters
  // These positions correspond to the distance from the gripper box to the front limit switch
  double max_insertion_position_;
  double min_insertion_position_;

  // Starting position of the gripper box
  double initial_insertion_position_;

  // Standard speed for retreating the gripper box during the duty cycle step
  double default_retreating_speed_;

  // Standard speed for flipping the needle in 180 degrees
  double default_flipping_speed_;

  double gripper_angle_correction_speed_;
  double gripper_correction_angle_;
  double needle_correction_displacement_;

  // Duty cycle threshold parameters
  double dc_max_threshold_;
  double dc_min_threshold_;

  /*
   * INTERNAL VARIABLES
   */

  // State variables
  bool configured_;
  bool initialized_;
  bool calibrated_;
  unsigned duty_cycle_rotation_direction_;

  // Internal position estimation
  bool front_gripper_closed_;
  bool back_gripper_closed_;
  double insertion_position_;

  // Waves
  int wave_insertion_with_rotation_;
  int wave_pure_insertion_;
  int wave_half_rotation_;

  // Wave flags
  bool has_wave_pure_insertion_;
  bool has_wave_insertion_with_rotation_;
  bool has_wave_remaining_;

  // Duty cycle wave parameters
  unsigned num_dc_periods_;
  unsigned insertion_step_half_period_;
  unsigned rotation_step_half_period_;
  unsigned micros_remaining_;

  // Time parameters for the Bidirectional Duty Cycle
  unsigned seconds_rotation_;
  unsigned seconds_pure_insertion_;
  unsigned micros_rotation_;
  unsigned micros_pure_insertion_;

  // Time parameters for the Flipping Duty Cycle
  unsigned seconds_flipped_;
  unsigned seconds_unflipped_;
  unsigned seconds_half_rotation_;
  unsigned micros_flipped_;
  unsigned micros_unflipped_;
  unsigned micros_half_rotation_;

  // Feed back variables
  double performed_displacement_;                   // Analogous to calculated_insertion_depth_, but set inside the moveMotorConstantSpeed function
  double calculated_insertion_depth_;
  double calculated_minimum_insertion_depth_;
  double calculated_insertion_speed_;
  double calculated_rotation_speed_;
  double calculated_duty_cycle_;
  unsigned micros_real_rotation_duration_;
  double rotation_ramp_step_percentage_;

  /*
   * AUXILIARY FUNCTIONS
   */

  // DESCRIPTION PENDING
  void displayParameters();

  // Clear the wave variables, clear the wave flags and clear all the duty cycle parameters
  void clearWaves();

  // Combine all the wave flags in a single variable
  int checkExistingWaves();

  // Safety check
  int verifyMotorSpeedLimits(double motor_speed, bool allow_ramp);

  // Calculate all the duty cycle parameters, based on the requested speeds and duty cycle
  // OBS: The actual speeds and duty cycle may differ from the requested due to
  // truncation and the need of ramping the rotation motor. The performed
  // speeds and duty cycle are then saved to feedback variables.
  int calculateDutyCycleMotionParameters(double insert_motor_distance,  double insert_motor_speed, double rot_motor_speed, double duty_cycle);

  // Calculate the maximum rotation speed, if a frequency ramp is necessary.
  // Motor is considered to start at a base speed, ramp up until a 'maximum speed'
  // with constant acceleration and ramp down to base speed again, performing
  // one full rotation.
  // The 'maximum speed' is calculated as the lowest speed so that the motor can
  // perform an entire rotation within 'rot_insert_time_us' microseconds.
  int calculateRotationSpeed(unsigned max_rotation_time);

  // DESCRIPTION PENDING
  int calculateFeedbackInformation();

  int calculateFlippingDutyCycleTimes(unsigned total_insertion_steps, unsigned minimum_insertion_steps, unsigned total_rotation_steps, double duty_cycle);

  /*
   * WAVE GENERATION FUNCTIONS
   */

  // Generate a wave containing pulses for both the insertion and the rotation motor
  // In case of success the wave is saved to the member variable and the
  // corresponding flag is set.
  // In case of failure, the wave is not created and this functions return an error result
  int generateWaveInsertionWithRotation();

  // Generate a wave containing one step of the insertion motor
  int generateWavePureInsertion();

  // DESCRIPTION PENDING
  int generateWavesFlippingDutyCycle();

  // Build an array of pulses with constant speed
  // This function is called from the 'generateWavePureInsertion()' function
  gpioPulse_t* generatePulsesConstantSpeed(unsigned port_number, unsigned half_period, unsigned num_steps, unsigned total_time);

  // Build an array of pulses forming a motion profile that starts at a 'frequency_initial'
  // ramps up until 'frequency_final', maintain 'frequency_final' for some time and
  // decelerate back to 'frequency_initial'. Both ramps use the same 'step_acceleration'.
  // The final sequence of pulses contains 'num_steps' steps and lasts 'total_time' microseconds.
  // This function is called from the 'generateWaveInsertionWithRotation()' function
  gpioPulse_t* generatePulsesRampUpDown(unsigned port_number, double frequency_initial, double frequency_final, double step_acceleration, unsigned num_steps, unsigned total_time);

  // Build an array of pulses for ramping from 'frequency_initial' to 'frequency_final'
  // The pointer for the pulses must be provided and must have previously allocated enough
  // memory for storing  'max_steps' steps.
  // This function returns the number of pulses used in the ramp
  unsigned generatePulsesRampUp(unsigned port_number, double frequency_initial, double frequency_final, double step_acceleration, gpioPulse_t* pulses, unsigned max_steps);

  /*
   * PRIVATE MOTION FUNCTIONS
   */

  // DESCRIPTION PENDING
  int setEnable(unsigned char motor, unsigned enable);

  // DESCRIPTION PENDING
  int setDirection(unsigned char motor, unsigned direction);

  // DESCRIPTION PENDING
  int moveMotorConstantSpeed(unsigned char motor, double displacement, double speed);

  // DESCRIPTION PENDING
  int moveGripperToLimitSwitch(double speed);

  // DESCRIPTION PENDING
  int debugMoveMotorSteps(unsigned char motor, double motor_displacement_step, double motor_speed_step);

  // DESCRIPTION PENDING
  int prepareDutyCyleStep(double needle_insertion_depth);

  // Calculate the duty cycle motion parameters and create the motion waves
  // OBS: This function should be set as PRIVATE
  int setBidirectionalDutyCycle(double needle_insertion_depth,  double needle_insertion_speed, double needle_rotation_speed, double duty_cycle);

  // Send the duty cycle motion waves
  // OBS: This function should be set as PRIVATE
  int startBidirectionalDutyCycle();

  // Calculate the duty cycle motion parameters and create the motion waves
  // OBS: This function should be set as PRIVATE
  int setFlippingDutyCycle(double needle_insertion_depth,  double needle_insertion_speed, double minimum_insertion_depth, double duty_cycle);

  // Send the duty cycle motion waves
  // OBS: This function should be set as PRIVATE
  int startFlippingDutyCycle();

 public:

  // Empty constructor
  UStepDevice();

  /*
   * CONFIGURATION FUNCTIONS
   */

  // Read device parameters from file and set the member variables
  void configureMotorParameters();

  // Initialize the Raspberry Pi GPIO
  int initGPIO();

  // Terminate the Raspberry Pi GPIO
  void terminateGPIO();

  // DESCRIPTION PENDING
  int calibrateMotorsStartingPosition();

  /*
   * BASIC MOTION FUNCTIONS
   */

  // DESCRIPTION PENDING
  int openFrontGripper();

  // DESCRIPTION PENDING
  int closeFrontGripper();

  // DESCRIPTION PENDING
  int openBackGripper();

  // DESCRIPTION PENDING
  int closeBackGripper();

  // DESCRIPTION PENDING
  int rotateNeedle(double needle_revolutions, double needle_rotation_speed);

  // DESCRIPTION PENDING
  int translateFrontGripper(double front_gripper_displacement, double front_gripper_speed);

  /*
   * DUTY CYCLE STEP FUNCTIONS
   */

  // DESCRIPTION PENDING
  int performBidirectionalDutyCyleStep(double needle_insertion_depth,  double needle_insertion_speed, double needle_rotation_speed, double duty_cycle);

  // DESCRIPTION PENDING
  int performFlippingDutyCyleStep(double needle_insertion_depth,  double needle_insertion_speed, double minimum_insertion_depth, double duty_cycle);

  // DESCRIPTION PENDING
  int performBackwardStep(double needle_insertion_depth,  double needle_insertion_speed);

  // DEBUG FUNCTIONS
  int performFlippingDutyCyleStepPart1(double needle_insertion_depth,  double needle_insertion_speed, double minimum_insertion_depth, double duty_cycle);
  int performFlippingDutyCyleStepPart2(double needle_insertion_depth,  double needle_insertion_speed, double minimum_insertion_depth, double duty_cycle);
};

#endif /* USTEPDEVICE_H_ */

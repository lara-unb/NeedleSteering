/*
 * MyMotor.h
 *
 *  Created on: May 18, 2015
 *      Author: andre
 */

#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_

// Struct for holding all the motor configuration parameters
struct MotorParameters {
  unsigned port_enable;
  unsigned port_direction;
  unsigned port_step;
  unsigned steps_per_revolution;
  double gear_ratio;
} ;

class StepperMotor
{
 private:

  // Pinout of the interface board connecting to this motor
  unsigned port_enable_;
  unsigned port_direction_;
  unsigned port_step_;

  // Motor resolution configured through the DIP Switch in the STR2 driver
  unsigned steps_per_revolution_;

  // Mechanical constant that converts the motor revolutions into revolutions of
  // the end effector connected to the motor.
  // OBS: This parameter depends on the mechanical assembly of the device
  double gear_ratio_;

  // State variables
  bool configured_;
  bool initialized_;

 public:

  // Empty constructor
  StepperMotor();

  // Accessors
  unsigned port_enable();
  unsigned port_direction();
  unsigned port_step();
  unsigned steps_per_revolution();
  double gear_ratio();

  // Configuration functions
  void configureParameters(MotorParameters parameters);
  int initGPIO();

};

#endif /* STEPPERMOTOR_H_ */

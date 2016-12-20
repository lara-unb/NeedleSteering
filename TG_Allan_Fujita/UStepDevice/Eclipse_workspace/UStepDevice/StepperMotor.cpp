/*
 * MyMotor.cpp
 *
 *  Created on: May 18, 2015
 *      Author: andre
 */

#include "StepperMotor.h"
#include "debug.h"
#include <pigpio.h>

// Empty constructor: simply initialize all member variables with null values
StepperMotor::StepperMotor()
{
  port_enable_ = 0;
  port_direction_ = 0;
  port_step_ = 0;

  steps_per_revolution_ = 0;
  gear_ratio_ = 1.0;

  initialized_ = false;
  configured_ = false;
}

/*
 * ACCESSORS
 */

unsigned StepperMotor::port_enable()
{
  return port_enable_;
}

unsigned StepperMotor::port_direction()
{
  return port_direction_;
}

unsigned StepperMotor::port_step()
{
  return port_step_;
}

unsigned StepperMotor::steps_per_revolution()
{
  return steps_per_revolution_;
}

double StepperMotor::gear_ratio()
{
  return gear_ratio_;
}

// Copy the provided parameters into the corresponding member variables
void StepperMotor::configureParameters(MotorParameters parameters)
{
  port_enable_ = parameters.port_enable;
  port_direction_ = parameters.port_direction;
  port_step_ = parameters.port_step;

  steps_per_revolution_ = parameters.steps_per_revolution;

  gear_ratio_ = parameters.gear_ratio;

  configured_ = true;
}

// Initialize the GPIO ports of the Raspberry Pi connecting to this motor
// Simply set the ports as outputs and write their initial value to 0
int StepperMotor::initGPIO()
{
  // Check if the motor parameters have already been configured
  if(configured_)
  {
    // Set the motor output ports and write their initial value to 0
    gpioSetMode(port_enable_, PI_OUTPUT);
    gpioSetMode(port_direction_, PI_OUTPUT);
    gpioSetMode(port_step_, PI_OUTPUT);

    gpioWrite(port_enable_, 0);
    gpioWrite(port_direction_, 0);
    gpioWrite(port_step_, 0);

    initialized_ = true;
    return 0;
  }

  // If the motor parameters have not been set, return an error code
  else
  {
    Error("ERROR StepperMotor::initGPIO - Motor parameters not configured. You must call configureParameters() before \n");
    return ERR_MOTOR_NOT_CONFIGURED;
  }
}

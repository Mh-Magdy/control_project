/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi,
               double output_lim_mini) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/

  // Set the coefficients
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;

  // Set the output limits
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;

  // Initialize the errors
  error_p = 0.0;
  error_i = 0.0;
  error_d = 0.0;

  // Initialize the delta time
  delta_time = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   **/

  error_d = (delta_time > 0) ? ((cte - error_p) / delta_time) : 0.0;

  error_i += cte * delta_time;

  error_p = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   * The code should return a value in the interval [output_lim_mini,
   * output_lim_maxi]
   */
  double control = (Kp * error_p) + (Ki * error_i) + (Kd * error_d);
  return std::min(control, std::max(output_lim_max, output_lim_min));
}

double PID::UpdateDeltaTime(double new_delta_time) {
  /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
  return delta_time;
}
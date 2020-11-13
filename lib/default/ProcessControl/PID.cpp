/**
 * Copyright 2020 Marcos Tolentino
 *
 * based on PID.cpp by Colin Law, ajusted to use PID Arduino like algorithm.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * See Timeprop.h for Usage
 *
 **/


#include "PID.h"

PID::PID() {
  m_initialised = 0;
  m_last_sample_time = 0;
  m_last_pv_update_time = 0;
}

void PID::initialise( float setpoint,
    int max_interval, unsigned char mode_auto, double manual_op
    , double Kp, double Ki, double Kd
     ) {

  m_setpoint = setpoint;
  m_max_interval = max_interval;
  m_mode_auto= mode_auto;
  m_manual_op = manual_op;

  m_kp = Kp;
  m_ki = Ki;
  m_kd = Kd;

  m_initialised = 1;

}

double PID::tick( unsigned long nowSecs ) {
  double power;
  double factor;
  if (m_initialised && m_last_pv_update_time) {
    // we have been initialised and have been given a pv value
    // check whether too long has elapsed since pv was last updated
    if (m_max_interval > 0  &&  nowSecs - m_last_pv_update_time > m_max_interval) {
      // yes, too long has elapsed since last PV update so go to fallback power
      power = m_manual_op;
    } else {
      // is this the first time through here?
      if (m_last_sample_time) {
        // not first time
        unsigned long delta_t = nowSecs - m_last_sample_time;  // seconds
        if (delta_t <= 0 || delta_t > m_max_interval) {
          // too long since last sample so leave integral as is and set deriv to zero
          m_derivative = 0;
        } else {
          double error = m_setpoint - m_pv;
          double delta_v = m_pv - m_last_pv;

          // integral term
          m_integral   = m_ki * error * delta_t;
          m_power_sum += m_integral;

          //proportional = m_kp * error;
          proportional = -1.0 * m_kp * delta_v;
          m_power_sum += proportional;

          if (m_power_sum < 0.0) {
            m_power_sum = 0.0;
          } else if (m_power_sum > 1.0) {
            m_power_sum = 1.0;
          }

          m_derivative = -1.0 * m_kd * delta_v / delta_t;

          power = m_power_sum + m_derivative;
        }
      }

      //power = proportional + m_integral - m_derivative;
      // set power to disabled value if the loop is not enabled
      if (!m_mode_auto) {
        power = m_manual_op;
      }
    }
  } else {
    // not yet initialised or no pv value yet so set power to disabled value
    power = m_manual_op;
  }
  if (power < 0.0) {
    power = 0.0;
  } else if (power > 1.0) {
    power = 1.0;
  }
  m_last_sample_time = nowSecs;
  m_last_pv = m_pv;

  return power;
}

// call to pass in new process value
void PID::setPv( double pv, unsigned long nowSecs ){
  m_pv = pv;
  m_last_pv_update_time = nowSecs;
}

// methods to modify configuration data
void PID::setSp( double setpoint ) {
  m_setpoint = setpoint;
}

void PID::setAuto( unsigned char mode_auto ) {
  m_mode_auto = mode_auto;
}

void PID::setManualPower( double manual_op ) {
  m_manual_op = manual_op;
}

void PID::setMaxInterval( int max_interval ) {
  m_max_interval = max_interval;
}

void PID::setKp( double Kp ) {
  m_kp = Kp;
}

void PID::setKi( double Ki ) {
  m_ki = Ki;
}

void PID::setKd( double Kd ) {
  m_kd = Kd;
}

double PID::getPv() {
  return(m_pv);
}

double PID::getSp() {
  return(m_setpoint);
}

unsigned char PID::getAuto() {
  return(m_mode_auto);
}

double PID::getManualPower() {
  return(m_manual_op);
}

int PID::getMaxInterval() {
  return(m_max_interval);
}

double PID::getProportional() {
  return(proportional);
}

double PID::getIntegral() {
  return(m_integral);
}

double PID::getDerivative() {
  return(m_derivative);
}

double PID::getKp() {
  return(m_kp);
}

double PID::getKi() {
  return(m_ki);
}

double PID::getKd() {
  return(m_kd);
}

double PID::getPowerSum() {
  return(m_power_sum);
}

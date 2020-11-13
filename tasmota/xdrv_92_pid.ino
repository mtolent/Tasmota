/*
  xdrv_92_pid.ino - PID algorithm plugin for Sonoff-Tasmota
  Copyright (C) 2018 Colin Law and Thomas Herrmann
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * Code to
 *
 * Usage:
 * Place this file in the tasmota folder.
 * Clone the library https://github.com/colinl/process-control.git from Github
 * into a subfolder of lib.
 * If you want to use a time proportioned relay output with this then also get
 * xdrv_91_timeprop.ino
 * In user_config.h or user_config_override.h include code as follows:

 #define USE_PID         // include the pid feature (+4.3k)
   #define PID_SETPOINT                  19.5    // Setpoint value. This is the process value that the process is
                                                 // aiming for.
                                                 // May be adjusted via MQTT using cmnd pid_sp

   #define PID_MAX_INTERVAL              300     // This is the maximum time in seconds that is expected between samples.
                                                 // It is provided to cope with unusual situations such as a faulty sensor
                                                 // that might prevent the node from being supplied with a process value.
                                                 // If no new process value is received for this time then the power is set
                                                 // to the value defined for PID_MANUAL_POWER.
                                                 // May be adjusted via MQTT using cmnd pid_max_interval

   #define PID_AUTO                      1       // Auto mode 1 or 0 (for manual). This can be used to enable or disable
                                                 // the control (1=enable, auto mode, 0=disabled, manual mode). When in
                                                 // manual mode the output is set the value definded for PID_MANUAL_POWER
                                                 // May be adjusted via MQTT using cmnd pid_auto

   #define PID_MANUAL_POWER              0       // Power output when in manual mode or fallback mode if too long elapses
                                                 // between process values
                                                 // May be adjusted via MQTT using cmnd pid_manual_power

   #define PID_UPDATE_SECS               0       // How often to run the pid algorithm (integer secs) or 0 to run the algorithm
                                                 // each time a new pv value is received, for most applictions specify 0.
                                                 // Otherwise set this to a time
                                                 // that is short compared to the response of the process.  For example,
                                                 // something like 15 seconds may well be appropriate for a domestic room
                                                 // heating application.
                                                 // May be adjusted via MQTT using cmnd pid_update_secs

   #define PID_USE_TIMPROP               1       // To use an internal relay for a time proportioned output to drive the
                                                 // process, set this to indicate which timeprop output to use. For a device
                                                 // with just one relay then this will be 1.
                                                 // It is then also necessary to define USE_TIMEPROP and set the output up as
                                                 // explained in xdrv_91_timeprop.ino
                                                 // To disable this feature leave this undefined (undefined, not defined to nothing).

   #define PID_USE_LOCAL_SENSOR                  // if defined then the local sensor will be used for pv. Leave undefined if
                                                 // this is not required.  The rate that the sensor is read is defined by TELE_PERIOD
                                                 // If not using the sensor then you can supply process values via MQTT using
                                                 // cmnd pid_pv

   //#define PID_SHUTTER                   1     // if using the PID to control a 3-way valve, create Tasmota Shutter and define the 
                                                 // number of the shutter here. Otherwise leave this commented out

 * Baseed on https://github.com/colinl/Sonoff-Tasmota but changed to use Arduino PID aproach.
 * Help with using the PID algorithm and with loop tuning can be found at
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

 *
**/

#ifdef USE_PID

#include "PID.h"

#define D_CMND_PID "pid_"

#define D_CMND_PID_SETPV "pv"
#define D_CMND_PID_SETSETPOINT "sp"
#define D_CMND_PID_SETAUTO "auto"
#define D_CMND_PID_SETMANUAL_POWER "manual_power"
#define D_CMND_PID_SETMAX_INTERVAL "max_interval"
#define D_CMND_PID_SETUPDATE_SECS "update_secs"
#define D_CMND_PID_GET_STATUS "status"
#define D_CMND_PID_KP "kp"
#define D_CMND_PID_KI "ki"
#define D_CMND_PID_KD "kd"

enum PIDCommands { CMND_PID_SETPV, CMND_PID_SETSETPOINT, 
  CMND_PID_SETAUTO,
  CMND_PID_SETMANUAL_POWER, CMND_PID_SETMAX_INTERVAL, CMND_PID_SETUPDATE_SECS, CMND_PID_GET_STATUS
  , CMND_PID_KP, CMND_PID_KI, CMND_PID_KD 
   };
const char kPIDCommands[] PROGMEM = D_CMND_PID_SETPV "|" D_CMND_PID_SETSETPOINT "|" 
  D_CMND_PID_SETAUTO "|" D_CMND_PID_SETMANUAL_POWER "|" D_CMND_PID_SETMAX_INTERVAL "|" D_CMND_PID_SETUPDATE_SECS "|" D_CMND_PID_GET_STATUS
  "|" D_CMND_PID_KP "|" D_CMND_PID_KI "|" D_CMND_PID_KD;

static PID pid;
static int update_secs = PID_UPDATE_SECS <= 0  ?  0  :  PID_UPDATE_SECS;   // how often (secs) the pid alogorithm is run
static int max_interval = PID_MAX_INTERVAL;
static unsigned long last_pv_update_secs = 0;
static boolean run_pid_now = false;     // tells PID_Every_Second to run the pid algorithm

static long pid_current_time_secs = 0;  // a counter that counts seconds since initialisation

void PID_Init()
{
  snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID Init");
  AddLog(LOG_LEVEL_INFO);
  pid.initialise( 
    Settings.pid_sp ? Settings.pid_sp / 10.0 : PID_SETPOINT,
    PID_MAX_INTERVAL, PID_AUTO, PID_MANUAL_POWER,
    Settings.pid_kp ? Settings.pid_kp / 1000.0 : 1,
    Settings.pid_ki ? Settings.pid_ki / 1000.0 : 0.05, 
    Settings.pid_kd ? Settings.pid_kd / 1000.0 : 2 
     );
}

void PID_Every_Second() {
  static int sec_counter = 0;
  pid_current_time_secs++;    // increment time
  // run the pid algorithm if run_pid_now is true or if the right number of seconds has passed or if too long has
  // elapsed since last pv update. If too long has elapsed the the algorithm will deal with that.
  if (run_pid_now  ||  pid_current_time_secs - last_pv_update_secs > max_interval  ||  (update_secs != 0 && sec_counter++ % update_secs  ==  0)) {
    run_pid();
    run_pid_now = false;
  }
}

void PID_Show_Sensor() {
  // Called each time new sensor data available, data in mqtt data in same format
  // as published in tele/SENSOR
  // Update period is specified in TELE_PERIOD
  // e.g. "{"Time":"2018-03-13T16:48:05","DS18B20":{"Temperature":22.0},"TempUnit":"C"}"
  String jsonStr = TasmotaGlobal.mqtt_data;
  JsonParser parser((char*)jsonStr.c_str());
  JsonParserObject data_json = parser.getRootObject();
  JsonParserToken json_ds18b20_tk = data_json["DS18B20"];
  JsonParserObject json_ds18b20 = json_ds18b20_tk.getObject();
  if (json_ds18b20) {
    if (json_ds18b20.isValid()) {
      float value = json_ds18b20.getFloat("Temperature", 0);
      char tmpstr[sizeof(TasmotaGlobal.log_data)];
      // snprintf_P(log_data, sizeof(log_data), "PID_Show_Sensor: Temperature: %s", dtostrf(value, 1, 2, tmpstr));    // check that something was found and it contains a number
      // AddLog(LOG_LEVEL_INFO);
      // pass the value to the pid alogorithm to use as current pv
      last_pv_update_secs = pid_current_time_secs;
      // pid.setPv(atof(value), last_pv_update_secs);
      pid.setPv(value, last_pv_update_secs);
      // also trigger running the pid algorithm if we have been told to run it each pv sample
      if (update_secs == 0) {
        // this runs it at the next second
        run_pid_now = true;
      }
    } else {
      snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID_Show_Sensor - no temperature found");
      AddLog(LOG_LEVEL_INFO);
    }
  } else  {
    // parse failed
    snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID_Show_Sensor - json parse failed");
    AddLog(LOG_LEVEL_INFO);
  }
}


/* struct XDRVMAILBOX { */
/*   uint16_t      valid; */
/*   uint16_t      index; */
/*   uint16_t      data_len; */
/*   int16_t       payload; */
/*   char         *topic; */
/*   char         *data; */
/* } XdrvMailbox; */

boolean PID_Command()
{
  char command [CMDSZ];
  boolean serviced = true;
  uint8_t ua_prefix_len = strlen(D_CMND_PID); // to detect prefix of command

  snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "Command called: "
    "index: %d data_len: %d payload: %d topic: %s data: %s",
    XdrvMailbox.index,
    XdrvMailbox.data_len,
    XdrvMailbox.payload,
    (XdrvMailbox.payload >= 0 ? XdrvMailbox.topic : ""),
    (XdrvMailbox.data_len >= 0 ? XdrvMailbox.data : ""));
  AddLog(LOG_LEVEL_INFO);

  if (0 == strncasecmp_P(XdrvMailbox.topic, PSTR(D_CMND_PID), ua_prefix_len)) {
    // command starts with pid_
    int command_code = GetCommandCode(command, sizeof(command), XdrvMailbox.topic + ua_prefix_len, kPIDCommands);
    serviced = true;
    switch (command_code) {
      case CMND_PID_SETPV:
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID command setpv");
        AddLog(LOG_LEVEL_INFO);
        last_pv_update_secs = pid_current_time_secs;
        pid.setPv(atof(XdrvMailbox.data), last_pv_update_secs);
        // also trigger running the pid algorithm if we have been told to run it each pv sample
        if (update_secs == 0) {
          // this runs it at the next second
          run_pid_now = true;
        }
        break;

      case CMND_PID_SETSETPOINT:
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID command setsetpoint");
        AddLog(LOG_LEVEL_INFO);
        //save as int - saves space and trunc to 1 decimal place
        Settings.pid_sp = (int) (atof(XdrvMailbox.data) * 10.0); 
        pid.setSp(Settings.pid_sp / 10.0);
        break;

      case CMND_PID_SETAUTO:
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID command auto");
        AddLog(LOG_LEVEL_INFO);
        pid.setAuto(atoi(XdrvMailbox.data));
        break;

      case CMND_PID_SETMANUAL_POWER:
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID command manual power");
        AddLog(LOG_LEVEL_INFO);
        pid.setManualPower(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETMAX_INTERVAL:
      snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID command set max interval");
      AddLog(LOG_LEVEL_INFO);
      max_interval = atoi(XdrvMailbox.data);
      pid.setMaxInterval(max_interval);
      break;

      case CMND_PID_SETUPDATE_SECS:
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID command set update secs");
        AddLog(LOG_LEVEL_INFO);
        update_secs = atoi(XdrvMailbox.data) ;
        if (update_secs < 0) update_secs = 0;
        break;

      case CMND_PID_KP:
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID command set kp");
        AddLog(LOG_LEVEL_INFO);
        //save as int - saves space and trunc to 3 decimal place
        Settings.pid_kp = (int) (atof(XdrvMailbox.data) * 1000.0); 
        pid.setKp(Settings.pid_kp / 1000.0);
        break;
        
      case CMND_PID_KI:
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID command set ki");
        AddLog(LOG_LEVEL_INFO);
        //save as int - saves space and trunc to 3 decimal place
        Settings.pid_ki = (int) (atof(XdrvMailbox.data) * 1000.0); 
        pid.setKi(Settings.pid_ki / 1000.0);
        break;

      case CMND_PID_KD:
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID command set kd");
        AddLog(LOG_LEVEL_INFO);
        //save as int - saves space and trunc to 3 decimal place
        Settings.pid_kd = (int) (atof(XdrvMailbox.data) * 1000.0); 
        pid.setKd(Settings.pid_kd / 1000.0);
        break;

      char tmpstr[sizeof(TasmotaGlobal.log_data)];
      case CMND_PID_GET_STATUS:
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "PID settings:");
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  pid_pv:           %s", dtostrf(pid.getPv(), 1, 2, tmpstr));
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  pid_sp:           %s", dtostrf(pid.getSp(), 1, 2, tmpstr));
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  pid_kp:           %s", dtostrf(pid.getKp(), 1, 2, tmpstr));
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  pid_ki:           %s", dtostrf(pid.getKi(), 1, 2, tmpstr));
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  pid_kd:           %s", dtostrf(pid.getKd(), 1, 2, tmpstr));
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  pid_auto:         %d", pid.getAuto());
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  pid_manual_power: %s", dtostrf(pid.getManualPower(), 1, 2, tmpstr));
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  pid_max_interval: %d", pid.getMaxInterval());
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  pid_update_secs:  %d", update_secs);
        AddLog(LOG_LEVEL_INFO);        
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  Settings.pid_sp:  %d", Settings.pid_sp);
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  Settings.pid_kp:  %d", Settings.pid_kp);
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  Settings.pid_ki:  %d", Settings.pid_ki);
        AddLog(LOG_LEVEL_INFO);
        snprintf_P(TasmotaGlobal.log_data, sizeof(TasmotaGlobal.log_data), "  Settings.pid_kd:  %d", Settings.pid_kd);
        AddLog(LOG_LEVEL_INFO);
        break;

      default:
        serviced = false;
  }

    if (serviced) {
      // set mqtt RESULT
      snprintf_P(TasmotaGlobal.mqtt_data, sizeof(TasmotaGlobal.mqtt_data), PSTR("{\"%s\":\"%s\"}"), XdrvMailbox.topic, XdrvMailbox.data);
    }

  } else {
    serviced = false;
  }
  return serviced;
}

static void run_pid()
{
  double power = pid.tick(pid_current_time_secs);
  char tmpstr[20];
  char buffer[sizeof(TasmotaGlobal.mqtt_data)];
  snprintf_P(buffer, sizeof(buffer), "\"%s\":%s",               "power",            dtostrfd(power, 3, tmpstr));
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "pid_pv",           dtostrf(pid.getPv(), 1, 2, tmpstr));
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "pid_sp",           dtostrf(pid.getSp(), 1, 2, tmpstr));
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%d", buffer,   "pid_auto",         pid.getAuto());
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "pid_manual_power", dtostrf(pid.getManualPower(), 1, 2, tmpstr));
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%d", buffer,   "pid_max_interval", pid.getMaxInterval());
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%d", buffer,   "pid_update_secs",  update_secs);
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "pid_proportional", dtostrf(pid.getProportional(), 1, 2, tmpstr));
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "pid_integral",     dtostrf(pid.getIntegral(), 1, 2, tmpstr));
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "pid_derivative",   dtostrf(pid.getDerivative(), 1, 2, tmpstr));
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "pid_kp",           dtostrf(pid.getKp(), 1, 2, tmpstr));
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "pid_ki",           dtostrf(pid.getKi(), 1, 2, tmpstr));
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "pid_kd",           dtostrf(pid.getKd(), 1, 2, tmpstr));
  snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "pid_power_sum",    dtostrf(pid.getPowerSum(), 1, 2, tmpstr));
  //snprintf_P(buffer, sizeof(buffer), "%s, \"%s\":%s", buffer,   "settins_kd",       Settings.pid_kd);

  ResponseTime_P(PSTR(",\"PID\":{%s}}"), buffer);

  MqttPublishPrefixTopic_P(TELE, "PID", false);

#if defined PID_SHUTTER
    // send output as a position from 0-100 to defined shutter
    int pos = power * 100;
    ShutterSetPosition(PID_SHUTTER, pos);
#endif //PID_SHUTTER

#if defined PID_USE_TIMPROP
    // send power to appropriate timeprop output
    Timeprop_Set_Power( PID_USE_TIMPROP-1, power );
#endif // PID_USE_TIMPROP
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

#define XDRV_92       92

boolean Xdrv92(byte function)
//boolean XDRV_92(byte function)
{
  if (!Settings.flag5.enable_pid) {
     return false;
  }

  boolean result = false;

  switch (function) {
  case FUNC_INIT:
    PID_Init();
    break;
  case FUNC_EVERY_SECOND:
    PID_Every_Second();
    break;
  case FUNC_SHOW_SENSOR:
    // only use this if the pid loop is to use the local sensor for pv
    #if defined PID_USE_LOCAL_SENSOR
      PID_Show_Sensor();
    #endif // PID_USE_LOCAL_SENSOR
    break;
  case FUNC_COMMAND:
    result = PID_Command();
    break;
  }
  return result;
}

#endif // USE_TIMEPROP

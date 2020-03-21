/* Includes */
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Math.h>
#include <AP_ADC.h>
#include <StorageManager.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <GCS_MAVLink.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_Compass.h>
#include <AP_NavEKF.h>
#include <PID.h>

/* Defines */

/* Variables */
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  // Hardware abstraction layer

float map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() 
{

}

void loop() 
{
  // Local variables
  uint16_t channels[8];  // array for raw channel values
  float rcthr, rcyaw, rcpit, rcroll;   // Variables to store rc input

  // Read RC channels and store in channels array
  hal.rcin->read(channels, 8);

  // Copy from channels array to something human readable - array entry 0 = input 1, etc.
  rcthr = channels[2];
  rcyaw = channels[3];
  rcpit = channels[1];
  rcroll = channels[0];

  //Print to serial monitor
  hal.console->printf_P( PSTR("THR %.1f YAW %.1f PIT %.1f ROLL %.1f \n"), rcthr, rcyaw, rcpit, rcroll);

  hal.scheduler->delay(50);  //Wait 50ms 

}

AP_HAL_MAIN();    // special macro that replace's one of Arduino's to setup

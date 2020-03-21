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
AP_InertialSensor ins;  // Generic inertial sensor
AP_InertialSensor_MPU6000 imu(ins);  // MPU6000 sensor
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi); // Barometer
AP_GPS gps;  // GPS
AP_AHRS_DCM ahrs(ins, baro, gps); // AHRS DCM implementation 

/* Functions */

void setup() 
{
 
hal.uartA->begin(115200);// for using

  // we need to stop the barometer from holding the SPI bus
  hal.gpio->pinMode(40, HAL_GPIO_OUTPUT);
  hal.gpio->write(40, 1);

  // Initialize MPU6050 sensor
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

  // Initialize the accelerometer
  ins.init_accel();

  // Initialize the AHRS DCM implementation
  ahrs.init();

}

void loop() 
{
  Vector3f gyro;
  float gyroroll, gyropitch, gyroyaw;  // Velocity in deg/s
  float roll, pitch, yaw;  // Angle in degrees
  static uint16_t counter;
  static uint32_t last_print, last_compass;

  uint32_t now = hal.scheduler->micros();
  if (last_print == 0) {
    last_print = now;
    return;
  }
  // Wait for a new measurement
  ins.wait_for_sample();

  // Read the gyroscope (deg/s)
  ins.update();
  gyro = ins.get_gyro();
  gyropitch = ToDeg(gyro.y);
  gyroroll = ToDeg(gyro.x);
  gyroyaw = ToDeg(gyro.z);

  // Read the AHRS to get attitude (deg)
  ahrs.update();
  roll = ToDeg(ahrs.roll);
  pitch = ToDeg(ahrs.pitch);
  yaw = ToDeg(ahrs.yaw);

  // Print to serial monitor at 10Hz
  //counter++;
  if (now - last_print >= 100000 /* 100ms : 10hz */) {
  // hal.uartC->printf_P(PSTR("P:%4.1f  R:%4.1f Y:%4.1f rate=%.1f\n"),pitch,roll,yaw,(1.0e6*counter)/(now-last_print));
   hal.uartA->printf_P(PSTR("%2.1f\n "),pitch);
   //hal.uartE->printf_P(PSTR("P:%4.1f  R:%4.1f Y:%4.1f rate=%.1f\n"),pitch,roll,yaw,(1.0e6*counter)/(now-last_print));
  //  hal.uartD->printf_P(PSTR("P:%4.1f  R:%4.1f Y:%4.1f rate=%.1f\n"),pitch,roll,yaw,(1.0e6*counter)/(now-last_print));
    last_print = now;
    //counter = 0;
  }

}

AP_HAL_MAIN();    // special macro that replace's one of Arduino's to setup

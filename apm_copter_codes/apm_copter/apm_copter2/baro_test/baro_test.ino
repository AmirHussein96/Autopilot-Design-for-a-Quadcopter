/*
  generic Baro driver test
 */
 #include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <GCS_MAVLink.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
float x=0;
#define CONFIG_BARO HAL_BARO_DEFAULT
#if CONFIG_BARO == HAL_BARO_BMP085
static AP_Baro_BMP085 barometer;
hal.console->println("1");
#elif CONFIG_BARO == HAL_BARO_PX4
static AP_Baro_PX4 barometer;
hal.console->println("2");
#elif CONFIG_BARO == HAL_BARO_VRBRAIN
static AP_Baro_VRBRAIN barometer;
hal.console->println("3");
#elif CONFIG_BARO == HAL_BARO_HIL
static AP_Baro_HIL barometer;
hal.console->println("4");
#elif CONFIG_BARO == HAL_BARO_MS5611
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
hal.console->println("5");
#elif CONFIG_BARO == HAL_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);

#else
 #error Unrecognized CONFIG_BARO setting
#endif

static uint32_t timer;

void setup()
{
    hal.console->println("Barometer library test");

    hal.scheduler->delay(1000);

/*#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // disable CS on MPU6000
    hal.gpio->pinMode(63, HAL_GPIO_OUTPUT);
    hal.gpio->write(63, 1);
#endif*/

    barometer.init();
    barometer.calibrate();

    timer = hal.scheduler->micros();
}

void loop()
{get_baro
    if((hal.scheduler->micros() - timer) > 100000UL) {
        timer = hal.scheduler->micros();
        float sum=0;
        uint32_t read_time = hal.scheduler->micros() - timer;
        for (int i=0;i<100;i++){
           barometer.read();
           float alt = barometer.get_altitude();
          sum=sum+alt;
          
        }
        float avg= sum/100;
        
        if (!barometer.healthy()) {
            hal.console->println("not healthy");
            return;
        }
       /* hal.console->print("Pressure:");
        hal.console->print(barometer.get_pressure());
        hal.console->print(" Temperature:");
        hal.console->print(barometer.get_temperature());
        hal.console->print(" Altitude:");
        hal.console->print(avg);
        hal.console->printf(" climb=%.2f t=%u samples=%u",
                      barometer.get_climb_rate(),
                      (unsigned)read_time,
                      (unsigned)barometer.get_pressure_samples());
        hal.console->println();*/
        hal.console->printf_P(PSTR("%2.1f\n "),avg);
    }
}

AP_HAL_MAIN();


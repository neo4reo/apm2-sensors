/*
        APM2 Sensor Head Firmware
	Written by Curtis L. Olson, Airborne Technologies, Inc. colson@atiak.com

        Provides all the APM2 sensor data via binary checksummed data packets sent over the console
        port.  Additionally will accept binary checksummed data packets to command the servo positions
        in autopilot mode.
        
        Additional documentation to follow ...
*/

///////////////////////////////////////////
// Config section
///////////////////////////////////////////

// Firmware rev (needs to be updated manually)
#define FIRMWARE_REV 100

// Serial number (needs to be updated manually)
#define SERIAL_NUMBER 12345

// this is the hardware PWM generation rate
// note the default is 50hz and this is the max we can drive analog servos
// digital servos should be able to run at 200hz -- 250hz is getting up close to the theoretical maximum
// of a 100% duty cycle.  Advantage for running this at 200+hz with digital servos is we should catch commanded
// position changes slightly faster for a slightly more responsive system (emphasis on slightly)
// TODO: make this configurable via an external command.
#define PWM_OUTPUT_HZ 50

// this is the master loop update rate
#define MASTER_HZ 100

// starting communication baud
#define DEFAULT_BAUD 115200

///////////////////////////////////////////
// End of config section
///////////////////////////////////////////


///////////////////////////////////////////
// Hardware specific (APM2) config section
///////////////////////////////////////////

#define CONFIG_MPU6000_CHIP_SELECT_PIN 53

// pulled from AP_InertialSensor_MPU6000.cpp
#define MPU6000_GYRO_SCALE 0.00106421951219512195
#define MPU6000_ACCEL_SCALE 0.00239501953125
#define MPU6000_TEMP_SCALE 0.02

#define A_LED_PIN        27
#define B_LED_PIN        26
#define C_LED_PIN        25
#define LED_ON           LOW
# define LED_OFF          HIGH

///////////////////////////////////////////
// End Hardware specific config section
///////////////////////////////////////////


#include <Arduino_Mega_ISR_Registry.h>

// APM Library Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <I2C.h>
#include <SPI.h>
#include <APM_RC.h> // ArduPilot Mega RC Library
#include <AP_Math.h>
#include <AP_PeriodicProcess.h>  // ArduPilot Mega TimerProcess
#include <AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_InertialSensor.h> // Inertial Sensor (uncalibrated IMU) Library
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_IMU.h>         // ArduPilot Mega IMU Library
//#include <AP_AHRS.h>        // ArduPilot Mega AHRS Library
#include <Filter.h>			// Filter library
#include <ModeFilter.h>		// Mode Filter from Filter library
#include <LowPassFilter.h>	// LowPassFilter class (inherits from Filter class)

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess timer_scheduler;

FastSerialPort0(Serial);  // FTDI/Console
FastSerialPort1(Serial1); // GPS port
bool binary_output = false; // start with ascii output (then switch to binary if we get binary commands in

APM_RC_APM2 APM_RC;
uint16_t receiver_pos[NUM_CHANNELS];
uint16_t servo_pos[NUM_CHANNELS];

//#ifndef CONFIG_MPU6000_CHIP_SELECT_PIN
#  define CONFIG_MPU6000_CHIP_SELECT_PIN 53
//#endif
AP_InertialSensor_MPU6000 ins( CONFIG_MPU6000_CHIP_SELECT_PIN );
AP_IMU_INS imu( &ins );
#define MAX_IMU_SENSORS 7
float imu_sensors[MAX_IMU_SENSORS];

// FIXME: better analog support
#define MAX_ANALOG_INPUTS 5
uint16_t analog[MAX_ANALOG_INPUTS];

static uint32_t loop_timer = 0;
static uint32_t dt_millis = 1000 / MASTER_HZ;

// GPS
AP_GPS_MTK16      gps(&Serial1);

//AP_AHRS_Quaternion ahrs(&imu, g_gps);
  
void setup()
{
    I2c.begin();
    I2c.timeOut(5);
    // initially set a fast I2c speed, and drop it on first failures
    I2c.setSpeed(true);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16); // 1MHZ SPI rate

    isr_registry.init();
    timer_scheduler.init( &isr_registry );

    // we need to stop the barometer from holding the SPI bus
    pinMode(40, OUTPUT);
    digitalWrite(40, HIGH);

    APM_RC.Init(&isr_registry);	 // APM Radio initialization
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        APM_RC.enable_out(i);
        servo_pos[i] = 1500;
    }
    servo_pos[2] = 900; // (special case) throttle to minimum
    
    // set the PWM output rateas as defined above
    uint32_t ch_mask = _BV(CH_1) | _BV(CH_2) | _BV(CH_3) | _BV(CH_4) | _BV(CH_5) | _BV(CH_6) | _BV(CH_7) | _BV(CH_8);
    APM_RC.SetFastOutputChannels( ch_mask, PWM_OUTPUT_HZ );

    Serial.begin(DEFAULT_BAUD);
    Serial.println("APM2 Sensor Head");
    Serial.print("Firmware Revision: ");
    Serial.println(FIRMWARE_REV);
    Serial.print("Serial Number: ");
    Serial.println(SERIAL_NUMBER);
    delay(100);
    
    Serial.println("Initializing gyros ... please keep sensors motionless...");
    imu.init(IMU::COLD_START, delay, flash_leds, &timer_scheduler);
    Serial.println("done.");
    delay(100);

    Serial.println("Initializing MTK GPS...");
    // standard gps rate
    Serial1.begin(38400, 256, 16);
    gps.init();
    delay(200);
    
    //ahrs.reset();

    loop_timer = millis();
}

void loop()
{
    while ( millis() < loop_timer ); // busy wait for next frame
    loop_timer += dt_millis;
    
    // New radio frame?
    if ( APM_RC.GetState() == 1 ) {
        // read channel data
	for ( int i = 0; i < NUM_CHANNELS; i++ ) {
            receiver_pos[i] = APM_RC.InputCh(i);
	}
    }

    // suck in any host commmands    
    while ( read_commands_bin() );

    // update servos
    for ( int i = 0; i < NUM_CHANNELS - 1; i++ ) {
        if ( receiver_pos[CH_8] > 1500 ) {
          // manual pass through
          APM_RC.OutputCh(i, receiver_pos[i] );
        } else {
          // autonomous mode
          APM_RC.OutputCh(i, servo_pos[i] );
        }
    }
    APM_RC.OutputCh(CH_8, servo_pos[CH_8] );
    
    // IMU Update
    ins.update();
    ins.get_sensors(imu_sensors);
    // convert to "standard" coordinate system
    imu_sensors[0] *= -1.0;
    imu_sensors[1] *= -1.0;
    imu_sensors[3] *= -1.0;
    imu_sensors[4] *= -1.0;
    imu_sensors[6] = ins.temperature();

    // GPS Update
    gps.update();
    
    if ( binary_output ) {
        //write_pilot_in_bin();
        //write_imu_bin();
        //write_gps_bin();
    } else {
        // write_pilot_in_ascii();
        // write_imu_ascii();
        // write_gps_ascii();
    }
}

void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
    digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}


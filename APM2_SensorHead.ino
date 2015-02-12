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
///////////////////77//////////////////////

// Firmware rev (needs to be updated manually)
#define FIRMWARE_REV 210

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
//#define MASTER_HZ 200

// starting communication baud (16000000/16)/x where x is an integer produces the possible baud
// rates.  Note this is not the standard 300, 1200, 4800, 9600, 115,200, etc. series.  We can get
// close enough at the lower baud rates so both ends tolerate any slight discrepancy, but above
// 115,200 we diverge enough that it may not always work well end to end.  500,000 baud is a
// 1-to-1 match.
#define DEFAULT_BAUD 115200
//#define DEFAULT_BAUD 250000
//#define DEFAULT_BAUD 500000

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

#define PITOT_SOURCE_ANALOG_PIN 0
#define CURRENT1_ANALOG_PIN 1 /* A12 if power module port used */
#define VOLTAGE1_ANALOG_PIN 2 /* A13 if power module port used, but caution bug kills VCC sensing, fixed in later apm code. */
//#define CURRENT2_ANALOG_PIN 3
//#define VOLTAGE2_ANALOG_PIN 4
// ANALOG_PIN_VCC is defined in libraries/AP_AnalogSource/AP_Analog_Source_Arduino.h

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
#include <AP_AnalogSource.h>
#include <AP_AnalogSource_Arduino.h>
//#include <AP_AHRS.h>        // ArduPilot Mega AHRS Library
#include <Filter.h>			// Filter library
#include <ModeFilter.h>		// Mode Filter from Filter library
#include <LowPassFilter.h>	// LowPassFilter class (inherits from Filter class)
#include <AP_Airspeed.h>

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess timer_scheduler;

FastSerialPort0(Serial);  // FTDI/Console
FastSerialPort1(Serial1); // GPS port
//FastSerialPort2(Serial2); // Auxillary port
bool binary_output = false; // start with ascii output (then switch to binary if we get binary commands in

APM_RC_APM2 APM_RC;
int receiver_raw[NUM_CHANNELS];
int32_t receiver_norm[NUM_CHANNELS]; // normalized to fixed point [-10000, 10000] range.
int actuator_pos[NUM_CHANNELS];

#  define CONFIG_MPU6000_CHIP_SELECT_PIN 53
AP_InertialSensor_MPU6000 ins( CONFIG_MPU6000_CHIP_SELECT_PIN );
AP_IMU_INS imu( &ins );
#define MAX_IMU_SENSORS 7
float imu_sensors[MAX_IMU_SENSORS];
Vector3f imu_accel;
Vector3f imu_gyro;

AP_AnalogSource_Arduino pitot_source(PITOT_SOURCE_ANALOG_PIN);
AP_AnalogSource_Arduino current1_source(CURRENT1_ANALOG_PIN);
AP_AnalogSource_Arduino battery1_source(VOLTAGE1_ANALOG_PIN);
//AP_AnalogSource_Arduino current2_source(CURRENT2_ANALOG_PIN);
//AP_AnalogSource_Arduino battery2_source(VOLTAGE2_ANALOG_PIN);

#define MAX_ANALOG_INPUTS 6
uint16_t analog[MAX_ANALOG_INPUTS];

#if 0
// uncomment one of these
#define VOLT_DIV_RATIO     4.089   // This is the value I computed experimentally
// #define VOLT_DIV_RATIO     4.127   // This is the proper value for the AttoPilot 45A (13.6V) sensor
// #define VOLT_DIV_RATIO     15.70   // This is the proper value for the AttoPilot 50V/90A sensor

#define CURR_AMPS_OFFSET   0.0

// uncomment one of these
#define CURR_AMP_PER_VOLT     13.66   // This is the proper value for the AttoPilot 45A (13.6V) sensor
// #define CURR_AMP_PER_VOLT    27.32  // This is the proper value for the AttoPilot 50V/90A sensor
#endif

static uint32_t loop_timer = 0;
static uint32_t dt_millis = 1000 / MASTER_HZ;

// GPS (Enable the appropriate GPS)
static GPS *g_gps;
// AP_GPS_Auto g_gps_driver(&Serial1, &g_gps);
// AP_GPS_MTK16      g_gps_driver(&Serial1);
AP_GPS_UBLOX      g_gps_driver(&Serial1);

// Barometer
AP_Baro_MS5611 baro;

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

    // For a Futaba T6EX 2.4Ghz FASST system:
    //   Assuming all controls are at default center trim, no range scaling or endpoint adjustments:
    //   Minimum position = 1111
    //   Center position = 1525
    //   Max position = 1939
    
    APM_RC.Init(&isr_registry);	 // APM Radio initialization
    for ( int i = 0; i < NUM_CHANNELS; i++ ) {
        APM_RC.enable_out(i);
        actuator_pos[i] = 1500;
    }
    actuator_pos[2] = 1111;  // (special case) throttle to minimum.
    
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
    
    Serial.printf("F_CPU=%ld\n", F_CPU);
    uint16_t ubrr;
    uint32_t baud = 230400;
    ubrr = (F_CPU / 4 / baud - 1) / 2;
    Serial.printf("ubrr = %d\n", ubrr);
    
    Serial.println("Initializing MSC5611 Barometer...");
    baro.init(&timer_scheduler);
    baro.calibrate(delay);
 
    // we need to stop the barometer from holding the SPI bus during the calibration phase
    // pinMode(40, OUTPUT);
    // digitalWrite(40, HIGH);

    Serial.println("Initializing gyros ... please keep sensors motionless...");
    imu.init(IMU::COLD_START, delay, flash_leds, &timer_scheduler);
    Serial.println("done.");
    delay(100);

    Serial.println("Initializing GPS (Expecting ublox hardware) ...");
    // standard gps rate
    Serial1.begin(38400, 256, 16);
    g_gps = &g_gps_driver;
    g_gps->init(GPS::GPS_ENGINE_AIRBORNE_4G);
    delay(200);
    
    // Configure Analog inputs
    AP_AnalogSource_Arduino::init_timer(&timer_scheduler);
    pinMode(PITOT_SOURCE_ANALOG_PIN, INPUT);
    pinMode(CURRENT1_ANALOG_PIN, INPUT);
    pinMode(VOLTAGE1_ANALOG_PIN, INPUT);
    //pinMode(CURRENT2_ANALOG_PIN, INPUT);
    //pinMode(VOLTAGE2_ANALOG_PIN, INPUT);

    // prime the pump to avoid overflow in the "averaging" logic
    read_analogs();
    
    //Serial2.begin(DEFAULT_BAUD);
    //Serial2.println("APM2 Aux Port");
   
    loop_timer = millis();
}

void loop()
{
    while ( millis() < loop_timer ); // busy wait for next frame
    loop_timer += dt_millis;
    
    // Fetch new radio frame
    receiver_read();
    
    // suck in any host commmands    
    while ( read_commands() );

    // compute outputs (possibly with mixing)
    actuators_update();

    // IMU Update
    imu.update();
    imu_gyro = imu.get_gyro();
    imu_accel = imu.get_accel();
    
    // convert to "standard" coordinate system
    imu_sensors[0] = imu_gyro.x * -1.0;
    imu_sensors[1] = imu_gyro.y * -1.0;
    imu_sensors[2] = imu_gyro.z *  1.0;
    imu_sensors[3] = imu_accel.x * -1.0;
    imu_sensors[4] = imu_accel.y * -1.0;
    imu_sensors[5] = imu_accel.z *  1.0;
    imu_sensors[6] = ins.temperature();

    // GPS Update
    g_gps->update();
    
    // Barometer update
    baro.read();
    
    // Analog inputs
    read_analogs();
    
    if ( binary_output ) {
        write_pilot_in_bin();
        write_imu_bin();
        write_gps_bin();
        write_baro_bin();
        write_analog_bin();
    } else {
        write_pilot_in_ascii();
        // write_imu_ascii();
        // write_gps_ascii();
        // write_baro_ascii();
        // write_analog_ascii();
    }
}


void read_analogs() {
    // Analog inputs update (values are averages of the internal readings since the last read()
    // These are 10 bit values (0-1023) but floating point averages, so let's scale them up to the
    // full 16 bit range (0-65535) or multiple by 2^6 (64) so we can convey some precision of 
    // the "averaged" value.
    
    // special case (ugly) but if we put this in the global scope, then it hangs everything.  The
    // side effect here is that this isn't called until after setup() finishes.
    static AP_AnalogSource_Arduino vcc(ANALOG_PIN_VCC);
    
    analog[0] = (uint16_t)(pitot_source.read_average() * 64.0);
    analog[1] = (uint16_t)(battery1_source.read_average() * 64.0);
    analog[2] = (uint16_t)(current1_source.read_average() * 64.0);
    //analog[3] = (uint16_t)(battery2_source.read_average() * 64.0);
    //analog[4] = (uint16_t)(current2_source.read_average() * 64.0);
    analog[5] = vcc.read_vcc();

    /*    
    vcc_average = 0.99 * vcc_average + 0.01 * (analog[5] / 1000.0);
    battery_voltage = (analog[1]/64.0) * (vcc_average/1024.0) * VOLT_DIV_RATIO;
    battery_amps = (((analog[2]/64.0) * (vcc_average/1024.0)) - CURR_AMPS_OFFSET) * CURR_AMP_PER_VOLT * 10;
    amps_sum += battery_amps * dt_millis * 0.0002778; // .0002778 is 1/3600 (conversion to hours)
    */
}

void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
    digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}


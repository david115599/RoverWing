/*
* RoverWing firmware .h file
*/

//constants
#define REGA_SIZE 24 //registerA size in bytes
#define REGB_SIZE 48
//i2C slave address
#define SLAVE_ADDRESS 0x31 //49 binary
#define NO_ECHO 0xFFFF  //maximal possible uint16: 2^{16}-1


/*
 * pin definitions
 */
/* Analog pins */
uint16_t PINS_ANALOG[]={1,2,3,4,5,6}; //analog pins

// Sonar pins
uint16_t PINS_SONAR_ECHO[]={30,31,32};
uint16_t PINS_SONAR_TRIG[]={29,28,27};

/* Motor and Servo pins */

#define PIN_SERVO1           (7ul)
#define PIN_SERVO2           (8ul)
#define PIN_SERVO3           (9ul)
#define PIN_SERVO4           (10ul)

#define PIN_MOTOR1A          (19ul)
#define PIN_MOTOR1B          (20ul)
#define PIN_MOTOR2A          (23ul)
#define PIN_MOTOR1B          (24ul)

#define PIN_ENC1A            (17ul)
#define PIN_ENC1B            (18ul)
#define PIN_ENC2A            (22ul)
#define PIN_ENC2B            (21ul)


// Other pins
#define PIN_VSENSE           (0ul)


//registers
byte volatile REGA[REGA_SIZE];

/* Register A map

   These are registers that can be written to by the master
   All multibyte values are encoded using little endian convention: least significant byte first
   For many registers, there is also a pointer to the memory location of the register, giving an alternative way of accessing it.
   E.g., we can get the servo1 pulsewidth either using
    (REGA[REGA_SERVO1+1]<<8 | REGA[REGA_SERVO1])
    or using
    *p_servo1
   Note that each pointer has a corresponding type

   OFFSET NAME        |Byte   | Value                             | pointer name      |  type
   -------------------------------------------------------------------------------------
   REGA_ANALOG_CONFIG |  0     | Analog enabled bitmask            |  p_analogbitmask  | byte
                      |  1     | Not used                          |
   -------------------------------------------------------------------------------------
   REGA_SERVO1        |  2 - 3 | Servo1 pulsewidth, in us          |  p_servo1         | uint16
   REGA_SERVO2        |  4 -5  | Servo2 pulsewidth, in us          |  p_servo2         | uint16
   REGA_SERVO3        |  6 -7  | Servo3 pulsewidth, in us          |  p_servo3         | uint16
   REGA_SERVO4        |  8 - 9 | Servo4 pulsewidth, in us          |  p_servo4         | uint16
   -------------------------------------------------------------------------------------
   REGA_MOTOR_CONFIG  |  10    | Motors configuration              |                     byte
                      |  11    |                                   |                     byte
                      |  12    |                                   |                     byte
                      |  13    |                                   |                     byte
   -------------------------------------------------------------------------------------
   REGA_ENCODER_RESET |  14    | Encoder reset bitmask             |                     byte
                      |  15    | Not used
   -------------------------------------------------------------------------------------
   REGA_MOTOR1_SPEED  | 16-17  | Motor1 speed,  range -500...500   | p_motor1_speed     |int16
   REGA_MOTOR2_SPEED  | 18-19  | Motor2 speed,  range -500...500   | p_motor2_speed     |int16
   -------------------------------------------------------------------------------------
   REGA_SONARS_BITMASK|  20    | Sonar enabled bitmask             |  p_sonars_bitmask  |byte
                      |  21    | not used                          |
   REGA_SONARS_TIMEOUT|  22-23 | Sonar timeout, in us              | p_sonars_timeout  | uint16
   -------------------------------------------------------------------------------------

*/
#define REGA_ANALOG_CONFIG 0
#define REGA_SERVO1 2
#define REGA_SERVO2 4
#define REGA_SERVO3 6
#define REGA_SERVO4 8
#define REGA_MOTOR_CONFIG 10
#define REGA_ENCODER_RESET 14
#define REGA_MOTOR1_SPEED 16
#define REGA_MOTOR2_SPEED 18
#define REGA_SONARS_BITMASK 20
#define REGA_SONARS_TIMEOUT 22

volatile byte * p_analogbitmask = &REGA[REGA_ANALOG_CONFIG];
volatile uint16_t * p_servo1 = (* uint16_t) &REGA[REGA_SERVO1];
volatile uint16_t * p_servo2 = (* uint16_t) &REGA[REGA_SERVO2];
volatile uint16_t * p_servo3 = (* uint16_t) &REGA[REGA_SERVO3];
volatile uint16_t * p_servo4 = (* uint16_t) &REGA[REGA_SERVO4];
volatile int16_t * p_motor1_speed = (* int16_t) &REGA[REGA_MOTOR1_SPEED];
volatile int16_t * p_motor2_speed = (* int16_t) &REGA[REGA_MOTOR2_SPEED];
volatile byte * p_sonars_bitmask = (* byte) &REGA[REGA_SONARS_BITMASK];
volatile uint16_t * p_sonars_timeout = (* uint16_t) &REGA[REGA_SONARS_TIMEOUT];


byte volatile REGB[REGB_SIZE];
/* Register B map

   These are registers that can only be read by the master
   All multibyte values are encoded using little endian convention: least significant byte first
   For many registers, there is also a pointer to the memory location of the register, giving an alternative way of accessing it.

   Note that each pointer has a corresponding type

   OFFSET NAME        |Byte   | Value                             | pointer name      |  type
   REGB_VERSION       | 0-1   | Firmware  version                 | p_version         | uint16
                              |(byte0: major version; byte1:minor)|
   -------------------------------------------------------------------------------------
   REGB_ANALOG        |  2-3   | Analog1 read value (raw, 0-1023)  |  p_analog        | uint16
                      |  4-5   | Analog2 read value                |                  | uint16
                      |  6-7   | Analog3 read value                |                  | uint16
                      |  8-9   | Analog4 read value                |                  | uint16
                      |  10-11 | Analog5 read value                |                  | uint16
                      |  12-13 | Analog6 read value                |                  | uint16
   -------------------------------------------------------------------------------------
   REGB_ENC1          |  14-17 | Encoder1 read value               |  p_encoder1       | int32
   REGB_ENC2          |  18-21 | Encoder2 read value               |  p_encoder2       | int32
   -------------------------------------------------------------------------------------
   REGB_SONARS         |  22-23 | Sonar1 read value (time in us)   |  p_sonars          | uint16
                       |  24-25 | Sonar2 read value (time in us)   |                    | uint16
                       |  26-27 | Sonar3 read value (time in us)   |                    | uint16
   -------------------------------------------------------------------------------------
   REGB_GPSSTATUS      |  28    | GPS status                        | p_gpsstatus       |byte
                       |  29    | not used
   REGB_LATITUDE       |  30-33 | Latitude, in units of 10^{-7} deg | p_latitude        |int32
   REGB_LONGITUDE      |  34-37 | Longitude, in units of 10^{-7} deg| p_longitude       |int32
   REGB_TIMESTAMP      |  38-41 | Time acquired, in us after reset  | p_timestamp       |uint32
   -------------------------------------------------------------------------------------
   REGB_YAW            |  42-43 |  Yaw angle, in units of 0.1 deg   | p_yaw             |int16
   REGB_PITCH          |  44-45 |  Pitch angle, in units of 0.1 deg | p_pitch           |int16
   REGB_ROLL           |  46-47 |  Roll angle, in units of 0.1 deg  | p_roll            |int16
   -------------------------------------------------------------------------------------

*/
#define REGB_VERSION 0
#define REGB_ANALOG 2
#define REGB_ENC1 14
#define REGB_ENC2 16
#define REGB_SONARS 22
#define REGB_GPSSTATUS 28
#define REGB_LATITUDE 30
#define REGB_LONGITUDE 34
#define REGB_TIMESTAMP 38
#define REGB_YAW 42
#define REGB_PITCH 44
#define REGB_ROLL 46

volatile uint16_t * p_version = (* uint16_t) &REGB[REGB_VERSION];
volatile uint16_t * p_analog = (* uint16_t) &REGB[REGB_ANALOG];
volatile int32_t * p_encoder1 = (* int32_t) &REGB[REGB_ENC1];
volatile int32_t * p_encoder2 = (* int32_t) &REGB[REGB_ENC2];
volatile uint16_t * p_sonars = (* uint16_t) &REGB[REGB_SONARS];
volatile int32_t * p_latitude = (* int32_t) &REGB[REGB_LATITUDE];
volatile int32_t * p_longitude = (* int32_t) &REGB[REGB_LONGITUDE];
volatile uint32_t * p_timestamp = (* uint32_t) &REGB[REGB_TIMESTAMP];
volatile int16_t * p_yaw = (* int16_t) &REGB[REGB_YAW];
volatile int16_t * p_pitch = (* int16_t) &REGB[REGB_PITCH];
volatile int16_t * p_roll = (* int16_t) &REGB[REGB_ROLL];




//variables
//flags
volatile bool change_motors = false,
              change_servos = false,
              change_sonars = false;



//

/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

/* this file consists of several sections
 * to create a working combination you must at least make your choices in section 1.
 * 1 - BASIC SETUP - you must select an option in every block.
 *      this assumes you have 4 channels connected to your board with standard ESCs and servos.
 * 2 - COPTER TYPE SPECIFIC OPTIONS - you likely want to check for options for your copter type
 * 3 - RC SYSTEM SETUP
 * 4 - ALTERNATE CPUs & BOARDS - if you have
 * 5 - ALTERNATE SETUP - select alternate RX (SBUS, PPM, etc.), alternate ESC-range, etc. here
 * 6 - OPTIONAL FEATURES - enable nice to have features here (FlightModes, LCD, telemetry, battery monitor etc.)
 * 7 - TUNING & DEVELOPER - if you know what you are doing; you have been warned
 */

/* Notes:
 * 1. parameters marked with (*) in the comment are stored in eeprom and can be tweaked via serial monitor or LCD.
 *    Changing those values in config.h and upload will require a 'Reset' from the GUI to take effect
 */


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/**************************    The type of multicopter    ****************************/
//#define GIMBAL
//#define BI
//#define TRI
//#define QUADP
#define QUADX
//#define Y4
//#define Y6
//#define HEX6
//#define HEX6X
//#define OCTOX8
//#define OCTOFLATP
//#define OCTOFLATX
//#define FLYING_WING
//#define VTAIL4
//#define AIRPLANE
//#define SINGLECOPTER
//#define DUALCOPTER
//#define HELI_120_CCPM
//#define HELI_90_DEG

/****************************    Motor minthrottle    *******************************/
/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
   This is the minimum value that allow motors to run at a idle speed  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
//#define MINTHROTTLE 1064 // special ESC (simonk)
//#define MINTHROTTLE 1050 // for brushed ESCs like ladybird
#define MINTHROTTLE   (1048 + 22) // for low power case

/****************************    Motor maxthrottle    *******************************/
/* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
#define MAXTHROTTLE 1850

/****************************    Mincommand          *******************************/
/* this is the value for the ESCs when they are not armed
   in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
#define MINCOMMAND  1000

/**********************************    I2C speed   ************************************/
//#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

/***************************    Internal i2c Pullups   ********************************/
/* enable internal I2C pull ups (in most cases it is better to use external pullups) */
//#define INTERNAL_I2C_PULLUPS

/**************************************************************************************/
/*****************          boards and sensor definitions            ******************/
/**************************************************************************************/

/***************************    Combined IMU Boards    ********************************/
/* if you use a specific sensor board:
   please submit any correction to this list.
     Note from Alex: I only own some boards, for other boards, I'm not sure, the info was gathered via rc forums, be cautious */
//#define FFIMUv1         // first 9DOF+baro board from Jussi, with HMC5843                   <- confirmed by Alex
//#define FFIMUv2         // second version of 9DOF+baro board from Jussi, with HMC5883       <- confirmed by Alex
//#define FREEIMUv1       // v0.1 & v0.2 & v0.3 version of 9DOF board from Fabio
//#define FREEIMUv03      // FreeIMU v0.3 and v0.3.1
//#define FREEIMUv035     // FreeIMU v0.3.5 no baro
//#define FREEIMUv035_MS  // FreeIMU v0.3.5_MS                                                <- confirmed by Alex
//#define FREEIMUv035_BMP // FreeIMU v0.3.5_BMP
//#define FREEIMUv04      // FreeIMU v0.4 with MPU6050, HMC5883L, MS561101BA                  <- confirmed by Alex
//#define FREEIMUv043     // same as FREEIMUv04 with final MPU6050 (with the right ACC scale)
//#define NANOWII         // the smallest multiwii FC based on MPU6050 + pro micro based proc <- confirmed by Alex
//#define PIPO            // 9DOF board from erazz
//#define QUADRINO        // full FC board 9DOF+baro board from witespy  with BMP085 baro     <- confirmed by Alex
//#define QUADRINO_ZOOM   // full FC board 9DOF+baro board from witespy  second edition
//#define QUADRINO_ZOOM_MS// full FC board 9DOF+baro board from witespy  second edition       <- confirmed by Alex
//#define ALLINONE        // full FC board or standalone 9DOF+baro board from CSG_EU
//#define AEROQUADSHIELDv2
//#define ATAVRSBIN1      // Atmel 9DOF (Contribution by EOSBandi). requires 3.3V power.
//#define SIRIUS          // Sirius Navigator IMU                                             <- confirmed by Alex
//#define SIRIUSGPS       // Sirius Navigator IMU using external MAG on GPS board
//#define SIRIUS600       // Sirius Navigator IMU  using the WMP for the gyro
//#define MINIWII         // Jussi's MiniWii Flight Controller                                <- confirmed by Alex
//#define MICROWII        // MicroWii 10DOF with ATmega32u4, MPU6050, HMC5883L, MS561101BA from http://flyduino.net/
//#define CITRUSv2_1      // CITRUS from qcrc.ca
//#define CHERRY6DOFv1_0
//#define DROTEK_10DOF    // Drotek 10DOF with ITG3200, BMA180, HMC5883, BMP085, w or w/o LLC
//#define DROTEK_10DOF_MS // Drotek 10DOF with ITG3200, BMA180, HMC5883, MS5611, LLC
//#define DROTEK_6DOFv2   // Drotek 6DOF v2
//#define DROTEK_6DOF_MPU // Drotek 6DOF with MPU6050
//#define DROTEK_10DOF_MPU//
//#define MONGOOSE1_0     // mongoose 1.0    http://store.ckdevices.com/
//#define CRIUS_LITE      // Crius MultiWii Lite
//#define CRIUS_SE        // Crius MultiWii SE
//#define OPENLRSv2MULTI  // OpenLRS v2 Multi Rc Receiver board including ITG3205 and ADXL345
//#define BOARD_PROTO_1   // with MPU6050 + HMC5883L + MS baro
//#define BOARD_PROTO_2   // with MPU6050 + slave  MAG3110 + MS baro
//#define GY_80           // Chinese 10 DOF with  L3G4200D ADXL345 HMC5883L BMP085, LLC
//#define GY_85           // Chinese 9 DOF with  ITG3205 ADXL345 HMC5883L LLC
//#define GY_86           // Chinese 10 DOF with  MPU6050 HMC5883L MS5611, LLC
//#define GY_521          // Chinese 6  DOF with  MPU6050, LLC
//#define INNOVWORKS_10DOF // with ITG3200, BMA180, HMC5883, BMP085 available here http://www.diymulticopter.com
//#define INNOVWORKS_6DOF // with ITG3200, BMA180 available here http://www.diymulticopter.com
//#define MultiWiiMega    // MEGA + MPU6050+HMC5883L+MS5611 available here http://www.diymulticopter.com
//#define PROTO_DIY       // 10DOF mega board
//#define IOI_MINI_MULTIWII// www.bambucopter.com
//#define Bobs_6DOF_V1     // BobsQuads 6DOF V1 with ITG3200 & BMA180
//#define Bobs_9DOF_V1     // BobsQuads 9DOF V1 with ITG3200, BMA180 & HMC5883L
//#define Bobs_10DOF_BMP_V1 // BobsQuads 10DOF V1 with ITG3200, BMA180, HMC5883L & BMP180 - BMP180 is software compatible with BMP085
//#define FLYDUINO_MPU
//#define CRIUS_AIO_PRO_V1
//#define DESQUARED6DOFV2GO  // DEsquared V2 with ITG3200 only
//#define DESQUARED6DOFV4    // DEsquared V4 with MPU6050
//#define LADYBIRD
//#define MEGAWAP_V2_STD     // available here: http://www.multircshop.com                    <- confirmed by Alex
//#define MEGAWAP_V2_ADV
//#define HK_MultiWii_SE_V2  // Hobbyking board with MPU6050 + HMC5883L + BMP085
//#define HK_MultiWii_328P   // Also labeled "Hobbybro" on the back.  ITG3205 + BMA180 + BMP085 + NMC5583L + DSM2 Connector (Spektrum Satellite)



/***************************    independent sensors    ********************************/
/* leave it commented if you already checked a specific board above */
/* I2C gyroscope */
//#define WMP
//#define ITG3200
#define L3G4200D
//#define MPU6050       //combo + ACC

/* I2C accelerometer */
//#define NUNCHUCK  // if you want to use the nunckuk connected to a WMP
//#define MMA7455
#define ADXL345
#define ADXL345_ADDRESS 0x53
//#define BMA020
//#define BMA180
//#define NUNCHACK  // if you want to use the nunckuk as a standalone I2C ACC without WMP
//#define LIS3LV02
//#define LSM303DLx_ACC

/* I2C barometer */
#define BMP085
//#define MS561101BA

/* I2C magnetometer */
//#define HMC5843
#define HMC5883
//#define AK8975
//#define MAG3110

/* Sonar */ // for visualization purpose currently - no control code behind
//#define SRF02 // use the Devantech SRF i2c sensors
//#define SRF08
//#define SRF10
//#define SRF23

/* ADC accelerometer */ // for 5DOF from sparkfun, uses analog PIN A1/A2/A3
//#define ADCACC

/* individual sensor orientation */
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z; /* Why Z here?*/ }
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;} // confirmed
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}

//#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  = Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = Z;}

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  2 - COPTER TYPE SPECIFIC OPTIONS                               *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/*************************    Board orientation shift    ************************/
/* If you have frame designed only for + mode and you cannot rotate FC phisycally for flying in X mode (or vice versa)
   you can use one of of this options for virtual sensors rotation by 45 deegres, then set type of multicopter according to flight mode.
   Check motors order and directions of motors rotation for matching with new front point!  Uncomment only one option! */

//#define SENSORS_TILT_45DEG_RIGHT        // rotate the FRONT 45 degres clockwise
//#define SENSORS_TILT_45DEG_LEFT         // rotate the FRONT 45 degres counterclockwise

/********************************    TRI    *********************************/
#define YAW_DIRECTION 1
//#define YAW_DIRECTION -1 // if you want to reverse the yaw correction direction
/* you can change the tricopter servo travel here */
#define TRI_YAW_CONSTRAINT_MIN 1020
#define TRI_YAW_CONSTRAINT_MAX 2000
#define TRI_YAW_MIDDLE 1500 // (*) tail servo center pos. - use this for initial trim; later trim midpoint via LCD

/********************************    BI    *********************************/
/* you can change the bicopter servo travel direction here */
//#define BI_PITCH_DIRECTION 1
#define BI_PITCH_DIRECTION -1

/********************************    ARM/DISARM    *********************************/
/* optionally disable stick combinations to arm/disarm the motors.
  * In most cases one of the two options to arm/disarm via TX stick is sufficient */
//#define ALLOW_ARM_DISARM_VIA_TX_YAW
//#define ALLOW_ARM_DISARM_VIA_TX_ROLL

/***********************          Cam Stabilisation             ***********************/
/* The following lines apply only for a pitch/roll tilt stabilization system. Uncomment the first or second line to activate it */
//#define SERVO_MIX_TILT
//#define SERVO_TILT
#define TILT_PITCH_MIN    1020    //servo travel min, don't set it below 1020
#define TILT_PITCH_MAX    2000    //servo travel max, max value=2000
#define TILT_PITCH_MIDDLE 1500    //servo neutral value
#define TILT_PITCH_PROP   10      //servo proportional (tied to angle) ; can be negative to invert movement
#define TILT_PITCH_AUX_CH AUX3    //AUX channel to overwrite CAM pitch (AUX1-AUX4), comment to disable manual input and free the AUX channel
#define TILT_ROLL_MIN     1020
#define TILT_ROLL_MAX     2000
#define TILT_ROLL_MIDDLE  1500
#define TILT_ROLL_PROP    10
#define TILT_ROLL_AUX_CH  AUX4    //AUX channel to overwrite CAM Roll (AUX1-AUX4), comment to disable manual input and free the AUX channel

/* camera trigger function : activated via Rc Options in the GUI, servo output=A2 on promini */
//#define CAMTRIG
#define CAM_SERVO_HIGH 2000  // the position of HIGH state servo
#define CAM_SERVO_LOW 1020   // the position of LOW state servo
#define CAM_TIME_HIGH 1000   // the duration of HIGH state servo expressed in ms
#define CAM_TIME_LOW 1000    // the duration of LOW state servo expressed in ms

/***********************          Flying Wing                   ***********************/
/* you can change change servo orientation and servo min/max values here
   valid for all flight modes, even passThrough mode
   need to setup servo directions here; no need to swap servos amongst channels at rx */
#define PITCH_DIRECTION_L 1 // left servo - pitch orientation
#define PITCH_DIRECTION_R -1  // right servo - pitch orientation (opposite sign to PITCH_DIRECTION_L, if servos are mounted in mirrored orientation)
#define ROLL_DIRECTION_L 1 // left servo - roll orientation
#define ROLL_DIRECTION_R 1  // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)
#define WING_LEFT_MID  1500 // (*) left servo center pos. - use this for initial trim; later trim midpoint via LCD
#define WING_RIGHT_MID 1500 // (*) right servo center pos. - use this for initial trim; later trim midpoint via LCD
#define WING_LEFT_MIN  1020 // limit servo travel range must be inside [1020;2000]
#define WING_LEFT_MAX  2000 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MIN 1020 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MAX 2000 // limit servo travel range must be inside [1020;2000]

/***********************          Airplane                       ***********************/
//#define USE_THROTTLESERVO // For use of standard 50Hz servo on throttle.
#define SERVO_RATES      {100, 100, 100, 100, 100, 100, 100, 100} // Rates in 0-100%
#define SERVO_DIRECTION  { -1,   1,   1,   -1,  1,   1,   1,   1 } // Invert servos by setting -1

//#define FLAPPERONS    AUX4          // Mix Flaps with Aileroins.
#define FLAPPERON_EP   { 1500, 1700 } // Endpooints for flaps on a 2 way switch else set {1020,2000} and program in radio.
//#define FLAPPERON_EP   { 1200, 1500 } // Or Flapperons up for CrowMix
#define FLAPPERON_INVERT { 1, -1 }    // Change direction om flapperons { Wing1, Wing2 }

//#define FLAPS         AUX4          // Traditional Flaps on A2 invert with SERVO_DIRECTION servo[2).
#define FLAP_EP      { 1500, 1900 }   // Endpooints for flaps on a 2 way switch else set {1020,2000} and program in radio.

//#define FLAPSPEED     3             // Make flaps move slowm Higher value is Higher Speed.

/***********************      Common for Heli & Airplane         ***********************/
//#define D12_POWER      // Use D12 on PROMINI to power sensors. Will disable servo[4] on D12
#define SERVO_OFFSET     {  0,   0,   0,  0,   0,   0,  0,   0 } // (*) Adjust Servo MID Offset & Swash angles
// Selectable channels:=    ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4

/***********************          Heli                           ***********************/
/* Channel to control CollectivePitch */
#define COLLECTIVE_PITCH      THROTTLE
/* Set Maximum available movement for the servos. Depending on model */
#define SERVO_ENDPOINT_HIGH {2000,2000,2000,2000,2000,2000,2000,2000};
#define SERVO_ENDPOINT_LOW  {1020,1020,1020,1020,1020,1020,1020,1020};

/* Limit the range of Collective Pitch. 100% is Full Range each way and position for Zero Pitch */
#define COLLECTIVE_RANGE { 80, 0, 80 }// {Min%, ZeroPitch offset from 1500, Max%}.
#define YAW_CENTER             1500      // Use servo[5] SERVO_ENDPOINT_HIGH/LOW for the endpoits.
#define YAWMOTOR                 0       // If a motor is useed as YAW Set to 1 else set to 0.

/* Servo mixing for heli 120 Use 1/10 fractions (ex.5 = 5/10 = 1/2)
                     {Coll,Nick,Roll} */
#define SERVO_NICK   { +10, -10, -0 }
#define SERVO_LEFT   { +10, +5, +10 }
#define SERVO_RIGHT  { +10, +5, -10 }

/* Servo mixing for heli 90
                        {Coll,Nick,Roll} */
#define SERVO_DIRECTIONS { +1, -1, -1 } // -1 will invert servo

/* Limit Maximum controll for Roll & Nick  in 0-100% */
#define CONTROL_RANGE   { 100, 100 }      //  { ROLL,PITCH }

/* use servo code to drive the throttle output. You want this for analog servo driving the throttle on IC engines.
   if inactive, throttle output will be treated as a motor output, so it can drive an ESC */
//#define HELI_USE_SERVO_FOR_THROTTLE

/***********************      Single and DualCopter Settings     ***********************/
/* Change to -1 to reverse servomovement per axis
   Servosettings for SingleCopter */
#define SINGLECOPTRER_YAW   {1, 1, 1, 1} // Left, Right,Front,Rear
#define SINGLECOPTRER_SERVO {1,-1, 1,-1} // Pitch,Pitch,Roll, Roll    

/* Servosettings for DualCopter */
#define DUALCOPTER_SERVO {1,1} //Pitch,Roll
/* Use  SERVO_OFFSET and SERVO_RATES in Heli and Airplane section for centering and endpoints */

/***********************      your individual mixing     ***********************/
/* if you want to override an existing entry in the mixing table, you may want to avoid esditing the
 * mixTable() function for every version again and again. Then you must
 * 1) enable the correct copter type which resembles correct number of motors&servos
 * 2) create a file with your choice of name which contains all the mixing code for motors and servos.
 *    To get an idea, look at mixTable() function
 * 3) enable your mixing code with this define; replace filename with your chosen name
 *    (if you needed this info then probably this is not for you; start with an existing copter type and
 *    predefined mixing table entry)
 */


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  3 - RC SYSTEM SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/* note: no need to uncomment something in this section if you use a standard receiver */

/**************************************************************************************/
/********                       special receiver types             ********************/
/**************************************************************************************/

/****************************    PPM Sum Reciver    ***********************************/
/* The following lines apply only for specific receiver with only one PPM sum signal, on digital PIN 2
   Select the right line depending on your radio brand. Feel free to modify the order in your PPM order is different */
//#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
//#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Robe/Hitec/Futaba
//#define SERIAL_SUM_PPM         ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Multiplex
//#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For some Hitec/Sanwa/Others

// Uncommenting following line allow to connect PPM_SUM receiver to standard THROTTLE PIN on MEGA boards (eg. A8 in CRIUS AIO)
//#define PPM_ON_THROTTLE

/**********************    Spektrum Satellite Reciver    *******************************/
/* The following lines apply only for Spektrum Satellite Receiver
   Spektrum Satellites are 3V devices.  DO NOT connect to 5V!
   For MEGA boards, attach sat grey wire to RX1, pin 19. Sat black wire to ground. Sat orange wire to Mega board's 3.3V (or any other 3V to 3.3V source).
   For PROMINI, attach sat grey to RX0.  Attach sat black to ground. */
//#define SPEKTRUM 1024
//#define SPEKTRUM 2048
//#define SPEK_SERIAL_PORT 1    // Forced to 0 on Pro Mini and single serial boards; Set to your choice of 0, 1, or 2 on any Mega based board (defaults to 1 on Mega).
//**************************
// Defines that allow a "Bind" of a Spektrum or Compatible Remote Receiver (aka Satellite) via Configuration GUI.
//   Bind mode will be same as declared above, if your TX is capable.
//   Ground, Power, and Signal must come from three adjacent pins.
//   By default, these are Ground=4, Power=5, Signal=6.  These pins are in a row on most MultiWii shield boards. Pins can be overriden below.
//   Normally use 3.3V regulator is needed on the power pin!!  If your satellite hangs during bind (blinks, but won't complete bind with a solid light), go direct 5V on all pins.
//**************************
//   For Pro Mini, the connector for the Satellite that resides on the FTDI can be unplugged and moved to these three adjacent pins.
//#define SPEK_BIND             //Un-Comment for Spektrum Satellie Bind Support.  Code is ~420 bytes smaller without it.
//#define SPEK_BIND_GROUND 4
//#define SPEK_BIND_POWER  5
//#define SPEK_BIND_DATA   6

/*******************************    SBUS RECIVER    ************************************/
/* The following line apply only for Futaba S-Bus Receiver on MEGA boards at RX1 only (Serial 1).
   You have to invert the S-Bus-Serial Signal e.g. with a Hex-Inverter like IC SN74 LS 04 */
//#define SBUS

/******************* RC signal from the serial port via Multiwii Serial Protocol *********/
#define RCSERIAL

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  4 - ALTERNATE CPUs & BOARDS                                    *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/**************************************************************************************/
/********                      Promini Specifig Settings           ********************/
/**************************************************************************************/

/**************************    Hexa Motor 5 & 6 Pins    *******************************/
/* PIN A0 and A1 instead of PIN D5 & D6 for 6 motors config and promini config
   This mod allow the use of a standard receiver on a pro mini
   (no need to use a PPM sum receiver) */
//#define A0_A1_PIN_HEX

/*********************************    Aux 2 Pin     ***********************************/
/* possibility to use PIN8 or PIN12 as the AUX2 RC input (only one, not both)
   it deactivates in this case the POWER PIN (pin 12) or the BUZZER PIN (pin 8) */
//#define RCAUXPIN8
//#define RCAUXPIN12


/**************************************************************************************/
/*****************             Teensy 2.0 Support                    ******************/
/**************************************************************************************/
/* uncomment this if you use a teensy 2.0 with teensyduino
   it needs to run at 16MHz */
//#define TEENSY20


/**************************************************************************************/
/********   Settings for ProMicro, Leonardo and other Atmega32u4 Boards     ***********/
/**************************************************************************************/

/*********************************    pin Layout     **********************************/
/* activate this for a better pinlayout if all pins can be used => not possible on ProMicro */
//#define A32U4ALLPINS

/**********************************    PWM Setup     **********************************/
/* activate all 6 hardware PWM outputs Motor 5 = D11 and 6 = D13.
   note: not possible on the sparkfun promicro (pin 11 & 13 are not broken out there)
   if activated:
   Motor 1-6 = 10-bit hardware PWM
   Motor 7-8 = 8-bit Software PWM
   Servos    = 8-bit Software PWM
   if deactivated:
   Motor 1-4 = 10-bit hardware PWM
   Motor 5-8 = 10-bit Software PWM
   Servos    = 10-bit Software PWM */
//#define HWPWM6

/**********************************    Aux 2 Pin     **********************************/
/* AUX2 pin on pin RXO */
//#define RCAUX2PINRXO

/* aux2 pin on pin D17 (RXLED) */
//#define RCAUX2PIND17

/**********************************    Buzzer Pin    **********************************/
/* this moves the Buzzer pin from TXO to D8 for use with ppm sum or spectrum sat. RX (not needed if A32U4ALLPINS is active) */
#define D8BUZZER

/***********************      Promicro version related     ****************************/
/* Inverted status LED for Promicro ver 10 */
//#define PROMICRO10


/**************************************************************************************/
/********                      override default pin assignments    ********************/
/**************************************************************************************/

/* only enable any of this if you must change the default pin assignment, e.g. your board does not have a specific pin */
/* you may need to change PINx and PORTx plus shift # according to the desired pin! */
//#define V_BATPIN                   A3 // instead of A0    // Analog PIN 3
#define OVERRIDE_V_BATPIN            A6

//#define LEDPIN_PINMODE             pinMode (A1, OUTPUT); // use A1 instead of d13
//#define LEDPIN_TOGGLE              PINC |= 1<<1; // PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
//#define LEDPIN_OFF                 PORTC &= ~(1<<1); // PORTB &= ~(1<<5);
//#define LEDPIN_ON                  PORTC |= 1<<1;    // was PORTB |= (1<<5);

//#define BUZZERPIN_PINMODE          pinMode (A2, OUTPUT); // use A2 instead of d8
//#define BUZZERPIN_ON               PORTC |= 1<<2 //PORTB |= 1;
//#define BUZZERPIN_OFF              PORTC &= ~(1<<2); //PORTB &= ~1;

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  5 - ALTERNATE SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/******                Serial com speed    *********************************/
/* This is the speed of the serial interfaces */
#define SERIAL0_COM_SPEED 115200
#define SERIAL1_COM_SPEED 115200
#define SERIAL2_COM_SPEED 115200
#define SERIAL3_COM_SPEED 115200

/* interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config
   if the ACC calibration time is very long (20 or 30s), try to increase this delay up to 4000
   it is relevent only for a conf with NK */
#define INTERLEAVING_DELAY 3000

/* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
   it is relevent only for a conf with at least a WMP */
#define NEUTRALIZE_DELAY 100000


/**************************************************************************************/
/********                              Gyro filters                ********************/
/**************************************************************************************/

/*********************    Lowpass filter for some gyros    ****************************/
/* ITG3200 & ITG3205 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
   to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
   It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
   balancing options ran out. Uncomment only one option!
   IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
//#define ITG3200_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
//#define ITG3200_LPF_188HZ
//#define ITG3200_LPF_98HZ
//#define ITG3200_LPF_42HZ
//#define ITG3200_LPF_20HZ
//#define ITG3200_LPF_10HZ      // Use this only in extreme cases, rather change motors and/or props

/* MPU6050 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
   to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
   It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
   balancing options ran out. Uncomment only one option!
   IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
//#define MPU6050_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
//#define MPU6050_LPF_188HZ
//#define MPU6050_LPF_98HZ
//#define MPU6050_LPF_42HZ
//#define MPU6050_LPF_20HZ
//#define MPU6050_LPF_10HZ
//#define MPU6050_LPF_5HZ       // Use this only in extreme cases, rather change motors and/or props

/******                Gyro smoothing    **********************************/
/* GYRO_SMOOTHING. In case you cannot reduce vibrations _and_ _after_ you have tried the low pass filter options, you
   may try this gyro smoothing via averaging. Not suitable for multicopters!
   Good results for helicopter, airplanes and flying wings (foamies) with lots of vibrations.*/
//#define GYRO_SMOOTHING {20, 20, 3}    // (*) separate averaging ranges for roll, pitch, yaw

/************************    Moving Average Gyros    **********************************/
//#define MMGYRO 10                      // (*) Active Moving Average Function for Gyros
//#define MMGYROVECTORLENGTH 15          // Length of Moving Average Vector (maximum value for tunable MMGYRO
/* Moving Average ServoGimbal Signal Output */
//#define MMSERVOGIMBAL                  // Active Output Moving Average Function for Servos Gimbal
//#define MMSERVOGIMBALVECTORLENGHT 32   // Lenght of Moving Average Vector




/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  6 - OPTIONAL FEATURES                                          *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/* Pseudo-derivative conrtroller for level mode (experimental)
   Additional information: http://www.multiwii.com/forum/viewtopic.php?f=8&t=503 */
//#define LEVEL_PDF

/************************        AP FlightMode        **********************************/
/* Gyrocalibration will be repeated if copter is moving during calibration. */
//#define GYROCALIBRATIONFAILSAFE
/************************        AP FlightMode        **********************************/
/* Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks.*/
#define AP_MODE 40  // Create a deadspan for GPS.

/************************   Assisted AcroTrainer    ************************************/
/* Train Acro with auto recovery. Value set the point where ANGLE_MODE takes over.
   Remember to activate ANGLE_MODE first!...
   A Value on 200 will give a very distinct transfer */
//#define ACROTRAINER_MODE 200   // http://www.multiwii.com/forum/viewtopic.php?f=16&t=1944#p17437


/********                          Failsafe settings                 ********************/
/* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels)
   the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC or nunchuk is avaliable),
   PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THR0TTLE value. You must set this value to descending about 1m/s or so
   for best results. This value is depended from your configuration, AUW and some other params.  Next, afrer FAILSAFE_OFF_DELAY the copter is disarmed,
   and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */
#define FAILSAFE                                        // uncomment  to activate the failsafe function
#define FAILSAFE_DELAY          10                      // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAFE_OFF_DELAY      450                     // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAFE_THROTTLE       (MINTHROTTLE + 400)     // 1470 // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case
/*****************                Failsafe descending to the ground by BARO          *********************************/
#define FAILSAFE_VARIO          35                      // desceding speed, in cm/s      
#define FAILSAFE_SONAR_ALT      5
#define FAILSAFE_MAX_RISING_ALT 500


/*****************                Failsafe descending to the ground by BARO          *********************************/
/* For vario-controlled descending instead of having a fix throttle value, uncomment FAILSAFE_ALT_MODE. This allows to descend with preset vario.
   When flying on high altitudes, and would like to descend faster than apply FAILSAFE_SAFE_ALT and FAILSAFE_FAST_VARIO.
     This case will let copter descend faster from high altitude till reach the safety altitude, after will slow down to slow vario.
   If you have GPS and would like to have RTH activated when in failsafe, uncomment FAILSAFE_RTH_MODE. Please note that in this case FAILSAFE_ALT_MODE doesn't have to be uncommented,
     if GPS signal is weak, failsafe will work in FAILSAFE_ALT_MODE.
   Both with work only with FAILSAFE uncommented! Also Please note that both modes will work even if SUPPRESS_BARO_ALTHOLD is uncommented, so you can still save some space with better failsafe alt handling!

   Please note that FAILSAFE_OFF_DELAY is still active for security reasons, so set up long time enough to be able to RTH before this timer ends because motors will stop!!!*/

#define FAILSAFE_ALT_MODE             // uncomment for descending with constant vario if in Failsafe - use with FAILSAFE and define the FAILSAFE_SLOW_VARIO
#define FAILSAFE_SLOW_VARIO   30      // in cm/s - slow desceding speed under SAFETY_ALT, this is default if SAFETY_ALT is not used.  - maximum 250!!!
#define FAILSAFE_FAST_VARIO   100     // in cm/s - fast desceding speed over SAFETY_ALT, maximum 250!!!
#define FAILSAFE_SAFETY_ALT   200     // in cm   - safety altitude, where to slow down descending before landing, in cm!!!

#define FAILSAFE_RTH_MODE             // if GPS present and ready, copter starts RTH when signal lost. When signal is back, control is back again.
#define FAILSAFE_RTH_VARIO    70     // in cm/s - vario for RTH function for failsafe, maximum 250!!!
#define FAILSAFE_RTH_ALT      700    // in cm   - minimum RTH altitude for failsafe. If copter is higher than this, it will keep altitude.
#define FAILSAFE_RTH_HOME     300     // in cm   - home altitude for RTH, copter will descend to this altitude and wait.
#define FAILSAFE_RTH_DELAY    5      // in s    - safety delay, after reaching HOME altitude, it'll land in FAILSAFE_ALT_MODE when safety delay terminates.



/*****************                DFRobot LED RING    *********************************/
/* I2C DFRobot LED RING communication */
//#define LED_RING

/********************************    LED FLASHER    ***********************************/
//#define LED_FLASHER
//#define LED_FLASHER_DDR DDRB
//#define LED_FLASHER_PORT PORTB
//#define LED_FLASHER_BIT PORTB4
//#define LED_FLASHER_INVERT
//#define LED_FLASHER_SEQUENCE        0b00000000      // leds OFF
//#define LED_FLASHER_SEQUENCE_ARMED  0b00000101      // create double flashes
//#define LED_FLASHER_SEQUENCE_MAX    0b11111111      // full illumination
//#define LED_FLASHER_SEQUENCE_LOW    0b00000000      // no illumination


/*******************************    Landing lights    *********************************/
/* Landing lights
   Use an output pin to control landing lights.
   They can be switched automatically when used in conjunction
   with altitude data from a sonar unit. */
//#define LANDING_LIGHTS_DDR DDRC
//#define LANDING_LIGHTS_PORT PORTC
//#define LANDING_LIGHTS_BIT PORTC0
//#define LANDING_LIGHTS_INVERT

/* altitude above ground (in cm) as reported by sonar */
//#define LANDING_LIGHTS_AUTO_ALTITUDE 50

/* adopt the flasher pattern for landing light LEDs */
//#define LANDING_LIGHTS_ADOPT_LED_FLASHER_PATTERN

/*************************    INFLIGHT ACC Calibration    *****************************/
/* This will activate the ACC-Inflight calibration if unchecked */
//#define INFLIGHT_ACC_CALIBRATION

/**************************    Disable WMP power pin     *******************************/
/* disable use of the POWER PIN
   (allready done if the option RCAUXPIN12 is selected) */
//#define DISABLE_POWER_PIN

/*******************************    OSD Switch    *************************************/
// This adds a box that can be interpreted by OSD in activation status (to switch on/off the overlay for instance)
//#define OSD_SWITCH

/**************************************************************************************/
/***********************                  TX-related         **************************/
/**************************************************************************************/

/* introduce a deadband around the stick center
   Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw */
#define DEADBAND 25

/* defines the neutral zone of throttle stick during altitude hold, default setting is
   +/-20 uncommend and change the value below if you want to change it. */
#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE           50
#define ALT_HOLD_VARIO_MAX                       150                   // in cm/s  - maximum rising/descending vario when full throttle applied
#define ALT_HOLD_VARIO_P_CORR                    30   // For test and developer purposes only!!!  -  ALT-P correction during rising
#define ALT_HOLD_VARIO_I_CORR                    0    // For test and developer purposes only!!!  -  ALT-I correction during rising
#define ALT_HOLD_VARIO_D_CORR                    20   // For test and developer purposes only!!!  -  ALT-D correction during rising


/**************************************************************************************/
/***********************                  GPS                **************************/
/**************************************************************************************/

/* GPS using a SERIAL port
   if enabled, define here the Arduino Serial port number and the UART speed
   note: only the RX PIN is used in case of NMEA mode, the GPS is not configured by multiwii
   in NMEA mode the GPS must be configured to output GGA and RMC NMEA sentences (which is generally the default conf for most GPS devices)
   at least 5Hz update rate. uncomment the first line to select the GPS serial port of the arduino */
//#define GPS_SERIAL 2 // should be 2 for flyduino v2. It's the serial port number on arduino MEGA
//#define GPS_BAUD   57600
#define GPS_BAUD   115200


/* GPS protocol
    NMEA  - Standard NMEA protocol GGA, GSA and RMC  sentences are needed
    UBLOX - U-Blox binary protocol, use the ublox config file (u-blox-config.ublox.txt) from the source tree
    MTK_BINARY16 and MTK_BINARY19 - MTK3329 chipset based GPS with DIYDrones binary firmware (v1.6 or v1.9)
    With UBLOX and MTK_BINARY you don't have to use GPS_FILTERING in multiwii code !!! */


//#define NMEA
//#define UBLOX
//#define MTK_BINARY16
//#define MTK_BINARY19
//#define INIT_MTK_GPS        // initialize MTK GPS for using selected speed, 5Hz update rate and GGA & RMC sentence or binary settings

//#define GPS_PROMINI_SERIAL    57600 // Will Autosense if GPS is connected when ardu boots

/* I2C GPS device made with an independant arduino + GPS device
   including some navigation functions
   contribution from EOSBandi   http://code.google.com/p/i2c-gps-nav/
   You have to use at least I2CGpsNav code r33 */
#define I2C_GPS

/* I2C GPS device made with an indeedent ATTiny[24]313 + GPS device and
   optional sonar device.    https://github.com/wertarbyte/tiny-gps/ */
/* get GPS data from Tiny-GPS */
//#define TINY_GPS
/* get sonar data from Tiny-GPS */
//#define TINY_GPS_SONAR

/* GPS data readed from Misio-OSD - GPS module connected to OSD, and MultiWii read GPS data from OSD - tested and working OK ! */
//#define GPS_FROM_OSD

/* indicate a valid GPS fix with at least 5 satellites by flashing the LED  - Modified by MIS - Using stable LED (YELLOW on CRIUS AIO) led work as sat number indicator
  - No GPS FIX -> LED blink at speed of incoming GPS frames
  - Fix and sat no. bellow 5 -> LED off
  - Fix and sat no. >= 5 -> LED blinks, one blink for 5 sat, two blinks for 6 sat, three for 7 ... */
//#define GPS_LED_INDICATOR     // done on i2c nav

// Set/get waypoint
#define USE_MSP_WP                        //Enables the MSP_WP command, which is used by WinGUI to display and log Home and Poshold positions

//#define DONT_RESET_HOME_AT_ARM             // HOME position is reset at every arm, uncomment it to prohibit it (you can set home position with GyroCalibration)

/* GPS navigation can control the heading */

#define NAV_CONTROLS_HEADING       true      // copter faces toward the navigation point, maghold must be enabled for it
#define NAV_TAIL_FIRST             false     // true - copter comes in with tail first 
#define NAV_SET_TAKEOFF_HEADING    true      // true - when copter arrives to home position it rotates it's head to takeoff direction


/* Get your magnetic decliniation from here : http://magnetic-declination.com/
   Convert the degree+minutes into decimal degree by ==> degree+minutes*(1/60)
   Note the sign on declination it could be negative or positive (WEST or EAST) */
//#define MAG_DECLINIATION  3.96f              //For Budapest Hungary.
#define MAG_DECLINIATION      -5.18f       // For Ningbo, Zhejiang, China -5 deg 11 min -> -5.183333333333333

#define GPS_LEAD_FILTER                      // Adds a forward predictive filterig to compensate gps lag. Code based on Jason Short's lead filter implementation

//#define GPS_FILTERING                        // add a 5 element moving average filter to GPS coordinates, helps eliminate gps noise but adds latency comment out to disable
#define GPS_WP_RADIUS              200       // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
#define NAV_SLEW_RATE              30        // Adds a rate control to nav output, will smoothen out nav angle spikes


/**************************************************************************************/
/***********************        LCD/OLED - display settings       *********************/
/**************************************************************************************/

/* uncomment this line if you plan to use a LCD or OLED */
//#define LCD_CONF

/* to include setting the aux switches for AUX1 -> AUX4 via LCD */
//#define LCD_CONF_AUX

/* if program gets too large (>32k), need to exclude some functionality */
/* uncomment to suppress some unwanted aux3 aux4 items in config menu (only useful if LCD_CONF_AUX is enabled) */
//#define SUPPRESS_LCD_CONF_AUX34

/*****************************   The type of LCD     **********************************/
/* choice of LCD attached for configuration and telemetry, see notes below */
//#define LCD_DUMMY       // No Physical LCD attached.  With this & LCD_CONF defined, TX sticks still work to set gains, by watching LED blink.
//#define LCD_SERIAL3W    // Alex' initial variant with 3 wires, using rx-pin for transmission @9600 baud fixed
//#define LCD_TEXTSTAR    // SERIAL LCD: Cat's Whisker LCD_TEXTSTAR Module CW-LCD-02 (Which has 4 input keys for selecting menus)
//#define LCD_VT100       // SERIAL LCD: vt100 compatible terminal emulation (blueterm, putty, etc.)
//#define LCD_TTY         // SERIAL LCD: useful to tweak parameters over cable with arduino IDE 'serial monitor'
//#define LCD_ETPP        // I2C LCD: Eagle Tree Power Panel LCD, which is i2c (not serial)
//#define LCD_LCD03       // I2C LCD: LCD03, which is i2c
//#define OLED_I2C_128x64 // I2C LCD: OLED http://www.multiwii.com/forum/viewtopic.php?f=7&t=1350

/******************************   Display settings   ***********************************/
//#define LCD_SERIAL_PORT 0    // must be 0 on Pro Mini and single serial boards; Set to your choice on any Mega based board

//#define SUPPRESS_OLED_I2C_128x64LOGO  // suppress display of OLED logo to save memory

/* double font height for better readability. Reduces visible #lines by half.
 * The lower part of each page is accessible under the name of shifted keyboard letter :
 * 1 - ! , 2 - @ , 3 - # , 4 - $ , 5 - % , 6 - ^ , 7 - & , 8 - * , 9 - (
 * You must add both to your lcd.telemetry.* sequences
 */
//#define DISPLAY_FONT_DSIZE //currently only aplicable for OLED_I2C_128x64

/* style of display - AUTODETECTED via LCD_ setting - only activate to override defaults */
//#define DISPLAY_2LINES
//#define DISPLAY_MULTILINE
//#define MULTILINE_PRE 2  // multiline configMenu # pref lines
//#define MULTILINE_POST 6 // multiline configMenu # post lines
/********************************    Navigation     ***********************************/
/* keys to navigate the LCD menu */
#define LCD_MENU_PREV 'p'
#define LCD_MENU_NEXT 'n'
#define LCD_VALUE_UP 'u'
#define LCD_VALUE_DOWN 'd'

#define LCD_MENU_SAVE_EXIT 's'
#define LCD_MENU_ABORT 'x'

/* To use an LCD03 for configuration:
   http://www.robot-electronics.co.uk/htm/Lcd03tech.htm
   Remove the jumper on its back to set i2c control.
   VCC to +5V VCC (pin1 from top)
   SDA - Pin A4 Mini Pro - Pin 20 Mega (pin2 from top)
   SCL - Pin A5 Mini Pro - Pin 21 Mega (pin3 from top)
   GND to Ground (pin4 from top)*/

/* To use an Eagle Tree Power Panel LCD for configuration:
   White wire  to Ground
   Red wire    to +5V VCC (or to the WMP power pin, if you prefer to reset everything on the bus when WMP resets)
   Yellow wire to SDA - Pin A4 Mini Pro - Pin 20 Mega
   Brown wire  to SCL - Pin A5 Mini Pro - Pin 21 Mega */

/* Cat's whisker LCD_TEXTSTAR LCD
   Pleae note this display needs a full 4 wire connection to (+5V, Gnd, RXD, TXD )
   Configure display as follows: 115K baud, and TTL levels for RXD and TXD, terminal mode
   NO rx / tx line reconfiguration, use natural pins.
   The four buttons sending 'A', 'B', 'C', 'D' are supported for configuration navigation and request of telemetry pages 1-4 */


/**************************************************************************************/
/***********************                telemetry            **************************/
/**************************************************************************************/

/* to monitor system values (battery level, loop time etc. with LCD enable this
   note: for now you must send single characters to request  different pages
   Buttons toggle request for page on/off
   The active page on the LCD does get updated automatically
   Easy to use with Terminal application or display like LCD - if available uses the 4 preconfigured buttons  to send 'A', 'B', 'C', 'D' */
/********************************    Activation     ***********************************/
//#define LCD_TELEMETRY

/* to enable automatic hopping between a choice of telemetry pages uncomment this.
   This may be useful if your LCD has no buttons or the sending is broken
   hopping is activated and deactivated in unarmed mode with throttle=low & roll=left & pitch=forward
   set it to the sequence of telemetry pages you want to see
   2 line displays support pages 1-9
   multiline displays support pages 1-5 */
//#define LCD_TELEMETRY_AUTO "123452679" // pages 1 to 9 in ascending order
//#define LCD_TELEMETRY_AUTO  "212232425262729" // strong emphasis on page 2

/* same as above, but manual stepping sequence; requires
   stick input (throttle=low & roll=right & pitch=forward) to
   step through each defined telemetry page
   First page of the sequence gets loaded at startup to allow non-interactive display */
//#define LCD_TELEMETRY_STEP "0123456789" // should contain a 0 to allow switching off. First page of sequence gets loaded upon startup

/* if program gets too large (>32k), need to exclude some functionality
   uncomment to suppress some unwanted telemetry pages (only useful if telemetry is enabled) */
//#define SUPPRESS_TELEMETRY_PAGE_1
//#define SUPPRESS_TELEMETRY_PAGE_2
//#define SUPPRESS_TELEMETRY_PAGE_3
//#define SUPPRESS_TELEMETRY_PAGE_4
//#define SUPPRESS_TELEMETRY_PAGE_5
//#define SUPPRESS_TELEMETRY_PAGE_6
//#define SUPPRESS_TELEMETRY_PAGE_7
//#define SUPPRESS_TELEMETRY_PAGE_8
//#define SUPPRESS_TELEMETRY_PAGE_9

//#define RX_RSSI
//#define RX_RSSI_PIN A3

/********************************************************************/
/****                             Buzzer                         ****/
/********************************************************************/
#define BUZZER
#define RCOPTIONSBEEP         // uncomment this if you want the buzzer to beep at any rcOptions change on channel Aux1 to Aux4
//#define ARMEDTIMEWARNING 330  // (*) Trigger an alarm after a certain time of being armed [s] to save you lipo (if your TX does not have a countdown)
//#define PILOTLAMP             //Uncomment if you are using a X-Arcraft Pilot Lamp

/********************************************************************/
/****           battery voltage monitoring                       ****/
/********************************************************************/
/* for V BAT monitoring
   after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
   with R1=33k and R2=51k
   vbat = [0;1023]*16/VBATSCALE
   must be associated with #define BUZZER ! */
#define VBAT              // uncomment this line to activate the vbat code
// #define VBATSCALE     131 // change this value if readed Battery voltage is different than real voltage
// new-vbatscale = current-vbatscale * mutiwii-reading / voltmeter-reading
// this value stored in the eeprom, you will have to reset the values
#define VBATSCALE       128 // with R1=30k (30.1k) and R2=47k (46.6k)
#define VBATNOMINAL     126 // 12,6V full battery nominal voltage - only used for lcd.telemetry
#define VBATLEVEL_WARN1 107 // (*) 10,7V
#define VBATLEVEL_WARN2  99 // (*) 9.9V
#define VBATLEVEL_CRIT   93 // (*) 9.3V - critical condition: if vbat ever goes below this value, permanent alarm is triggered
#define NO_VBAT          16  // (*) Avoid beeping without any battery


/********************************************************************/
/****           powermeter (battery capacity monitoring)         ****/
/********************************************************************/

/* enable monitoring of the power consumption from battery (think of mAh)
   allows to set alarm value in GUI or via LCD
  Full description and howto here http://www.multiwii.com/wiki/index.php?title=Powermeter
   Two options:
   1 - hard: - (uses hardware sensor, after configuration gives very good results)
   2 - soft: - (good results +-5% for plush and mystery ESCs @ 2S and 3S, not good with SuperSimple ESC)    */
//#define POWERMETER_SOFT
//#define POWERMETER_HARD
/* PLEVELSCALE is the step size you can use to set alarm */
#define PLEVELSCALE 50 // if you change this value for other granularity, you must search for comments in code to change accordingly
/* larger PLEVELDIV will get you smaller value for power (mAh equivalent) */
#define PLEVELDIV 5000 // (*) default for soft - if you lower PLEVELDIV, beware of overrun in uint32 pMeter
#define PLEVELDIVSOFT PLEVELDIV // for soft always equal to PLEVELDIV; for hard set to 5000
#define PSENSORNULL 510 // (*) set to analogRead() value for zero current; for I=0A my sensor gives 1/2 Vss; that is approx 2.49Volt;
#define PINT2mA 13 // (*) for telemtry display: one integer step on arduino analog translates to mA (example 4.9 / 37 * 100

/********************************************************************/
/****           altitude hold                                    ****/
/********************************************************************/

/* uncomment to disable the altitude hold feature.
 * This is useful if all of the following apply
 * + you have a baro
 * + want altitude readout
 * + do not use altitude hold feature
 * + want to save memory space
 */
//#define SUPPRESS_BARO_ALTHOLD

/* Experimental altitude hold code. It uses "vario mode" which means reising/descending speed relates to the throttle stick position.
 In this code the complete altitude hold code runs on 25Hz.
 The RTH code uses defined altitude during approach.  */

/* Experimental altitude hold code. It uses "vario mode" which means reising/descending vario relates to the throttle stick position.
   When RTH_ALT_MODE is uncommented, in RTH mode copter rises to defined altitude, and when reaches the target, descends back to defined home altitude.
     By deffault, copter descends back to RTH altitude, if is higher. When RTH_KEEP_ALT is uncommented,
     it means copter will keep altitude if higher than RTH_ALT. Only descends when reached home.
   Please note that these modes can be used separated! (So for example, when only RTH_MODE is uncommented, it will return as described previously but for BARO mode
     natural alt change will be used.
   For working RTH_ALT_MODE Baro should be enabled with switch!
   If none of them are uncommented, natural alt change will be used for rapid pilots. It's temporary switch OFF the althold when throttle stick is
     out of deadband defined with ALT_HOLD_THROTTLE_NEUTRAL_ZONE 
*/

/********** user tunable parameters ************/
#define VARIO_ALT_MODE                          // define ALT HOLD code
//#define VARIO_MODE_CHANGE_BEEP                  // beep if changing between rising/descending  - long beep when entering hoover mode, short beep when changing between rising/descending
#define ALT_VARIO_MAX                     200   // in cm/s  - maximum rising/descending vario when full throttle applied  -  maximum 250!!!
#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE    50    // in us    - deadband of stick around hovering point when in ALT HOLD is active (us in PWM signal)
//#define ALT_HOLD_THROTTLE_MIDPOINT        1500  // in us    - if uncommented, this value is used in ALT_HOLD for throttle stick middle point instead of initialThrottleHold parameter.

#define RTH_ALT_MODE                            // define RTH custom approach height
#define RTH_KEEP_ALT                            // if the altitude is higher than the RTH_ALT, copter maintains that altitude instead of descending to target - according to Dramida's request
#define RTH_VARIO      100                      // in cm/s  - vario used for reaching target altitudes during RTH - maximum 250!!!
#define RTH_ALT        1000                     // in cm    - altitude during approach
#define HOME_ALT       400                      // in cm    - altitude after reaching home position

//#define WP_ALT_MODE                             // define WP mode - approach WP altitude with WP vario - USE WITH EZ-GUI and RTH_ALT_MODE
#define WP_VARIO       100                      // in cm/s  - vario used for reaching WP altitude - maximum 250!!!

#define ALT_SAFETY_DEADBAND      150            // deadband for RTH mode and WP mode. In emergency situations control can be get back with throttle stick


/********** Angle Correction on Throttle ************/
/* This feature calculates correction in throttle regarding to banking angle. Thanks to Alexmos!!!
 Please note that this feature is always active, NOT ONLY IN BARO MODE!!!
 For setup correctly, please follow this method:
  - First set the value to 100 and try it in ANGLE mode
  - Than raise the value 50 step by step, the goal is that when throttle is in hovering position and you start to move with the copter,
    it should stay in the same altitude. Please note that since the props will act like a wing when copter has speed, maybe value should
    reduced with 50-100. */

#define THROTTLE_ANGLE_CORRECTION   100

/********** Developer tunable parameters - PID correction for rising/descending ************/

/* Natural alt change for rapid pilots. It's temporary switch OFF the althold when throttle stick is out of deadband defined with ALT_HOLD_THROTTLE_NEUTRAL_ZONE
 * but if it's commented: Smooth alt change routine is activated, for slow auto and aerophoto modes (in general solution from alexmos). It's slowly increase/decrease
 * altitude proportional to stick movement (+/-100 throttle gives about +/-50 cm in 1 second with cycle time about 3-4ms)
 */
//#define ALTHOLD_FAST_THROTTLE_CHANGE

/********************************************************************/
/****           autoland function                                ****/
/********************************************************************/

/* This feature enables AUTOLAND BOXITEM.
 * Use together with RTH_ALT_MODE!
 * Works with valid GPS data only!
 * when AUTOLAND is active, RTH mode will be activated, so copter first move to home WP, then starts to descend
 * over SAFETY_ALT vario is the defined FAST_VARIO, under the SLOW_VARIO. This is for slowing down copter before land
 * the trigger for disarm is the BaroPID value. When copter can't descend anymore (touched the ground),
   the RPM will slow down until it reaches low BaroPID enough for DISARM.
 * Please note that for security reasons, SAFETY DEADBAND is applied, it throttle stick is out of deadband the automatic altitude control will be disabled
 */

#define AUTOLAND
#define AUTOLAND_FAST_VARIO       100  // vario over safety alt (cm/s)
#define AUTOLAND_SLOW_VARIO       25   // vario under safety alt (cm/s)
#define AUTOLAND_SAFETY_ALT       400  // safety altitude (cm)
#define AUTOLAND_SAFETY_DEADBAND  100  // deadband for AUTOLAND mode. See in description.


/********************************************************************/
/****           altitude variometer                              ****/
/********************************************************************/

/* enable to get audio feedback upon rising/falling copter/plane.
 * Requires a working baro.
 * For now, Output gets sent to an enabled vt100 terminal program over the serial line.
 * choice of two methods (enable either one or both)
 * method 1 : use short term movement from baro ( bigger code size)
 * method 2 : use long term observation of altitude from baro (smaller code size)
 */
//#define VARIOMETER 12            // possible values: 12 = methods 1 & 2 ; 1 = method 1 ; 2 = method 2
//#define SUPPRESS_VARIOMETER_UP   // if no signaling for up movement is desired
//#define SUPPRESS_VARIOMETER_DOWN // if no signaling for down movement is desired
//#define VARIOMETER_SINGLE_TONE   // use only one tone (BEL); neccessary for non-patched vt100 terminals

/********************************************************************/
/****           baord naming                                     ****/
/********************************************************************/

/*
 * this name is displayed together with the MultiWii version number
 * upon powerup on the LCD.
 * If you are without a DISPLAYD then You may enable LCD_TTY and
 * use arduino IDE's serial monitor to view the info.
 *
 * You must preserve the format of this string!
 * It must be 16 characters total,
 * The last 4 characters will be overwritten with the version number.
 */
#define BOARD_NAME "MultiWii   V-.--"
//                  123456789.123456

/*************      Support multiple configuration profiles in EEPROM     ************/
//#define MULTIPLE_CONFIGURATION_PROFILES

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  7 - TUNING & DEVELOPER                                  **************/
/*****************                                                                 ***************/
/*************************************************************************************************/

/************ Experimental: force a stable, fixated (high) cycle time       **********/
/* when activated, the displayed cycle time in GUI will not be correct.
 * Tunable via LCD config menu.
 * value of 0 turns the feature off.
 */
//#define CYCLETIME_FIXATED 9000 // (*)

/**************************************************************************************/
/********   special ESC with extended range [0-2000] microseconds  ********************/
/**************************************************************************************/
//#define EXT_MOTOR_RANGE

/**************************************************************************************/
/***********************     motor, servo and other presets     ***********************/
/**************************************************************************************/
/* motors will not spin when the throttle command is in low position
   this is an alternative method to stop immediately the motors */
#define MOTOR_STOP

/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500

/***********************         Servo Refreshrates            ***********************/
/* Default 50Hz Servo refresh rate*/
#define SERVO_RFR_50HZ

/* up to 160Hz servo refreshrate .. works with the most analog servos*/
//#define SERVO_RFR_160HZ

/* up to 300Hz refreshrate it is as fast as possible (100-300Hz depending on the cound of used servos and the servos state).
   for use with digital servos
   dont use it with analog servos! thay may get damage. (some will work but be careful) */
//#define SERVO_RFR_300HZ

/***********************             HW PWM Servos             ***********************/
/* HW PWM Servo outputs for Arduino Mega.. moves:
  Pitch   = pin 44
  Roll    = pin 45
  CamTrig = pin 46
  SERVO4  = pin 11 (assigned to PPM or SPECTRUM CH9 on copter configuration)
  SERVO5  = pin 12 (assigned to PPM or SPECTRUM CH10 on copter configuration)
  this option disable other software PWM's for servos - only five hardware controlled servos avaliable
  */
//#define MEGA_HW_PWM_SERVOS

/********************************************************************/
/****           Serial command handling - MSP and other          ****/
/********************************************************************/

/* to reduce memory footprint, it is possible to suppress handling of serial commands.
 * This does _not_ affect handling of RXserial, Spektrum or GPS. Those will not be affected and still work the same.
 * Enable either one or both of the following options  */

/* Remove handling of all commands of the New MultiWii Serial Protocol.
 * This will disable use of the GUI, winGUI, android apps and any other program that makes use of the MSP.
 * You must find another way (like LCD_CONF) to tune the parameters or live with the defaults.
 * If you run a LCD/OLED via i2c or serial/Bluetooth, this is safe to use */
//#define SUPPRESS_ALL_SERIAL_MSP // saves approx 2700 bytes

/* Remove handling of other serial commands.
 * This includes navigating via serial the lcd.configuration menu, lcd.telemetry and permanent.log .
 * Navigating via stick inputs on tx is not affected and will work the same.  */
//#define SUPPRESS_OTHER_SERIAL_COMMANDS // saves  approx 0 to 100 bytes, depending on features enabled

/********************************************************************/
/****           diagnostics                                      ****/
/********************************************************************/

/* to log values like max loop time and others to come
   logging values are visible via LCD config
   set to 1, enable 'R' option to reset values, max current, max altitude
   set to 2, adds min/max cycleTimes
   set to 3, adds additional powerconsumption on a per motor basis (this uses the big array and is a memory hog, if POWERMETER <> PM_SOFT) */
//#define LOG_VALUES 1

/* Permanent logging to eeprom - survives (most) upgrades and parameter resets.
 * used to track number of flights etc. over lifetime of controller board.
 * Writes to end of eeprom - should not conflict with stored parameters yet.
 * Logged values: accumulated lifetime, #powercycle/reset/initialize events, #arm events, #disarm events, last armedTime,
 *                #failsafe@disarm, #i2c_errs@disarm
 * To activate set to size of eeprom for your mcu: promini 328p: 1023 ; 2560: 4095.
 * Enable one or more options to show the log
 */
//#define LOG_PERMANENT 1023
//#define LOG_PERMANENT_SHOW_AT_STARTUP // enable to display log at startup
//#define LOG_PERMANENT_SHOW_AT_L // enable to display log when receiving 'L'
//#define LOG_PERMANENT_SHOW_AFTER_CONFIG // enable to display log after exiting LCD config menu
//#define LOG_PERMANENT_SERVICE_LIFETIME 36000 // in seconds; service alert at startup after 10 hours of armed time

/* to add debugging code
   not needed and not recommended for normal operation
   will add extra code that may slow down the main loop or make copter non-flyable */
#define DEBUG

/* Use this to trigger LCD configuration without a TX - only for debugging - do NOT fly with this activated */
//#define LCD_CONF_DEBUG

/* Use this to trigger telemetry without a TX - only for debugging - do NOT fly with this activated */
//#define LCD_TELEMETRY_DEBUG    //This form rolls between all screens, LCD_TELEMETRY_AUTO must also be defined.
//#define LCD_TELEMETRY_DEBUG 6  //This form stays on the screen specified.

/* Enable string transmissions from copter to GUI */
//#define DEBUGMSG


/********************************************************************/
/****           ESCs calibration                                 ****/
/********************************************************************/

/* to calibrate all ESCs connected to MWii at the same time (useful to avoid unplugging/re-plugging each ESC)
   Warning: this creates a special version of MultiWii Code
   You cannot fly with this special version. It is only to be used for calibrating ESCs
   Read How To at http://code.google.com/p/multiwii/wiki/ESCsCalibration */
//#define ESC_CALIB_LOW  MINCOMMAND
//#define ESC_CALIB_HIGH 2000
//#define ESC_CALIB_CANNOT_FLY  // uncomment to activate

/****           internal frequencies                             ****/
/* frequenies for rare cyclic actions in the main loop, depend on cycle time
   time base is main loop cycle time - a value of 6 means to trigger the action every 6th run through the main loop
   example: with cycle time of approx 3ms, do action every 6*3ms=18ms
   value must be [1; 65535] */
#define LCD_TELEMETRY_FREQ 23       // to send telemetry data over serial 23 <=> 60ms <=> 16Hz (only sending interlaced, so 8Hz update rate)
#define LCD_TELEMETRY_AUTO_FREQ 967 // to step to next telemetry page 967 <=> 3s
#define PSENSORFREQ 6               // to read hardware powermeter sensor 6 <=> 18ms
#define VBATFREQ PSENSORFREQ        // to read battery voltage - keep equal to PSENSORFREQ unless you know what you are doing

/********************************************************************/
/****           Sonar                                            ****/
/********************************************************************/
//#define HCSR04
#define I2C_SONAR

//mixing values between sonar and baro
#define SONAR_SONARFULL     (150+200) //full sonar reading before this point (cm)
#define SONAR_BAROFULL      (300+200) //full baro reading after this point (cm)
/* Maximum number of errors to switch to baro (integer 1..20) */
#define SONAR_ERROR_MAX     10

#define ADNS_5020           5020
#define ADNS_5050           5050
#define OF_DEBUG

#define I2C_OPTFLOW   
#define OPTFLOW 			1

//#define OPTFLOW             ADNS_5050

#define OF_YAW_DEADBAND 	40

#define OF_SCLK             A2
#define OF_SDIO             A1
#define OF_NCS              A0 

/* Lense focal distance, mm (set it for your own lense)
(How to check: debug4 in GUI should not react on ROLL tilt, but react on ROLL slide) */
#define OF_FOCAL_DIST 7
/* Deadband for ROLL,PITCH sticks where position hold is enabled. Max value 100 */
#define OF_DEADBAND 15
/* Rotate I-term with heading rotation. It will well compensate wind */
#define OF_ROTATE_I
/* Low-pass filter factor to prevent shaking. Possible values 1..8.  Default is 5. */
#define OF_LPF_FACTOR 5

/* Debug to GUI */

//ESC Refresh rate to 432Hz
//#define ESC_SKYWALKER_432HZ

#define RCSERIAL_RC_OFFLOAD

#define MAX_ANGLE_MODE_DEGREE   35

//#define CALC_GLOBAL_ACC  

/********************************************************************/
/****           Regression testing                               ****/
/********************************************************************/

/* for development only:
   to allow for easier and reproducable config sets for test compiling, different sets of config parameters are kept
   together. This is meant to help detecting compile time errors for various features in a coordinated way.
   It is not meant to produce your flying firmware
   To use:
   - do not set any options in config.h,
   - enable with #define COPTERTEST 1, then compile
   - if possible, check for the size
   - repeat with other values of 2, 3, 4 etc.
    */
//#define COPTERTEST 1

/*************************************************************************************************/
/****           END OF CONFIGURABLE PARAMETERS                                                ****/
/*************************************************************************************************/

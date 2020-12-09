#include <Wire.h>   
#include "Arduino.h"
#include<EEPROM.h>

#include <EnableInterrupt.h>
#include <Servo.h>

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_XOUT_H  0x04
#define AK8963_YOUT_L  0x05
#define AK8963_YOUT_H  0x06
#define AK8963_ZOUT_L  0x07
#define AK8963_ZOUT_H  0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#define MS5637_ADDRESS 0x76   // Address of altimeter
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#define MS5637_ADDRESS 0x76   // Address of altimeter
#endif 

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
  
// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13; // Set up pin 13 led for toggling

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magBias[3] = {0, 0, 0},magScale[3]={0,0,0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

#define Kp 2.0f * 5.0f // 5.1f*5.0f   //0.28f   //Kp 2.0f * 5.0f // Kp 5.1f * 5.0f
#define Ki 0.0f         //0.02f  //Ki=0.0f      //  Ki 1.0f 

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

//motor related
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR1_PIN 8
#define MOTOR2_PIN 9
#define MOTOR3_PIN 10
#define MOTOR4_PIN 11

Servo motor1,motor2,motor3,motor4;
float pwm_L_F,pwm_R_F,pwm_R_B,pwm_L_B;

//rc related
#define RC_NUM_CHANNELS 4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  A8
#define RC_CH2_INPUT  A9
#define RC_CH3_INPUT  A10
#define RC_CH4_INPUT  A11

uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];
uint16_t rc_values[RC_NUM_CHANNELS];

//pid related
float pid_roll_input=0;
float pid_pitch_input=0;
float pid_yaw_input=0;
float pid_throttle_input=0;


float pid_roll_kp=3.2;   //3.2
float pid_roll_ki=0.006; //0.006
float pid_roll_kd=1.4; //1.4

float pid_roll_error;
float pid_roll_previousError;
float pid_roll_cumError;
float pid_roll_rateError;

float pid_roll_p;
float pid_roll_i;
float pid_roll_d;
float pid_roll_output;


float pid_pitch_kp=0;
float pid_pitch_ki=0;
float pid_pitch_kd=0;

float pid_pitch_error;
float pid_pitch_previousError;
float pid_pitch_cumError;
float pid_pitch_rateError;

float pid_pitch_p;
float pid_pitch_i;
float pid_pitch_d;
float pid_pitch_output;

float pid_yaw_kp=0;
float pid_yaw_ki=0;
float pid_yaw_kd=0;

float pid_yaw_error;
float pid_yaw_previousError;
float pid_yaw_cumError;
float pid_yaw_rateError;

float pid_yaw_p;
float pid_yaw_i;
float pid_yaw_d;
float pid_yaw_output;

boolean pid_reset=true;


void setup()
{
  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
  delay(1000);
  Serial.println("1");
  delay(1000);
//  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);
  Serial.println("2");

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.println("5");
  Serial.print("c= ");Serial.println(c,HEX);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c,HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  Serial.println();
  delay(1000); 

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {  
    Serial.println("MPU9250 is online...");

    getAres();
    getGres();
    getMres();

    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1);
    
    uint8_t data[12];
    for(int i=0;i<12;i++)
    {
      data[i]=EEPROM.read(i);
    }
    // Push gyro biases to hardware registers
    writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
    // Push accelerometer biases to hardware registers
    writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[6]);
    writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[7]);
    writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[8]);
    writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[9]);
    writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[10]);
    writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[11]);
  
    initMPU9250(); 
    Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    Serial.println();
    
    
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    Serial.println();
    delay(1000); 
    
    initAK8963(magCalibration);Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
    Serial.println();
    
    
    if(SerialDebug)
    {
       //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
      Serial.println();
    }
    Serial.println();
    delay(1000); 

     magBias[0]=readFloat(24);
     magBias[1]=readFloat(28);
     magBias[2]=readFloat(32);
     
     magScale[0]=readFloat(36);
     magScale[1]=readFloat(40);
     magScale[2]=readFloat(44);
     
    Serial.println("AK8963 mag biases (mG)"); 
    Serial.print("magBias[0]= ");Serial.print(magBias[0]);Serial.print(",");Serial.print("magBias[1]= ");Serial.print(magBias[1]);Serial.print(",");Serial.print("magBias[2]= ");Serial.println(magBias[2]); 
    Serial.println();
    Serial.println("AK8963 mag scale (mG)");
    Serial.print("magScale[0]= ");Serial.print(magScale[0]);Serial.print(",");Serial.print("magScale[1]= ");Serial.print(magScale[1]);Serial.print(",");Serial.print("magScale[2]= "); Serial.println(magScale[2]); 
    delay(2000); // add delay to see results before serial spew of data 
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

   //rc related
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);

  //motor related
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  motor3.attach(MOTOR3_PIN);
  motor4.attach(MOTOR4_PIN);
  
}

void loop()
{  
  // If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
        
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes;//- accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes;//- accelBias[1];   
    az = (float)accelCount[2]*aRes;//- accelBias[2];  
   
    readGyroData(gyroCount);  // Read the x/y/z adc values
     
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;  
    gz = (float)gyroCount[2]*gRes;   
  
    readMagData(magCount);  // Read the x/y/z adc values
    
   // magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
  //  magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
  //  magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2]; 
    mx *= magScale[0];
    my *= magScale[1];
    mz *= magScale[2];   
  }
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f, my, -mx, mz,deltat);


    if (!AHRS)
    {
      delt_t = millis() - count;
      if(delt_t > 500)
      {

         if(SerialDebug) 
         {
            // Print acceleration values in milligs!
            Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg ");
            Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg ");
            Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg ");
 
            // Print gyro values in degree/sec
            Serial.print("X-gyro rate: "); Serial.print(gx, 3); Serial.print(" degrees/sec "); 
            Serial.print("Y-gyro rate: "); Serial.print(gy, 3); Serial.print(" degrees/sec "); 
            Serial.print("Z-gyro rate: "); Serial.print(gz, 3); Serial.println(" degrees/sec"); 
    
            // Print mag values in degree/sec
            Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG "); 
            Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG "); 
            Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG"); 
 
            tempCount = readTempData();  // Read the adc values
            temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
             // Print temperature in degrees Centigrade      
            Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
          }
    
   
          count = millis();
          digitalWrite(myLed, !digitalRead(myLed));  // toggle led
        }
     }
     else 
     {
        // Serial print and/or display at 0.5 s rate independent of data rates
       delt_t = millis() - count;
       if (delt_t > 500) 
       { 
         if(SerialDebug)
         {
            Serial.println();
            Serial.print("ax = "); Serial.print((int)1000*ax);  
            Serial.print(" ay = "); Serial.print((int)1000*ay); 
            Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
            Serial.print("gx = "); Serial.print( gx, 2); 
            Serial.print(" gy = "); Serial.print( gy, 2); 
            Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
            Serial.print("mx = "); Serial.print( (int)mx ); 
            Serial.print(" my = "); Serial.print( (int)my ); 
            Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
    
            Serial.print("q0 = "); Serial.print(q[0]);
            Serial.print(" qx = "); Serial.print(q[1]); 
            Serial.print(" qy = "); Serial.print(q[2]); 
            Serial.print(" qz = "); Serial.println(q[3]); 
          }              
    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
          
          yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
          pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
          roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
          pitch *= 180.0f / PI;
          yaw   *= 180.0f / PI; 
          yaw   -= 8.3; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
          roll  *= 180.0f / PI;

          //rc related
   
         rc_read_values();
    /*     Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
         Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
         Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
         Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]);   */

      //pid related
    //  pid_roll_input=(float)map(rc_values[RC_CH1],1000,2000,-180,180);
    //  pid_pitch_input=(float)map(rc_values[RC_CH2],1000,2000,180,180);
    //  pid_yaw_input=(float)map(rc_values[RC_CH4],1000,2000,-360,360);
    // pid_throttle_input=(float)rc_values[RC_CH3];

          drone_PID();

          motor_output();
        
          if(SerialDebug)
          {
            Serial.print("Yaw: ");Serial.print(yaw,2);Serial.print("     ,     ");Serial.print("pitch: ");Serial.print(pitch,2);Serial.print("     ,     ");Serial.print("roll: ");Serial.println(roll,2);
            Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");  

             // Serial.print("pid_roll_error = "); ;Serial.print("pid_roll_i = "); Serial.print("pid_roll_d = ");Serial.print("pid_roll_output = ");
        //  Serial.print("pid_roll_p = ");
           
         //     Serial.print(roll);
          //    Serial.print(" ");
         //   Serial.println(pid_roll_error);
          //  Serial.print(" ");
           // Serial.print(pid_roll_p);
         //  Serial.print(" ");
         //  Serial.print(pid_roll_i);
        //    Serial.print(" ");
        //   Serial.println(pid_roll_d);
         //   Serial.print(" ");
         //  Serial.println(pid_roll_output);
        //  Serial.print(pid_roll_i);
       //   Serial.print(" ");
       //   Serial.print(pid_roll_d);
      //    Serial.print(" ");
           
          
        //  Serial.print("pwm_R_F : ");Serial.println(pwm_R_F);
        //  Serial.print("pwm_R_B : ");Serial.println(pwm_R_B);
        //  Serial.print("pwm_L_F : ");Serial.println(pwm_L_F);
        //  Serial.print("pwm_L_B : ");Serial.println(pwm_L_B);


         // Serial.print("pid_pitch_error = "); Serial.println(pid_pitch_error);
         // Serial.print("pid_pitch_p = "); Serial.println(pid_pitch_p);
        //  Serial.print("pid_pitch_i = "); Serial.println(pid_pitch_i);
        //  Serial.print("pid_pitch_d = "); Serial.println(pid_pitch_d);
        //  Serial.print("pid_pitch_output = "); Serial.println( pid_pitch_output);
        //  Serial.print("pwm_L_F : ");Serial.println(pwm_L_F);
        //  Serial.print("pwm_R_F : ");Serial.println(pwm_R_F);
       //   Serial.print("pwm_L_B : ");Serial.println(pwm_L_B);
       //   Serial.print("pwm_R_B : ");Serial.println(pwm_R_B);

          //Serial.println();
          //Serial.print("pid_yaw_error = "); Serial.println(pid_yaw_error);
         // Serial.print("pid_yaw_p = "); Serial.println(pid_yaw_p);
        //  Serial.print("pid_yaw_i = "); Serial.println(pid_yaw_i);
        //  Serial.print("pid_yaw_d = "); Serial.println(pid_yaw_d);
        //  Serial.print("pid_yaw_output = "); Serial.println( pid_yaw_output);
        //  Serial.println();
        //  Serial.print("pwm_L_F : ");Serial.println(pwm_L_F);
        //  Serial.print("pwm_R_B : ");Serial.println(pwm_R_B);
         // Serial.print("pwm_R_F : ");Serial.println(pwm_R_F);
        //  Serial.print("pwm_L_B : ");Serial.println(pwm_L_B);
        //  Serial.println();
          
    
          }
   
          count = millis(); 
          sumCount = 0;
          sum = 0;    
      }
    }
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void motor_output(void)
{
  //pwm_L_F= pid_throttle_input - pid_pitch_output + pid_roll_output + pid_yaw_output;
  //pwm_R_F= pid_throttle_input - pid_pitch_output - pid_roll_output - pid_yaw_output;
  //pwm_R_B= pid_throttle_input + pid_pitch_output - pid_roll_output + pid_yaw_output;
  //pwm_L_B= pid_throttle_input + pid_pitch_output + pid_roll_output - pid_yaw_output;

    pwm_R_F= 115 + pid_throttle_input - pid_roll_output;
    pwm_R_B= 115 + pid_throttle_input - pid_roll_output;
    pwm_L_F= 115 + pid_throttle_input + pid_roll_output; 
    pwm_L_B= 115 + pid_throttle_input + pid_roll_output;
    
  //pwm_L_F= pid_throttle_input - pid_pitch_output;
  //pwm_R_F= pid_throttle_input - pid_pitch_output;
 // pwm_R_B= pid_throttle_input + pid_pitch_output;
 // pwm_L_B= pid_throttle_input + pid_pitch_output;

 // pwm_L_F= pid_throttle_input + pid_yaw_output;
 // pwm_R_B= pid_throttle_input + pid_yaw_output;
 // pwm_R_F= pid_throttle_input - pid_yaw_output;
 // pwm_L_B= pid_throttle_input - pid_yaw_output;

 if(pwm_L_F < 1100)
 {
   pwm_L_F=1100;
 }
 if(pwm_L_F>2000)
 {
   pwm_L_F=2000;
 }
 if(pwm_R_F<1100)
 {
   pwm_R_F=1100;
 }
 if(pwm_R_F>2000)
 {
   pwm_R_F=2000;
 }
 if(pwm_L_B<1100)
 {
   pwm_L_B=1100;
 }
 if(pwm_L_B>2000)
 {
   pwm_L_B=2000;
 }
 if(pwm_R_B<1100)
 {
   pwm_R_B=1100;
 }
 if(pwm_R_B>2000)
 {
   pwm_R_B=2000;
 }
 
  motor1.writeMicroseconds(pwm_L_F);  //Motor 1 (front left CW)
  motor2.writeMicroseconds(pwm_R_F);  //Motor 2 (front right,CCW)
  motor3.writeMicroseconds(pwm_R_B); //Motor 3 (bottom right, CW)
  motor4.writeMicroseconds(pwm_L_B); //Motor 4 (bottom left, CCW)

 
}


void drone_PID(void)
{
  if(pid_reset)
  {
    pid_roll_error=0;pid_roll_previousError=0;pid_roll_cumError=0;pid_roll_rateError=0;
    pid_roll_p=0;pid_roll_i=0;pid_roll_d=0;
    pid_roll_output=0;

    pid_pitch_error=0;pid_pitch_previousError=0;pid_pitch_cumError=0;pid_pitch_rateError=0;
    pid_pitch_p=0;pid_pitch_i=0;pid_pitch_d=0;
    pid_pitch_output=0;

    pid_yaw_error=0;pid_yaw_previousError=0;pid_yaw_cumError=0;pid_yaw_rateError=0;
    pid_yaw_p=0;pid_yaw_i=0;pid_yaw_d=0;
    pid_yaw_output=0;
    
    pid_reset=false;
  }
  
  
  pid_roll_error=roll-pid_roll_input;pid_pitch_error=pitch-pid_pitch_input;pid_yaw_error=yaw-pid_yaw_input;
  pid_roll_cumError += pid_roll_error*deltat;pid_pitch_cumError += pid_pitch_error*deltat;pid_yaw_cumError += pid_yaw_error*deltat;
  pid_roll_rateError=(pid_roll_error-pid_roll_previousError)/deltat;pid_pitch_rateError=(pid_pitch_error-pid_pitch_previousError)/deltat;pid_yaw_rateError=(pid_yaw_error-pid_yaw_previousError)/deltat;
  
  pid_roll_p=pid_roll_kp*pid_roll_error;pid_pitch_p=pid_pitch_kp*pid_pitch_error; pid_yaw_p=pid_yaw_kp*pid_yaw_error;
  if(-3<pid_roll_error<3)
  {
    pid_roll_i=pid_roll_ki*pid_roll_cumError;
  }
  if(-3<pid_pitch_error<3)
  {
    pid_pitch_i=pid_pitch_ki*pid_pitch_cumError;
  }
  if(-3<pid_yaw_error<3)
  {
     pid_yaw_i=pid_yaw_ki*pid_yaw_cumError;
  }
  pid_roll_d=pid_roll_kd*pid_roll_rateError; pid_pitch_d=pid_pitch_kd*pid_pitch_rateError;pid_yaw_d=pid_yaw_kd*pid_yaw_rateError;
  
  pid_roll_output=pid_roll_p+pid_roll_i+pid_roll_d; pid_pitch_output=pid_pitch_p+pid_pitch_i+pid_pitch_d; pid_yaw_output=pid_yaw_p+pid_yaw_i+pid_yaw_d;
  
  if(pid_roll_output<-400)
  {
    pid_roll_output=-400;
  }
  if(pid_roll_output>400)
  {
    pid_roll_output=400;
  }
  if(pid_pitch_output<-400)
  {
    pid_pitch_output=-400;
  }
  if(pid_pitch_output>400)
  {
    pid_pitch_output=400;
  }
  if(pid_yaw_output<-400)
  {
    pid_yaw_output=-400;
  }
  if(pid_yaw_output>400)
  {
    pid_yaw_output=400;
  }
  pid_roll_previousError=pid_roll_error; pid_pitch_previousError=pid_pitch_error; pid_yaw_previousError=pid_yaw_error;
}

void rc_read_values()
{
   noInterrupts();
   memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
   interrupts();
}

void calc_ch1()
{ 
    calc_input(RC_CH1, RC_CH1_INPUT);
 }
void calc_ch2()
{ 
    calc_input(RC_CH2, RC_CH2_INPUT);
 }
void calc_ch3()
{ 
   calc_input(RC_CH3, RC_CH3_INPUT);
}
void calc_ch4()
{
    calc_input(RC_CH4, RC_CH4_INPUT);
}

void calc_input(uint8_t channel, uint8_t input_pin)
{
    if (digitalRead(input_pin) == HIGH)
    {
        rc_start[channel] = micros();
    }
    else
    {
        uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
        rc_shared[channel] = rc_compare;
    }
}

void writeFloat(uint16_t addr,float x)
{
  union
  {
    byte b[4];
    float f;
  }data;
  data.f=x;
  for(int i=0;i<4;i++)
  {
    EEPROM.write(addr+i,data.b[i]);
  }
}

float readFloat(uint16_t addr)
{
  union
  {
    byte b[4];
    float f;
  }data;
  for(int i=0;i<4;i++)
  {
    data.b[i]=EEPROM.read(addr+i);
  }
  return data.f;
}



void getMres()
{
  switch (Mscale)
  {
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.0f*4912.0f/8190.0f; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.0f*4912.0f/32760.0f; // Proper scale to return milliGauss
          break;
  }
}

void getGres()
{
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0f/32768.0f;
          break;
    case GFS_500DPS:
          gRes = 500.0f/32768.0f;
          break;
    case GFS_1000DPS:
          gRes = 1000.0f/32768.0f;
          break;
    case GFS_2000DPS:
          gRes = 2000.0f/32768.0f;
          break;
  }
}

void getAres()
{
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0f/32768.0f;
          break;
    case AFS_4G:
          aRes = 4.0f/32768.0f;
          break;
    case AFS_8G:
          aRes = 8.0f/32768.0f;
          break;
    case AFS_16G:
          aRes = 16.0f/32768.0f;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
   }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}
       
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

void initMPU9250()
{  
 // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x03; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
   
// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;
   
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }
   
}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz,float deltat)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

// Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

// Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm; // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

// Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm; // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

// Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

// Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

// Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex; // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f; // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

// Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

// Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

// Normalise quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}



        
        // Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

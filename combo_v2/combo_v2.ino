#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
#define OUTPUT_READABLE_YAWPITCHROLL

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// ================================================================
// ===                        PID SETUP                         ===
// ================================================================

float yawKp = 4;
float pitchKp = 1;
float rollKp = 1;


float yawKd = 1;
float pitchKd = 30;
float rollKd = 30;


float yawKi = 0.1;
float pitchKi = 0.1;
float rollKi = 0.1;

long counter = 0;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===               MOTION VARIABLES                           ===
// ================================================================
short iYaw[101], iPitch[101], iRoll[101];
int motor1 = 0, motor2 = 0, motor3 = 0, motor4 = 0, M1 = 3, M2 = 5, M3 = 6, M4 = 9;
int vehicle_yaw = 0, vehicle_pitch = 0, vehicle_roll = 0, prevYawErr, prevPitchErr, prevRollErr, currYawErr, currPitchErr, currRollErr;
int yaw_output, pitch_output, roll_output;
long yawSum = 0, pitchSum = 0, rollSum = 0;
int n_thr = 0, n_pitch = 0, n_roll = 0, n_yaw = 0;
int armed = 0;




// ================================================================
// ===                        RADIO SETUP                       ===
// ================================================================


int yaw, pitch, roll, rb, lb, throttle;
struct tx_packet
{
    int yaw, pitch, roll, throttle, r_button, l_button;
    byte device_ID, target_ID, packet_type;
    float rollKp, pitchKp, rollKd, pitchKd;
}tp1;

struct telem_packet
{
  int yaw, pitch, roll, armed;
  byte vehicle_id;
  long timestamp;
} telem_p1;




RF24 radio(8, 7);
//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };


// The various roles supported by this sketch
typedef enum { role_ping_out = 1, role_pong_back } role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};

// The role of the current running sketch
role_e role = role_pong_back;
long count = 0;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  analogWrite(M1, 0);
  analogWrite(M2, 0);
  analogWrite(M3, 0);
  analogWrite(M4, 0);
  delay(100);

  // put your setup code here, to run once:
    for(int i=0; i<101;i++)
    {
      iYaw[i] = 0;
      iPitch[i] = 0;
      iRoll[i] = 0;
    }
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);

    radio.begin();
    radio.setRetries(20,6);
    delay(100);
    radio.startListening();
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);

    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}



void read_radio()
{
  if ( radio.available() )
  {
    Serial.println("RADIO READ");
    // Dump the payloads until we've gotten everything
    unsigned long got_time;
    bool done = false;
    while (!done)
    {
      //Serial.println("RADIO READ");
      // Dump the payloads until we've gotten everything
      unsigned long got_time;
      bool done = false;
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        done = radio.read( &tp1, sizeof(tp1) );

        yaw = tp1.yaw;
        pitch = tp1.pitch;
        roll = tp1.roll;
        throttle = tp1.throttle;
        lb = tp1.l_button;
        rb = tp1.r_button;
        rollKp = tp1.rollKp;
        pitchKp=tp1.pitchKp;
        rollKd = tp1.rollKd;
        pitchKd=tp1.pitchKd;
        /*READ RADIO COMMANDS

        // Spew it
        Serial.print(tp1.throttle);Serial.print("  ");
        Serial.print(tp1.yaw);Serial.print("  ");
        Serial.print(tp1.l_button);Serial.print("\t");
        Serial.print(tp1.pitch);Serial.print("  ");
        Serial.print(tp1.roll);Serial.print("  ");
        Serial.print(tp1.r_button); Serial.print("\t");
      */
    }

      telem_p1.yaw = vehicle_yaw;
      telem_p1.pitch = vehicle_pitch;
      telem_p1.roll = vehicle_roll;
      telem_p1.armed = armed;
      telem_p1.vehicle_id = 1;

      Serial.print(telem_p1.yaw); Serial.print("\t");
      Serial.print(telem_p1.pitch); Serial.print("\t");
      Serial.print(telem_p1.roll); Serial.println("\t");
      // First, stop listening so we can talk
      radio.stopListening();
      delay(5);
      radio.write( &telem_p1, sizeof(telem_p1));
      //Serial.println("POSE WRITTEN");
      // Now, resume listening so we catch the next packets.
      radio.startListening();
      if(rb==1 && lb==1)

      {
        radio.read( &tp1, sizeof(tp1) );
      }
    }
  }
}



void read_pose()
{
  // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
  
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    if (fifoCount > 0) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        vehicle_yaw = ypr[0]* 180 / M_PI;
        vehicle_pitch = ypr[1]* 180 / M_PI;
        vehicle_roll = ypr[2]* 180 / M_PI;
        Serial.print(vehicle_yaw);Serial.print("\t");Serial.print(vehicle_pitch);Serial.print("\t");Serial.println(vehicle_roll);
        mpu.resetFIFO();
    }
}

void PID_compute_output()
{
  if (armed == 1)
  {
    n_thr = map(throttle, 0, 1023, 10, 130);
    n_roll = map(roll, -512, 512, -30, 30);
    n_pitch = map(pitch, -512, 512, -30, 30);
    n_yaw = map(yaw, -512, 512, -90, 90);

    prevYawErr = currYawErr;
    prevPitchErr = currPitchErr;
    prevRollErr = currRollErr;


    currYawErr = vehicle_yaw - n_yaw;
    currPitchErr = vehicle_pitch - n_pitch;
    currRollErr = vehicle_roll - n_roll;
    yawSum = 0;
    pitchSum = 0;
    rollSum = 0;
    for (int i = 1; i < 100; i++)
    {
      n_thr = map(throttle, 0, 1023, 60, 200);
      n_roll = map(roll, -512, 512, -30, 30);
      n_pitch = map(pitch, -512, 512, -30, 30);
      n_yaw = map(yaw, -530, 510, -30, 30);
  
      prevYawErr = currYawErr;
      prevPitchErr = currPitchErr;
      prevRollErr = currRollErr;
      
      
      currYawErr = vehicle_yaw - n_yaw;
      currPitchErr = vehicle_pitch - n_pitch;
      currRollErr = vehicle_roll - n_roll;
      //Uncomment the following to use integral
      /*
      yawSum =0;
      pitchSum=0;
      rollSum=0;
      
      for (int i = 1; i<100; i++)
      {
        iYaw[i] = iYaw[i+1];
        iPitch[i] = iPitch[i+1];
        iRoll[i] = iRoll[i+1];
        yawSum +=iYaw[i];
        pitchSum +=iPitch[i];
        rollSum +=iRoll[i];
      }
      iYaw[100] = currYawErr;
      iPitch[100] = currPitchErr;
      iRoll[100] = currRollErr;
      
      int intYaw = constrain(yawSum, -1000, 1000);
      int intPitch = constrain(pitchSum, -800, 800);
      int intRoll = constrain(rollSum, -800, 800);
      //Serial.print(intYaw);Serial.print("\t");Serial.print(intPitch);Serial.print("\t");Serial.println(intRoll);
      
      yaw_output = -yawKp*n_yaw +yawKi*intYaw;
      pitch_output = pitchKp*currPitchErr + pitchKd*(currPitchErr-prevPitchErr)  +pitchKi*intPitch;
      roll_output = rollKp*currRollErr + rollKd*(currRollErr-prevRollErr) +rollKi*intRoll;
      */

      //Use this without Ki
      yaw_output = -yawKp*n_yaw;//+yawKi*intYaw;
      pitch_output = pitchKp*currPitchErr + pitchKd*(currPitchErr-prevPitchErr);
      roll_output = rollKp*currRollErr + rollKd*(currRollErr-prevRollErr);
    }
    iYaw[100] = currYawErr;
    iPitch[100] = currPitchErr;
    iRoll[100] = currRollErr;

    int intYaw = constrain(yawSum, -1000, 1000);
    int intPitch = constrain(pitchSum, -800, 800);
    int intRoll = constrain(rollSum, -800, 800);
    //Serial.print(intYaw);Serial.print("\t");Serial.print(intPitch);Serial.print("\t");Serial.println(intRoll);
    yaw_output = yawKp * currYawErr + yawKd * prevYawErr + yawKi * intYaw;
    pitch_output = pitchKp * currPitchErr + pitchKd * prevPitchErr + pitchKi * intPitch;
    roll_output = rollKp * currRollErr + rollKd * prevRollErr + rollKi * intRoll;
  }
}

void write_to_motors()
{
// FOR CONTROL WITH YAW
  motor1 = n_thr - yaw_output - pitch_output + roll_output;
  motor2 = n_thr - yaw_output + pitch_output - roll_output;
  motor3 = n_thr + yaw_output - pitch_output - roll_output;
  motor4 = n_thr + yaw_output + pitch_output + roll_output;
  
  
//FOR CONTROL WITHOUT YAW
/*

  motor1 = n_thr - pitch_output + roll_output;
  motor2 = n_thr + pitch_output - roll_output;
  motor3 = n_thr - pitch_output - roll_output;
  motor4 = n_thr + pitch_output + roll_output;
*/
  if(armed == 1)
  {

    motor1 = constrain(motor1, 10, 130);
    motor2 = constrain(motor2, 10, 130);
    motor3 = constrain(motor3, 10, 130);
    motor4 = constrain(motor4, 10, 130);

//     motor1 = constrain(motor1, 50, 254);
//     motor2 = constrain(motor2, 50, 254);
//     motor3 = constrain(motor3, 50, 254);
//     motor4 = constrain(motor4, 50, 254);

  }
  else if (armed == 0)
  {
    motor1 = 0;
    motor2 = 0;
    motor3 = 0;
    motor4 = 0;
  }
  analogWrite(M1, motor1);
  analogWrite(M2, motor2);
  analogWrite(M3, motor3);
  analogWrite(M4, motor4);
  //Serial.print(motor1);Serial.print("\t");Serial.print(motor2);Serial.print("\t"); Serial.print(motor3);Serial.print("\t");Serial.println(motor4);
}




void loop() {
  // put your main code here, to run repeatedly:
  if (millis() <= 6000)
  {
    mpu.resetFIFO();
    delay(100);
    yawSum = 0;
    pitchSum = 0;
    rollSum = 0;
  }
  else
  {
    read_radio();

    read_pose();
    PID_compute_output();
    //Serial.println("PID COMPUTED");
    write_to_motors();


    //Serial.println("-------------------------------");
    delay(5);
  //Serial.print(millis());Serial.print("\t"); Serial.println(counter);
  //counter++;
  }

}

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define TRANSMITTER_ID 1
#define DRONE_ID 1

int left_joy_x=A3, left_joy_y=A6, right_joy_x=A0, right_joy_y=A1, right_joy_button=3, left_joy_button=4;


//PID Gains

float ROLL_KP = 1;
float PITCH_KP = 1;

float ROLL_KD = 10;
float PITCH_KD = 10;



//





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
}telem_p1;


int success_ctr =0 , fail_ctr =0;


RF24 radio(8,7);
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
role_e role = role_ping_out;



void setup()
{
    Serial.begin(115200);
    radio.begin();
    radio.setRetries(10,10);
    pinMode(3, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    delay(100);

    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
}
void loop()
{
    int yaw= analogRead(left_joy_x);
    int throttle = analogRead(left_joy_y);
    int roll = analogRead(right_joy_x);
    int pitch = analogRead(right_joy_y);
    int button_right = digitalRead(3);
    int button_left = digitalRead(4);
    tp1.yaw = 502 - yaw;
    tp1.pitch = pitch - 506;
    tp1.roll = roll - 510;
    tp1.throttle = throttle;
    tp1.r_button = 1 - button_right;
    tp1.l_button = 1 - button_left;
    tp1.device_ID = TRANSMITTER_ID;
    tp1.target_ID = DRONE_ID;
    tp1.packet_type = 1;
    tp1.rollKp = ROLL_KP;
    tp1.pitchKp =PITCH_KP;
    tp1.rollKd = ROLL_KD;
    tp1.pitchKd = PITCH_KD;
    
    Serial.print(throttle);Serial.print("    ");
    Serial.print(502-yaw);Serial.print("    ");
    Serial.print(1-button_left);Serial.print("\t \t");
    Serial.print(pitch-506);Serial.print("    ");
    Serial.print(roll-510);Serial.print("    ");
    Serial.println(1-button_right);
    
    delay(5);
    
    // First, stop listening so we can talk.
    
    // Take the time, and send it.  This will block until complete
    unsigned long time = millis();
    //Serial.println("Now sending %lu...");
    bool ok = radio.write( &tp1, sizeof(tp1) );
    
    if (ok)
      Serial.println("ok...");
    else
      Serial.println("failed.\n\r");

    // Now, continue listening
    radio.startListening();
    //delay(5);
    bool timeout = false;
    long started_waiting_at = millis();
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 30 )
        timeout = true;
    if (timeout)
    {
      //Serial.println("Resnpose missed");
      fail_ctr++;
    }
    else
    {
      
      radio.read(&telem_p1, sizeof(telem_p1));
      //Serial.println("GOT RESPONSE");
      success_ctr++;
      
      Serial.print(telem_p1.yaw);Serial.print("\t");
      Serial.print(telem_p1.pitch);Serial.print("\t");
      Serial.print(telem_p1.roll);Serial.print("\t");
      Serial.print(telem_p1.armed);Serial.print("\t");Serial.print(success_ctr+fail_ctr);Serial.print("\t");Serial.println(millis());
      
    }
    radio.stopListening();
    delay(5);
    //Serial.print(success_ctr);Serial.print("\t"); Serial.println(fail_ctr);
    //Serial.println(millis());
    
}

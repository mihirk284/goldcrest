#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define TRANSMITTER_ID 1
#define DRONE_ID 1

int left_joy_x=A3, left_joy_y=A6, right_joy_x=A0, right_joy_y=A1, right_joy_button=3, left_joy_button=4;

struct tx_packet
{
    int yaw, pitch, roll, throttle, r_button, l_button;
    byte device_ID, target_ID, packet_type;
}tp1;






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
role_e role = role_pong_back;
long count =0;


void setup()
{
    Serial.begin(115200);
    radio.begin();
    radio.setRetries(20,3);
    pinMode(3, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    delay(100);
    radio.startListening();
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
}
void loop()
{
    
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      unsigned long got_time;
      bool done = false;
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        done = radio.read( &tp1, sizeof(tp1) );

        // Spew it
        Serial.print(tp1.throttle);Serial.print("  ");
        Serial.print(tp1.yaw);Serial.print("  ");
        Serial.print(tp1.l_button);Serial.print("\t");
        Serial.print(tp1.pitch);Serial.print("  ");
        Serial.print(tp1.roll);Serial.print("  ");
        Serial.println(tp1.r_button);

      	// Delay just a little bit to let the other unit
      	// make the transition to receiver
      	delay(5);
      }

      // First, stop listening so we can talk
      radio.stopListening();

      // Send the final one back.
      radio.write( &tp1, sizeof(tp1) );
      Serial.println("Sent response.\n\r");

      // Now, resume listening so we catch the next packets.
      radio.startListening();
      count++;
    }
    delay(5);
    Serial.println();
    Serial.println(millis());
    Serial.println(count);
    Serial.println();
}

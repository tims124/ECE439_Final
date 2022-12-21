
/*
file:sensorManagerMultiRate.ino
date:9.20.2016
Author: Alex Dawson-Elli
Contributor: Peter Adamczyk

Updated 2021-02-17: 
	multi-rate streaming (Encoders 100 Hz, Analog and Ultrasound 10 Hz)
	Remove #STREAM and #REQUEST_RESPONSE flags (never really existed)
	Remove #APERIODIC mode due to interrupt interference. 
	Add #DECIMATION_COUNT to report Analog and Ultrasound much less frequently than Encoders. 
Updated 2021-09-21: 
  Change encoders to E1 and E2
*/

//imports
#include <avr/io.h>
#include <stdint.h>            // has to be added to use uint8_t
#include <avr/interrupt.h>     // Needed to use interrupts    
#include <Arduino.h>           // Needed for analog macros A0 etc.

////libraries to be stored in local directories
//#include "TimerOne/TimerOne.h"         // High level lib for timer interrupt
//#include "NewPing/NewPing.h"
// Use this form if the libraries have been imported to Arduino workspace
#include <TimerOne.h>
#include <NewPing.h>

//--------------version selection--------------------


//------------------globals--------------------------


//-------------ISR globals

//encoder
volatile long E1Count;
volatile long E2Count;
volatile uint8_t PINBhistory = 0x00; 
volatile int interruptCount = 0;


/*
PIN mapping Table:

__________|_______|______
Encoder11 | PINB0 | D8          //encoder 1, channel 1 of quadrature
Encoder12 | PINB1 | D9          //encoder 1, channel 2 of quadrature            
Encoder21 | PINB2 | D10         //...
Encoder22 | PINB3 | D11         //...
*/


/*lookup table - 4x4 table, defining how to adjust
the counter depending on the transition between
the old state and the new one. the stateChangeTable
should be called as: stateChangeTable[old][new]

note that the state of the bits maps to the index
values of the table as the decimal representation of
their binary values:
00 -> 0
01 -> 1
10 -> 2
11 -> 3
*/

char stateChangeTable[4][4] = //validate correctness
{
    { 0,-1, 1, 0},
    { 1, 0, 0,-1},
    {-1, 0, 0, 1},
    { 0, 1,-1, 0}
};



//ping
volatile int pingFlightTime[3];

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define SONAR_NUM 3
uint8_t triggerPin[3] = {2, 4, 6};
uint8_t    echoPin[3] = {3, 5, 7};


//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 
NewPing sonar[SONAR_NUM] =    //sensor object array
{  
  NewPing(triggerPin[0], echoPin[0], MAX_DISTANCE),  // U0 
  NewPing(triggerPin[1], echoPin[1], MAX_DISTANCE),  // U1
  NewPing(triggerPin[2], echoPin[2], MAX_DISTANCE)   // U2
};

//analog
unsigned int A_0;
unsigned int A_1;
unsigned int A_2;
unsigned int A_3;
unsigned int A_4;
unsigned int A_5;
unsigned int A_6;  
unsigned int A_7; 
 
//serial print
#define DECIMATION_COUNT 3   // each non-encoder sensor prints every DECIMATION_COUNT encoder prints (at 100 Hz)
char count = 0;

//--------Update Functions-------------

// Interrupt Service Routine (ISR)

//Pin Change Interrupt Service Routine (pins D8 to D13)
// Counts encoders 
ISR (PCINT0_vect)
{
    //process PINB
    uint8_t PINBcurrent = PINB & 0b00001111; //grab only the first 4 pins of PINB
    uint8_t changedbits = PINBcurrent ^ PINBhistory;
    
    //read encoder 1 and update
     if(changedbits & 0b00000011) //something has changed in encoder 1's state
    {
        E1Count += stateChangeTable[(PINBhistory & 0b00000011)][(PINBcurrent & 0b00000011)];
    }
    

    //read encoder 2 and update
    if(changedbits & 0b00001100) //something has changed with encoder 2's state
    {
        // note: 0b00001111 >> 2  == 0b00000011 (shift and mask at same time)
        E2Count += stateChangeTable[(PINBhistory >> 2)][(PINBcurrent >> 2)];
    }


    PINBhistory = PINBcurrent;  

}

//----------- Ultrasound function to update all the two-port Ultrasonic sensors ------------
void updatePing()
{
	
     for (uint8_t i = 0; i < SONAR_NUM; i++)
     {
        //get read
        pingFlightTime[i] = sonar[i].ping();
        delay(50); //non-blocking
        
        //fix for defective sensors.
        if (pingFlightTime[i] == 0) 
        {
            pinMode(echoPin[i], OUTPUT);
            digitalWrite(echoPin[i], LOW);
            pinMode(echoPin[i], INPUT);
        }


     }

}


//--------- Timer interrupt callback to send data over Serial -------------
void serialStream()
{
    sei(); // don't miss encoder counts

    //Always stream back encoder values
	// Serial.print("E1:");
    // Serial.println(E1Count);
    // Serial.print("E2:");
    // Serial.println(E2Count);

	// Also stream other signals, decimated (one other sensor per encoder sample)
	// First Analogs 0-7
    if (count == 0)
    {
		A_0 = analogRead(A0);
		//Serial.print("LEFT:");
		Serial.print(A_0);
                //delay(10);
	}
	else if (count == 1)
	{
		A_1 = analogRead(A1);
		Serial.print(" : ");
		Serial.print(A_1);
                //delay(10);
	}
	else if (count == 2)
	{
		A_2 = analogRead(A2);
		Serial.print(" : ");
		Serial.println(A_2);
                //delay(10);
	}
	else if (count == 3)
	{
		// A_3 = analogRead(A3);
		// Serial.print("A3:");
		// Serial.println(A_3);
	}
	else if (count == 4)
	{
//		A_4 = analogRead(A4);
//		Serial.print("Left:");
//		Serial.print(A_4);
	}
	else if (count == 5)
	{
		// A_5 = analogRead(A5);
		// Serial.print("A5:");
		// Serial.println(A_5);
	}
	else if (count == 6)
	{
		// A_6 = analogRead(A6);
		// Serial.print("A6:");
		// Serial.println(A_6);
	}
	else if (count == 7)
	{
		// A_7 = analogRead(A7);
		// Serial.print("A7:");
		// Serial.println(A_7);
	}
//	// Now Ultrasounds
//	else if (count == 8)
//	{
//		Serial.print("U0:");
//		Serial.println(pingFlightTime[0]);
//	}
//	else if (count == 9)
//	{
//		Serial.print("U1:");
//		Serial.println(pingFlightTime[1]);
//	}
//	else if (count == 10)
//	{
//		Serial.print("U2:");
//		Serial.println(pingFlightTime[2]);
//	}

    //update "count"
    count++;
    if(count >= DECIMATION_COUNT){count = 0;}

}



//----setup-------------

void setup()
{
    //encoder PC interrupt setup
    DDRB &= ~((1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3)); // DDRB = xxxx0000, PB0,PB1,PB2,PB3 are now inputs
    PORTB &= ~((1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3)); // turn off the Pull-ups on b  PORTB = xxxx0000;
   
    PCICR  |= (1 << PCIE0);     // enable pin change (PC) interrupts for D8 to D13, aka PORTB. set PCIE0 to enable PCMSK0 scan
    PCMSK0  = 0b00001111;       // enable PC interrupts only on pins 0,1,2,3
    PCIFR  |= (1 << PCIF0);     // clear any outstanding interrupts in PC flag register

    //Timer1 timer counter interrupt setup
    Timer1.initialize(10000);     //ISR period in microseconds
    Timer1.attachInterrupt(serialStream);
    Serial.begin(57600);

    sei();                     // turn on interrupts

}


//----------main loop------------
void loop()
{
    updatePing();
}

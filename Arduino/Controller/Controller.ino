/*
 Copyright (C) 2012 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Simplest possible example of using RF24Network,
 *
 * RECEIVER NODE
 * Listens for messages from the transmitter and prints them out.
 */

#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <printf.h>
#define MAX_SIZE 6

// nRF24L01(+) radio attached using Getting Started board 
RF24 radio(6,7);

// Network uses that radio
RF24Network network(radio);

// Address of our node
const uint16_t this_node = 0;

// Address of the other node
const uint16_t other_node = 1;

// Structure of our payload
struct payload_t
{
  double dataVector[MAX_SIZE];
};
double time = 0;
int lastNumber = 0;
int missed = 0;
void setup(void)
{
  pinMode(2,OUTPUT);
  Serial.begin(115200);
  Serial.println("RF24Network/examples/helloworld_rx/");
      printf_begin();

  SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ this_node);
}

void loop(void)
{
  // Pump the network regularly
  network.update();
  digitalWrite(2,LOW);

  // Is there anything ready for us?
  while ( network.available() )
  {
    readData(); 
  }
}
void readData()
{
   RF24NetworkHeader header;
    payload_t payload;
    network.read(header,&payload,sizeof(payload));

    switch (header.type)
    {
      case 0:          
        Serial.print(payload.dataVector[0]);
        Serial.print(" \t");
        Serial.print(payload.dataVector[1]);
        Serial.print(" \t");
        Serial.print(payload.dataVector[2]);
        Serial.print(" \t");
        Serial.print(payload.dataVector[3]);
        Serial.println("");
        break;
    }
}

// vim:ai:cin:sts=2 sw=2 ft=cpp

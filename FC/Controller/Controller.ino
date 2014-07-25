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
  double angle[2];
  double pid[2];
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
    digitalWrite(2,HIGH);
    // If so, grab it and print it out
    RF24NetworkHeader header;
    payload_t payload;
    network.read(header,&payload,sizeof(payload));

    Serial.print(payload.angle[0]);
    Serial.print(" \t");
    Serial.print(payload.angle[1]);
    Serial.print(" \t");
    Serial.print(payload.pid[0]);
    Serial.print(" \t");
    Serial.print(payload.pid[1]);
    Serial.println("");

  }
}
// vim:ai:cin:sts=2 sw=2 ft=cpp

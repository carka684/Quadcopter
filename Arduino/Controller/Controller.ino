
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <printf.h>
#define MAX_SIZE 6
#define CE_PIN 7
#define CSN_PIN 8
#define THIS_NODE 0
#define OTHER_NODE 1
#define TYPE_ANGLE 0 
#define TYPE_CONTROL 1

RF24 radio(CE_PIN,CSN_PIN);
RF24Network network(radio);
struct payload_t
{
	double dataVector[MAX_SIZE];
};
double time = 0;
int lastNumber = 0;
int missed = 0;
int joystick;
void setup(void)
{
	Serial.begin(115200);
	printf_begin();

	SPI.begin();
	radio.begin();
	network.begin(/*channel*/ 90, /*node address*/ THIS_NODE);
}

void loop(void)
{
	network.update();
	if(network.available())
	{
		readData(); 
	}

	if(Serial.available())
	{
		readSerial();
	}

	int tmp = analogRead(0);
	if(abs(joystick - tmp) > 3 && millis() - time > 50 )
	{
		joystick = tmp;
		sendData(joystick);
		time = millis();
	}
  
}
void sendData(double data)
{
	network.update();
	payload_t payload;
	payload.dataVector[0] = data;  

	RF24NetworkHeader header(/*to node*/ OTHER_NODE,TYPE_CONTROL);
	while(!network.write(header,&payload,sizeof(payload)));
}
void readData()
{
	RF24NetworkHeader header;
	payload_t payload;
	network.read(header,&payload,sizeof(payload));

	switch (header.type)
	{
		case TYPE_ANGLE:          
			Serial.print(payload.dataVector[0]);
			Serial.print(" \t");
			Serial.print(payload.dataVector[1]);
			Serial.print(" \t");
			Serial.print(payload.dataVector[2]);
			Serial.print(" \t");
			Serial.print(payload.dataVector[3]);
			Serial.println("");
		break;
		case TYPE_CONTROL:
			Serial.println(payload.dataVector[0]);
		break;
	}
}
void readSerial()
{
  
}

// vim:ai:cin:sts=2 sw=2 ft=cpp

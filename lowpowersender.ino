
  
#include <Arduino.h>
#include "/home/thomas/Arduino/ESP32_Temp/exchange.h"
#include "/home/thomas/Arduino/ESP32_Temp/exchange.cpp"


#include <Wire.h>
#include <Adafruit_BMP085.h>


#define debugSerial Serial 
#define SHOW_DEBUGINFO 0
#define debugSerialprintln(...) { if (SHOW_DEBUGINFO) debugSerial.println(__VA_ARGS__); }
#define debugSerialprint(...)   { if (SHOW_DEBUGINFO) debugSerial.print(__VA_ARGS__); } 
#define debugSerialbegin(...)   { if (SHOW_DEBUGINFO) debugSerial.begin(__VA_ARGS__); } 
#define debugSerialflush(...)   { if (SHOW_DEBUGINFO) debugSerial.flush(); }

#include <SPI.h>
#include <LoRa.h>
#include <LowPower.h>
#include  "adcvcc.h" 
#include <avr/wdt.h>
// LED
#define LED PD3

// define the pins used by the transceiver module
#define ss 10
#define rst -1
#define dio0 PD2

// Identification String of module
#define IDENTIFICATION 'B'   
#define LORA_SYNC_WORD   0xFA   

u_exchange_t u_packet = {
  { {'F', IDENTIFICATION, 0, 0} , 1, 0.0, 0.0, {'V', '1', '.', '0'}, sizeof(s_exchange_t), 0 }, 
};


double temperature;
unsigned long crc;

long counter = 1; // number of packets
int start = true;

#define FASTINTERVAL    8    // 8 seconds (for testing)
#define NORMALINTERVAL  3600   // 60 minutes (normal)

int interval = NORMALINTERVAL / 8;
//int interval = FASTINTERVAL / 8;

// temperature sesnor
Adafruit_BMP085 bmp;


String s;

void blink(int nr, int ms) {
  for(int i=0; i<nr; i++) {
    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(ms);                 // wait for a msecond
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
    delay(ms);                 // wait for a msecond
  }
}


ISR(ADC_vect)  
{
  // Increment ADC counter
  _adc_irq_cnt++;
}


void onReceive(int packetSize) {
  // received a packet
  debugSerialprint("Received packet '");

  // read packet
  for (int i = 0; i < packetSize; i++) {
    debugSerialprint((char)LoRa.read());
  }

  // print RSSI of packet
  debugSerialprint("' with RSSI ");
  debugSerialprintln(LoRa.packetRssi());
}


float getVoltage() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (3.3 / 1023.0);
  // print out the value you read:
  debugSerialprintln(voltage);
  return(voltage);
}



void setup() {
  int i;
  
  //initialize Serial Monitor
  debugSerialbegin(115200);

  pinMode(LED, OUTPUT);
  
  while (!Serial);
  debugSerialprintln("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);


  for(i=0; !LoRa.begin(866E6) && i < 10; i++) {
    debugSerialprint(".");
    delay(500);
  }
  if(i >= 10) {
    for(;;){
      blink(3, 250); 
      delay(500);
    }
  }
  
  // Change sync word (0xFA) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(LORA_SYNC_WORD);
  debugSerialprintln("\n\nLoRa Initializing OK!");

  if (!bmp.begin()) {
    debugSerialprintln("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    for(;;){
      blink(4, 250); 
      delay(500);
    }
  }
  else debugSerialprintln("Found BMP085 sensor");

  debugSerialprint("Identification: F");
  debugSerialprintln(IDENTIFICATION);

  blink(1, 500);

  wdt_disable();

}

void loop() {

  u_packet.message.readingID = counter++;
  
  u_packet.message.temperature = (float)bmp.readTemperature();
  u_packet.message.vcc = getVoltage();

  u_packet.message.crc = crcFlash(u_packet.u_arrayChar, sizeof(s_exchange_t)-sizeof(unsigned long));

  debugSerialprint("\nCurrent Temperature:  ");
  debugSerial.print(u_packet.message.temperature);
  debugSerial.print(" and Vcc: ");
  debugSerial.println(u_packet.message.vcc);

  //debugSerialprint("CRC: ");
  //debugSerialprintln(u_packet.message.crc);

  for(int i=0; i <3; i++) {

    
    debugSerialprint("Sending 3 packets: ");
    debugSerialprintln(counter-1);

    //Send LoRa packet to receiver
    LoRa.beginPacket();
    int size;

    LoRa.write(&(*(u_packet.u_arrayChar)),SIZE_EXCHANGE);  
    // don't understand, why I have to dereference and the to take address
    // there must be some difference 
    
    LoRa.endPacket();
    
    debugSerialprintln("End Sending Packet ");
    debugSerialflush();
    delay(100);
  /*  We don't receive packets
    // register the receive callback
    LoRa.onReceive(onReceive);
    // put the radio into receive mode
    LoRa.receive();
    delay(1000); // wait one second to receive packet
  */
  }

  LoRa.end(); // sleep, to save poser
 
  if(start) {
    //start = false;
    blink(2, 500);
  }
  

  
  for (int i = 0; i < interval; i++)
  {  
          if((interval - i) % 100 == 0) {
            debugSerialprint("Number of 8s sleep cycles still to sleep: ");
            debugSerialprintln(interval - i);
          }
          debugSerialflush();
               
          // Enter power down state for 8 s with ADC and BOD module disabled
 
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);   
          
  }  
  
  debugSerialprint("Finished Sleeping 8s sleep cylces: ");
  debugSerialprintln(interval);
  debugSerialflush();
}

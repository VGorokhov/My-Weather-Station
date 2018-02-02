/*
 * 
 * Бибилиотеки для прошивки - MySensors.
   Copyright (C) 2013-2018 Sensnology AB
   Full contributor list: https://github.com/mysensors/MySensors
   
 *******************************
   Версия прошивки
   Version 2.0 - VGorokhov

   Документация по прошивке
   "Сенсоры погодной станции-2":
    Направление ветра
    Скорость ветра
        
 * ****************************
 *    PCFf8574T        ARDUINO
 *  --------------
 *  VCC          |  VCC ->    3.3V
 *  GND          |  GND ->    GND
 *  SCL          |  SCL -> A5 SCL
 *  SDA          |  SDA -> A4 SDA
 *  --------------
 *  
 *   
 * http://majordomo.smartliving.ru/forum/viewforum.php?f=20
 *
 * Исходный файл 
 * https://github.com/VGorokhov/My-Weather-Station/blob/master/Arduino/MyWeatherStation/WeatherSensor2/WeatherSensor2.ino
 *  
 *  
 *
  
  

DATA pin is to be connected directly to one of Arduino ports.
*/

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#define MY_NODE_ID 12 //uncomment this line to assign a static ID

#include <MySensors.h>
#include <SPI.h>
#include <Wire.h>    // Required for I2C communication
#include "PCF8575.h" // Required for PCF8575

byte address = 0x20;

PCF8575 PCF8575;

#define CHILD_ID_WIND 1

#define SKETCH_NAME "Weather Sensors_2"
#define SKETCH_VERSION "1.0"

// An anemometer for the Arduino
int LogInterval;
int SampInterval=2;   //Number of seconds in the LogInterval

#define WAIT_TO_START    0 // Wait for serial input in setup()

const byte nsamp=10;

#define WindPin 4

// Wind variables
bool SeeHigh=false;
int CntRPM=0;
unsigned long CntPeriod=0;
float RPM=0.0;
float AvPeriod=0.0;
float speed=0.0;

// Timing counters
unsigned long StartTime;  //Log Interval counter
unsigned long StartPeriod=0;  // Period Start
unsigned long timeSense=0;


// standard messages
MyMessage msgWSpeed(CHILD_ID_WIND, V_WIND);
MyMessage msgWDirection(CHILD_ID_WIND, V_DIRECTION);


void setup(){
analogReference(EXTERNAL); //3.3V connected to AREF thru 3.3K AREF=3VDC
  
Serial.begin(115200);
  //Serial.println("I'm alive!");
 Serial.println();
  pinMode(WindPin,INPUT);
  
  #if WAIT_TO_START
    Serial.println("Type any character to start");
    while (!Serial.available());
  #endif //WAIT_TO_START

  Serial.println("RPM, MPH");
  LogInterval=SampInterval*1000;   //  sec between entries
  StartTime=millis(); //Initialize StartTime

  
  PCF8575.begin(address);

  for(int i = 0; i < 7; i++)
  {
    PCF8575.pinMode(i,INPUT);
  }
  
}

void presentation()
{
    // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_WIND, S_WIND);
  
}

void loop()
{
  

// Wind calculator Look for High
  if (digitalRead(WindPin)==HIGH)
    {
    SeeHigh=true;
  
  }
  //Look for Low thus a High to Low transistion
  else if (SeeHigh==true)
  {
    if (StartPeriod != 0) //Not first sampleFrt
    {
      // Number of milliseconds in a revolution, added together
      CntPeriod=CntPeriod+millis()-StartPeriod;
    } 
    StartPeriod=millis();
    //Increment counter
    CntRPM++;
    SeeHigh=false;
  
  }
   
  if ((millis()-StartTime)>long(LogInterval))  // Log Interval has passed
  {
    // Do Wind calculations for LogInterval
    // RPM is calculated, Period is averaged
    RPM=CntRPM*(60.0/SampInterval);
    AvPeriod=CntPeriod/(CntRPM-1);
    speed=RPM*.054; //Estimate
    

   // Serial.print("$, ");
      //  Serial.print(RPM,1);
  //  Serial.print(", ");    
 //   Serial.print(speed,1);
    
  
    StartTime=millis();
   // Serial.println();

    StartTime = millis(); //Get StartTime for next LogInterval
    // Clear variables for next LogInterval
  
    SeeHigh=false;
    CntRPM=0;
    AvPeriod=0.0;
  }
  
  
//String dirTable[]= {"N","NE","E","SE","S","SW","W","NW"}; 
  String dirTable[]= {"0","45","90","135","180","225","270","315"};
 
  float a = 45.0;

  
  int direction;

 for(int i = 0; i < 7; i++)
  {
    direction = PCF8575.digitalRead(i); // Tun on all pins
    send(msgWDirection.set(direction*a ,1));
    send(msgWSpeed.set(speed, 1));
    Serial.println("Dir = " + dirTable[direction] + "°");
    Serial.println("Speed = " + String(speed,1) + " km/h");
  }

  

  
  //delay between succesive read requests must be at least 5sec, otherwise wind speed will read 0.
  delay(5000);
}

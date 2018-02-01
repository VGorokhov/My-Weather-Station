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
  # Arduino-LaCrosse-TX23-Library

  LaCrosse TX23 is a wind speed and direction sensor. It uses 3 wires for communication and power:

  Pin1 - Brown(Black) - DATA
  Pin2 - Red - Vcc
  Pin3 - Green - N/C
  Pin4 - Yellow - GND

DATA pin is to be connected directly to one of Arduino ports.
*/

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <MySensors.h>
#include <SPI.h>
#include <LaCrosse_TX23.h>

#define CHILD_ID_WIND 1

#define SKETCH_NAME "Weather Sensors_2"
#define SKETCH_VERSION "1.0"

//DATA wire connected to arduino port 4
LaCrosse_TX23 anemometer = LaCrosse_TX23(4);

// standard messages
MyMessage msgWSpeed(CHILD_ID_WIND, V_WIND);
MyMessage msgWDirection(CHILD_ID_WIND, V_DIRECTION);


void setup(){
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
  
  //String dirTable[]= {"N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW"}; 
  String dirTable[]= {"0","22.5","45","67.5","90","112.5","135","157.5","180","202.5","225","247.5","270","292.5","315","337.5"};
  float speed;
  float a = 22.5;
  int direction;

  if(anemometer.read(speed, direction))
  {
    send(msgWDirection.set(direction*a ,1));
    send(msgWSpeed.set(speed, 1));
    
    Serial.println("Speed = " + String(speed,1) + " m/s");
    Serial.println("Dir = " + dirTable[direction] + " Grad");  
  }
  else
  {
    Serial.println("Read error");
  }

  
  //delay between succesive read requests must be at least 2sec, otherwise wind speed will read 0.
  delay(2000);
}

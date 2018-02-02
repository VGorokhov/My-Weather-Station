/*
 * 
 * Бибилиотеки для прошивки - MySensors.
   Copyright (C) 2013-2018 Sensnology AB
   Full contributor list: https://github.com/mysensors/MySensors
   
 *******************************
   Версия прошивки
   Version 2.0 - VGorokhov

   Документация по прошивке
   "Сенсоры погодной станции-3":
    Влажность почвы
    Температура почвы
    Напряжение батареи питания
        
 * ****************************
 * http://majordomo.smartliving.ru/forum/viewforum.php?f=20
 *
 * Исходный файл 
 * https://github.com/VGorokhov/My-Weather-Station/blob/master/Arduino/MyWeatherStation/WeatherSensor2/WeatherSensor3.ino
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

#include <SPI.h>
#include <math.h>       // Conversion equation from resistance to %
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No

#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 16

#define NUM_READS (int)10    // Number of sensor reads for filtering
#define TEMP_CHILD_ID 0
#define SOIL_CHILD_ID 1
#define BATT_CHILD_ID 10

#define batteryVoltage_PIN  A0    //analog input A0 on ATmega328 is battery voltage ( /2)
#define LTC4067_CHRG_PIN  A1    //analog input A1 on ATmega 328 is /CHRG signal from LTC4067

const float VccMin        = 1.0*3.5;  // Minimum expected Vcc level, in Volts. Example for 1 rechargeable lithium-ion.
const float VccMax        = 1.0*4.2;  // Maximum expected Vcc level, in Volts. 

float lastBattVoltage;
int lastBattPct = 0;
float VccReference = 3.3;

boolean charge = false;

unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
bool receivedConfig = false;
bool metric = true;

static const uint64_t UPDATE_INTERVAL = 60000;
// Initialize temperature message
MyMessage msgTemp(0,V_TEMP);
MyMessage msgSoil(SOIL_CHILD_ID, V_LEVEL);
MyMessage msgPatteryVoltage(BATT_CHILD_ID, V_VOLTAGE);

//uint32_t SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)


long buffer[NUM_READS];
int idx;

/// @brief Structure to be used in percentage and resistance values matrix to be filtered (have to be in pairs)
typedef struct {
  int moisture; //!< Moisture
  long resistance; //!< Resistance
} values;

const long knownResistor = 4700;  // Constant value of known resistor in Ohms

int supplyVoltage;                // Measured supply voltage
int sensorVoltage;                // Measured sensor voltage

values valueOf[NUM_READS];        // Calculated moisture percentages and resistances to be sorted and filtered

int i;                            // Simple index variable

void before()
{
  // Startup up the OneWire library
  sensors.begin();
}

void setup()
{
  analogReference(DEFAULT);             // default external reference = 3.3v for Ceech board
  VccReference = 3.323 ;                // measured Vcc input (on board LDO)
  
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);
  
  // initialize the digital pins as an output.
  // Pin 6,7 is for sensor 1
  // initialize the digital pin as an output.
  // Pin 6 is sense resistor voltage supply 1
  pinMode(6, OUTPUT);

  // initialize the digital pin as an output.
  // Pin 7 is sense resistor voltage supply 2
  pinMode(7, OUTPUT);
}

void presentation()
{
  sendSketchInfo("Soil Moisture Sensor Reverse Polarity", "1.0");
  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();

  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
     present(i, S_TEMP);
  }
     present(SOIL_CHILD_ID, S_MOISTURE);
     present(BATT_CHILD_ID, S_POWER);    // Battery parameters
}



void loop()
{
// Wait, but receive messages
  wait(UPDATE_INTERVAL); 

// Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  sleep(conversionTime);

  // Read temperatures and send them to controller 
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
 
    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
 
    // Only send data if temperature has changed and no error
    #if COMPARE_TEMP == 1
    if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
    #else
    if (temperature != -127.00 && temperature != 85.00) {
    #endif
 
      // Send in the new temperature
      send(msgTemp.setSensor(i).set(temperature,1));
      // Save new temperatures for next compare
      lastTemperature[i]=temperature;
    }
  }
  
  measure(6,7,1);
  Serial.print ("\t");
  Serial.println (average());
  long read1 = average();

  measure(7,6,0);
  Serial.print ("\t");
  Serial.println (average());
  long read2= average();

  long sensor1 = (read1 + read2)/2;

  Serial.print ("resistance bias =" );
  Serial.println (read1-read2);
  Serial.print ("sensor bias compensated value = ");
  Serial.println (sensor1);
  Serial.println ();

  //send back the values
  send(msgSoil.set((long int)ceil(sensor1)));
  // delay until next measurement (msec)
  sleep(SLEEP_TIME);
}

void measure (int phase_b, int phase_a, int analog_input)
{
  // read sensor, filter, and calculate resistance value
  // Noise filter: median filter

  for (i=0; i<NUM_READS; i++) {

    // Read 1 pair of voltage values
    digitalWrite(phase_a, HIGH);                 // set the voltage supply on
    delayMicroseconds(25);
    supplyVoltage = analogRead(analog_input);   // read the supply voltage
    delayMicroseconds(25);
    digitalWrite(phase_a, LOW);                  // set the voltage supply off
    delay(1);

    digitalWrite(phase_b, HIGH);                 // set the voltage supply on
    delayMicroseconds(25);
    sensorVoltage = analogRead(analog_input);   // read the sensor voltage
    delayMicroseconds(25);
    digitalWrite(phase_b, LOW);                  // set the voltage supply off

    // Calculate resistance
    // the 0.5 add-term is used to round to the nearest integer
    // Tip: no need to transform 0-1023 voltage value to 0-5 range, due to following fraction
    long resistance = (knownResistor * (supplyVoltage - sensorVoltage ) / sensorVoltage) ;

    delay(1);
    addReading(resistance);
    Serial.print (resistance);
    Serial.print ("\t");
  }
}



// Averaging algorithm
void addReading(long resistance)
{
  buffer[idx] = resistance;
  idx++;
  if (idx >= NUM_READS) {
    idx = 0;
  }
}

long average()
{
  long sum = 0;
  for (int i = 0; i < NUM_READS; i++) {
    sum += buffer[i];
  }
  return (long)(sum / NUM_READS);
}

void sendVoltage(void)
// battery values
{
  // get Battery Voltage
  float batteryVoltage = ((float)analogRead(batteryVoltage_PIN)* VccReference/1024) * 2;  // actual voltage is double
  Serial.print("Batt: ");
  Serial.print(batteryVoltage);
  Serial.print("V ; ");
  
  // send battery percentage for node
  int battPct = 1 ;
  if (batteryVoltage > VccMin){
    battPct = 100.0*(batteryVoltage - VccMin)/(VccMax - VccMin);
  }

  charge = digitalRead(LTC4067_CHRG_PIN);
  
  send(msgPatteryVoltage.set(batteryVoltage, 3));     // Send (V)
  sendBatteryLevel(battPct);
  
  Serial.print("BattPct: ");
  Serial.print(battPct);
  Serial.println("% ");
}



/**
   
  *******************************

   Версия прошивки
   Version 1.0 - VGorokhov

   Документация по прошивке
   "Датчик осадков":
   
   
   http://majordomo.smartliving.ru/forum/viewforum.php?f=20

   Исходный файл 
   https://github.com/VGorokhov/My-Weather-Station/tree/master/Arduino/MyWeatherStation/SolarTracer


*/
//Rain variable
bool RainHigh=false;
const float LowAmt=5.0; //when rain is low, takes this ml to trip
const float HiAmt=5.0;    //when rain is high, takes this ml to trip
float RainAccum=0.0;     //Rain accumulator since start of sample

void setup(void) {
   // Rain get start state
   if (digitalRead(RainPin)==HIGH)
    {
        RainHigh=true;
    }
   else
   {
        RainHigh=false;
    }

In setup, I determine if the RainPin is high or low. This just determines which bucket is up and the starting point to start counting tips of the bucket.

void loop(void)
{

// Rain calculator, looks for Rain continuously
// Look for low to high
if ((RainHigh==false)&&(digitalRead(RainPin)==HIGH))
{
   RainHigh=true;
   RainAccum+=LowAmt;
}
if ((RainHigh==true)&&(digitalRead(RainPin)==LOW))
{
   RainHigh=false;
   RainAccum+=HiAmt;
}



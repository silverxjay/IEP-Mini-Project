// Title
// Mak Si Jie, Harold Mah
// 2516477, 2517100
// DCEP/FT/1A/09
// Description

#define LED_YELLOW 7
#define BUTTON_K2 9
#include <math.h>
#include <Wire.h>

#include "RichShieldLightSensor.h"
#include "RichShieldTM1637.h"
#include "RichShieldNTC.h"

#define NTC_PIN A1 //SIG pin of NTC module connect to A1 of IO Shield, that is pin A1 of OPEN-SMART UNO R3
NTC temper(NTC_PIN);
#define CLK 10//CLK of the TM1637 IC connect to D10 of OPEN-SMART UNO R3
#define DIO 11
TM1637 disp(CLK,DIO);

#define LIGHTSENSOR_PIN A2
LightSensor lightsensor(LIGHTSENSOR_PIN); 
int toggle = 0;

void setup() {
Serial.begin(9600);
pinMode(LED_YELLOW, OUTPUT); 
pinMode(BUTTON_K2, INPUT_PULLUP);
disp.init(); 

}

void loop() { 
float Rsensor = lightsensor.getRes(); 
float lux;
//--------Light Dependant LED------
if (digitalRead(BUTTON_K2) == 0)
{
 if (toggle == 0)
  toggle = 1;
else 
toggle = 0;
delay (300);
while (digitalRead(BUTTON_K2) == 0);
}
if (toggle == 1){
//Read light level

  lux = 325*pow(Rsensor,-1.4); //Calculate lux value
  
  Serial.print("Illuminance is almost "); 
  Serial.print(lux,1);
  Serial.println(" lux");
  delay(250);  

//Conditions for light to turn on
 if (lux < 100) 
    digitalWrite(LED_YELLOW, HIGH); 
  else
    digitalWrite(LED_YELLOW, LOW);

}
else
digitalWrite(LED_YELLOW, LOW);
}


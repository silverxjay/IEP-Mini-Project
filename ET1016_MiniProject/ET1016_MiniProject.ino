// Title
// Mak Si Jie, Harold Mah
// 2516477, 2517100
// DCEP/FT/1A/09
// Description


#include <math.h>
#include <Wire.h>
#include "PCA9685.h"
#include "RichShieldLightSensor.h"
#include "RichShieldTM1637.h"
#include "RichShieldNTC.h"

#define LED_YELLOW 7
#define BUTTON_K1 8
#define BUTTON_K2 9
#define NTC_PIN A1 //SIG pin of NTC module connect to A1 of IO Shield, that is pin A1 of OPEN-SMART UNO R3
#define CLK 10//CLK of the TM1637 IC connect to D10 of OPEN-SMART UNO R3
#define DIO 11
#define LIGHTSENSOR_PIN A2

NTC temper(NTC_PIN);
TM1637 disp(CLK,DIO);
LightSensor lightsensor(LIGHTSENSOR_PIN); 
PCA9685 pwmController(Wire);
PCA9685_ServoEval pwmServo1;

int toggle_blue = 0;
int toggle_yellow = 0;
int i; //for motor loop

void setup() {
Serial.begin(9600);
pinMode(LED_YELLOW, OUTPUT); 
pinMode(BUTTON_K1, INPUT_PULLUP);
pinMode(BUTTON_K2, INPUT_PULLUP);
disp.init(); 
delay(500);
pwmController.resetDevices();  
pwmController.init();
pwmController.setPWMFreqServo();
pwmController.setChannelPWM(0, pwmServo1.pwmForAngle(-10));
pwmController.setChannelPWM(1, pwmServo1.pwmForAngle(0));
delay(1000);
}

void loop() 
{ 
//--------Light Dependant LED---------------------
float Rsensor = lightsensor.getRes(); 
float lux;

//Make yellow button toggle the light sensor
if (digitalRead(BUTTON_K2) == 0) 
{
  if (toggle_yellow == 0)
    toggle_yellow = 1;
  else 
    toggle_yellow = 0;
  delay (300);
  while (digitalRead(BUTTON_K2) == 0);
}

if (toggle_yellow == 1) //When yellow button is toggled on, light sensor is on
{
//Read light level
  lux = 325*pow(Rsensor,-1.4); //Calculate lux value
  
  Serial.print("Illuminance is almost "); 
  Serial.print(lux,1);
  Serial.println(" lux");
  delay(250);  

//Conditions for LED to turn on
  if (lux <= 100) 
    digitalWrite(LED_YELLOW, HIGH); 
  else
    digitalWrite(LED_YELLOW, LOW);

}
else 
digitalWrite(LED_YELLOW, LOW);

//-------Temperature Dependant Fan-------------------
float celsius;
celsius = temper.getTemperature();//get temperature
displayTemperature((int8_t)celsius);
delay(0);

//Make blue button toggle the temperature sensor
if (digitalRead(BUTTON_K1) == 0) 
{
  if (toggle_blue == 0)
    toggle_blue = 1;
  else 
    toggle_blue = 0;
  delay (300);
  while (digitalRead(BUTTON_K1) == 0);
}

if (toggle_blue == 1) //When blue button is toggled on, temperature sensor is on
{
  if (celsius > 26)
  {
    //Lower Servo (Channel 1) Control

    for (i = 0; i <= 90; i += 5) { //Slow Turn anti-clockwise (from 0 to 90 degree
        pwmController.setChannelPWM(1, pwmServo1.pwmForAngle(i));
        delay(250);  //longer delay for Slow turn movement
    }

    for (i = 90; i >= 0; i -= 5) { //Fast Turn clockwise (from 90 to 0 degree)
        pwmController.setChannelPWM(1, pwmServo1.pwmForAngle(i));
        delay(100); //shorter delay for faster turn movement
    }

    for (i = 0; i >= -90; i -= 5) { //Slow Turn clockwise (from 0 to -90 degree)
        pwmController.setChannelPWM(1, pwmServo1.pwmForAngle(i));
        delay(250); //longer delay for Slow turn movement
    }

    for (i = -90; i <= 0; i += 5) { //Fast Turn anti-clockwise (from -90 to 0 degree)
        pwmController.setChannelPWM(1, pwmServo1.pwmForAngle(i));
        delay(100); //shorter delay for faster turn movement
    }

    //Upper servo (Channel 0) control

    for (i = -10; i <= 90; i += 5) { //Slow Turn going downward (from -10 to 90 degree
        // upper servo (degree of turn is limited (from -10 degree)

        pwmController.setChannelPWM(0, pwmServo1.pwmForAngle(i));
        delay(250); //longer delay for Slow turn movement
    }

    for (i = 90; i >= -10; i -= 5) { //Fast Turn raising up (from 90 to -10 degree)
        pwmController.setChannelPWM(0, pwmServo1.pwmForAngle(i));//upper servo
        delay(100);  //shorter delay for faster turn movement
    }
  }
  else
  pwmController.resetDevices();
}
else 
pwmController.resetDevices();
}

void displayTemperature(int8_t temperature)
{
  int8_t temp[4];
  if(temperature < 0)
	{
		temp[0] = INDEX_NEGATIVE_SIGN;
		temperature = abs(temperature);
	}
	else if(temperature < 100)temp[0] = INDEX_BLANK;
	else temp[0] = temperature/100;
	temperature %= 100;
	temp[1] = temperature / 10;
	temp[2] = temperature % 10;
	temp[3] = 12;	          //index of 'C' for celsius degree symbol.
	disp.display(temp);
}

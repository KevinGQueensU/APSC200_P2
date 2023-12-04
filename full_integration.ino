//neopixel defintions
#include <Adafruit_NeoPixel.h>

#define NEO_PIN1 11
#define NEO_PIN2 0
#define NUMPIXELS 10

//defining neopixels for sensor data
Adafruit_NeoPixel pixels_1(NUMPIXELS, NEO_PIN1, NEO_GRB + NEO_KHZ800);

//defining neopixels for historical data 
Adafruit_NeoPixel pixels_2(NUMPIXELS, NEO_PIN2, NEO_GRB + NEO_KHZ800);

//dust sensor defintions
#define        COV_RATIO                       0.2            //ug/mmm / mv
#define        NO_DUST_VOLTAGE                 400            //mv
#define        SYS_VOLTAGE                     5000 

const int iled = 7;                                            //drive the led of sensor
const int vout = 0;                                            //analog input 
float density, voltage;
int   adcvalue;

//gas sensor defintions
#if (ARDUINO >= 100)
    #include "Arduino.h"
#else
    #include <avr/io.h>
    #include "WProgram.h"
#endif

#include "MQ7.h"

MQ7::MQ7(uint8_t pin, float v_input){
  analogPin = pin;
  v_in = v_input;
}

float MQ7::getPPM(){
  return (float)(coefficient_A * pow(getRatio(), coefficient_B));
}

float MQ7::voltageConversion(int value){
  return (float) value * (v_in / 1023.0);
}

float MQ7::getRatio(){
  int value = analogRead(analogPin);
  float v_out = voltageConversion(value);
  return (v_in - v_out) / v_out;
}

float MQ7::getSensorResistance(){
  return R_Load * getRatio();
}

MQ7 mq7(A1,5.0);

//button and led defintions
const int buttonPin = 2; // the number of the pushbutton pin

const int led0_pin = 8;
const int led1_pin = 9;
const int led2_pin = 10;

int buttonState = 0; // variable for reading the pushbutton status
int lastButtonState = 0; // variable to check the last state
int counter = 0; // counter for the number of button presses
int caseNumber = 0; // control variable for neopixel display

void setup() {
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
  
  //Set up the pins as output voltages
  pinMode(led0_pin, OUTPUT);
  pinMode(led1_pin, OUTPUT);
  pinMode(led2_pin, OUTPUT);
  
  // initialize serial communication:
  Serial.begin(9600);

  //starting neopixels
  pixels_1.begin();
  pixels_2.begin();

}

void loop() {
  //read sensors
  //reading dust sensor
  digitalWrite(iled, HIGH);
  delayMicroseconds(280);
  adcvalue = analogRead(vout);
  digitalWrite(iled, LOW);
  
  adcvalue = Filter(adcvalue);
  
  voltage = (SYS_VOLTAGE / 1024.0) * adcvalue * 11;
  
  if(voltage >= NO_DUST_VOLTAGE)
  {
    voltage -= NO_DUST_VOLTAGE;
    
    density = voltage * COV_RATIO;
  }
  else
    density = 0;

  //mapping dust
  double dust_map = map(density, 100, 700, 1, 10);

  //readuing gas sensor and converting to ppm with function at bottom
  float gas=mq7.getPPM();

  //mapping gas
  double gas_map = map(gas, 1, 5, 1, 10);

  //averaging led indices from both sensors
  int led_index = map(dust_map + gas_map, 0, 20, 0, 10);

  //test prints
  Serial.print("Density ug/cm3: ");
  Serial.println(density);
  Serial.print("Gas ppm: ");
  Serial.println(gas);
  Serial.print("LED Index: ");
  Serial.println(led_index);

  //clear pixels
  pixels_1.clear();
  pixels_2.clear();

  //clear leds
  digitalWrite(led0_pin, LOW);
  digitalWrite(led1_pin, LOW);
  digitalWrite(led2_pin, LOW);

  //read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  Serial.print("Button State: ");
  Serial.println(buttonState);
  Serial.print("Counter: ");
  Serial.println(counter);

  if(buttonState == HIGH && lastButtonState == LOW)
    counter++;
  if(counter==3)
    counter = 0;

  //set neopixels from sensors
  for(int i = 0; i < led_index; i++){
    if(i < 3){
      pixels_1.setPixelColor(i, pixels_1.Color(0, 255, 0));
    }else if(2 < i && i < 7){
      pixels_1.setPixelColor(i, pixels_1.Color(255, 255, 0));
    }else if ( i > 6){
      pixels_1.setPixelColor(i, pixels_1.Color(255, 0, 0));
    }
  }

  //set neopixels from historical data
  caseNumber = counter;
  switch(caseNumber) {
    case 0: // 8 pixels light up
        for(int i=0; i<3; i++) pixels_2.setPixelColor(i, pixels_2.Color(0, 255, 0)); // Green
        for(int i=3; i<6; i++) pixels_2.setPixelColor(i, pixels_2.Color(255, 200, 0)); // Yellow
        for(int i=6; i<8; i++) pixels_2.setPixelColor(i, pixels_2.Color(255, 0, 0)); // Red
        digitalWrite(led0_pin, HIGH);
        break;
      case 1: // 6 pixels light up
        for(int i=0; i<3; i++) pixels_2.setPixelColor(i, pixels_2.Color(0, 255, 0)); // Green
        for(int i=3; i<6; i++) pixels_2.setPixelColor(i, pixels_2.Color(255, 200, 0)); // Yellow
        digitalWrite(led1_pin, HIGH);
        break;
      case 2: // 4 pixels light up
        for(int i=0; i<3; i++) pixels_2.setPixelColor(i, pixels_2.Color(0, 255, 0)); // Green
        for(int i=3; i<4; i++) pixels_2.setPixelColor(i, pixels_2.Color(255, 200, 0)); // Yellow
        digitalWrite(led2_pin, HIGH);
        break;
      }

  lastButtonState = buttonState;

  //show pixels
  pixels_1.show();
  pixels_2.show();

  delay(1000);
}

//filtering dust reading
int Filter(int m)
{
  static int flag_first = 0, _buff[10], sum;
  const int _buff_max = 10;
  int i;
  
  if(flag_first == 0)
  {
    flag_first = 1;

    for(i = 0, sum = 0; i < _buff_max; i++)
    {
      _buff[i] = m;
      sum += _buff[i];
    }
    return m;
  }
  else
  {
    sum -= _buff[0];
    for(i = 0; i < (_buff_max - 1); i++)
    {
      _buff[i] = _buff[i + 1];
    }
    _buff[9] = m;
    sum += _buff[9];
    
    i = sum / 10.0;
    return i;
  }
}

//function to convert analog reading
float getPPM(float value){
   float sensor_volt = value/1024*5.0;
   float RS_gas = (5.0-sensor_volt)/sensor_volt;
   float ratio = RS_gas/102; 
   float x = 1538.46 * ratio;
   float ppm = pow(x,-1.709);
   return ppm;
}

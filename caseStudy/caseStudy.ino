#include <math.h>
#include <String.h>

#define TRIGGER_PIN 12 // Ultrasound go
#define ECHO_PIN 11 // Ultrasound come

#define P_GAIN 3
#define R0_THERMISTOR 10000
#define T0 298.15
#define R_S 10000
#define MAX_PWM 511
#define V_IN 4.96
#define PWM_PIN 9
#define V_OUT_PIN 3
#define B 3997

const unsigned int MAX_DELAY = 23325;
const float SOUND_V = 343;
const int VAL_PROBE = 0; // Analog pin 0
const int MOISTURE_LEVEL = 250; // the value after the LED goes ON

float getTem()
{
  float vout = analogRead(V_OUT_PIN) * V_IN / 1023.0;
  float rThermistor = R_S *(V_IN / vout) - R_S;
  float tem;
  tem = B * T0 / ( B + T0 * log(rThermistor/R0_THERMISTOR) ) - 273.15;
  return tem;
}
void thermistorInit()
{
  TCCR1A = 0; // reset the register
  TCCR1B = 0; // reset the register
  TCNT1 = 0;
  TCCR1A = 0b10100010; // COM1A0 & COM1B0 are 0, COM1A1 & COM1B1 are 1
  TCCR1B = 0b00001101; // WGM13 and WGM12 are 0 with pre-scaler 1024
}

void ultrasonicInit()
{
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIGGER_PIN, LOW);
}

float getDistance()
{
    unsigned long t1;
    unsigned long t2;
    unsigned long roundTripTime;
    digitalWrite(TRIGGER_PIN,HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN,LOW);

    while (digitalRead(ECHO_PIN) == 0)
        ;
    t1 = micros();
    while (digitalRead(ECHO_PIN) == 1)
        ;
    t2 = micros();
    roundTripTime = t2 - t1;
    float distance;
    if(roundTripTime > MAX_DELAY){
          distance = 400;
     }
     else{
        distance = SOUND_V * (t2 - t1) / 20000;
     }
     return distance;
}

void setup() {
    Serial.begin(9600);
    thermistorInit();
    ultrasonicInit();
}
 

 
void loop() {
    int moisture = analogRead(VAL_PROBE);
    moisture = map(moisture, 0, 880, 0, 100);
    Serial.print(moisture);
    Serial.print(',');
    Serial.print(getTem());
    Serial.print(',');
    Serial.print(getDistance());
    Serial.print('\n');
    
    delay(100);
}
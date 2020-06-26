#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define encoderPin1 3
#define encoderPin2 2
#define swPin 4 //encoder switch
#define rampsPin 5 //ramps setpoint PCB mark "R"
#define pumpPin 6 // Pump motor  PCB mark "M"
#define sensorPin A3 // buffer pressure sensor  PCB mark "P"
#define maxPressureSetpoint 3.5

void readPressure();
void setPointMode();
void readRampsSetPoint();
void pumpControl();
void screen();
void i2cReceivePressure();
void i2cSendSetPoint();
void updateEncoder();

volatile int lastEncoded = 0;
volatile long encoderValue = 0;
int lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
unsigned long lastRunSetpoin;

int rawSetPoint;
int sw = 1;
double bufferPressure, pressure, setpoint;

Adafruit_SSD1306 display(-1);

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  //Serial.begin(9600);
  
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);
  pinMode(rampsPin, INPUT);
  pinMode(sensorPin, INPUT);
  pinMode(swPin, INPUT_PULLUP);
  pinMode(pumpPin, OUTPUT);
  lastRunSetpoin = millis();

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);
}


void loop() {
  if (millis() - lastRunSetpoin > 50) {
        setPointMode();
        lastRunSetpoin = millis();
    }
  readPressure();
  i2cReceivePressure();
  i2cSendSetPoint();
  pumpControl();
  //Serial.print("rawsetpoint: ");
  //Serial.println(encoderValue);
  //Serial.print("SW state:  ");
  //Serial.println(sw);
  screen();
  //delay(3);
  
}

void readPressure() 
{
  float fullScale = 9630.; // max possible value of analogRead
  float offset = 360.;
  float rawValue = 0; 
  for (int x = 0; x < 10; x++) rawValue = rawValue + analogRead(sensorPin);
  bufferPressure = (rawValue - offset) * 7.0 / (fullScale - offset); // pressure conversion
      
}

void screen(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setTextColor(INVERSE);
  display.setCursor(0, 0);
  display.print(F("now: "));
  display.setCursor(68, 0);
  display.print(pressure, 2);
  display.setCursor(0, 17);
  display.print(F("set: "));
  display.setCursor(68, 17);
  display.print(setpoint, 2);
  display.setCursor(0,34);
  display.print(F("mode: "));
  display.setCursor(0, 50);
  if (sw == 1){
    display.print(F("manual"));
  }
  else{
    display.print(F("automatic"));
  }
  display.display();
}

void pumpControl()
{
    if (bufferPressure < 2.5){  
        digitalWrite(pumpPin, LOW);
    }
    else if (bufferPressure >= 3.5) 
    {
        digitalWrite(pumpPin, HIGH);
    }
}
   
void setPointMode(){
  if (!(digitalRead(swPin))){
    if (sw == 1){
      sw=0;
    }
    else{
      sw=1;
    }
  }
  if (sw == 1){ // manual mode
    rawSetPoint = encoderValue;
  }
  else{ //auto mode
    readRampsSetPoint();
  }
  setpoint = float(rawSetPoint) * maxPressureSetpoint / 255.;
}

void readRampsSetPoint(){
  int pwm_value = pulseIn(rampsPin, HIGH);
  int s = map(pwm_value,0,2019,0,255);
  //float s = float(pwm_value) * maxPressureSetpoint/ 2019.;
  rawSetPoint = s;
 }

void i2cSendSetPoint()
{
    Wire.beginTransmission(43);// transmit to device #43
    Wire.write(rawSetPoint);      // sends one byte
    Wire.endTransmission();    // stop transmitting// send the current setPoint, between 0 and 255
}

void i2cReceivePressure()  
{
    Wire.requestFrom(43,1);
    uint8_t raw_pressure_ = Wire.read();
    pressure = float(raw_pressure_) * maxPressureSetpoint / 255.;    
}
 
void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2 ) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011){
      if (encoderValue >= 255){
      encoderValue = 255;
    }
    else{
      encoderValue ++;
    }    
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    if (encoderValue <= 0) {
       encoderValue = 0;
    }
    else {
      encoderValue --;
    }
  }
  lastEncoded = encoded; //store this value for next time
}
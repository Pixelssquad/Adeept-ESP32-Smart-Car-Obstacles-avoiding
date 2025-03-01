#include <Arduino.h>
#include "Adeept_Car_For_ESP32.h"
#include <Wire.h>
#include <SSD1306Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>


/////////////////////PCA9685 drive area///////////////////////////////////
#define PCA9685_SDA 13         //Define SDA pins
#define PCA9685_SCL 14         //Define SCL pins

#ifndef PCA9685_ADDRESS
#define PCA9685_ADDRESS   0x40
#endif
#define SERVO_FREQUENCY    50        //Define the operating frequency of servo
#define PCA9685_CHANNEL_0  0         //Define PCA9685 channel to control servo 1
#define PCA9685_CHANNEL_1  1         //Define the PCA9685 channel to control servo 2
#define PCA9685_CHANNEL_2  2         //Define the PCA9685 channel to control servo 3
#define SERVO_MIDDLE_POINT 1500      //Define the middle position of the servo   
#define MOTOR_SPEED_MIN   -4095      //Define a minimum speed limit for wheels
#define MOTOR_SPEED_MAX   4095       //Define a maximum speed limit for wheels

#define PIN_MOTOR_M1_IN1 15      //Define the positive pole of M1
#define PIN_MOTOR_M1_IN2 14      //Define the negative pole of M1
#define PIN_MOTOR_M2_IN1 12       //Define the positive pole of M2
#define PIN_MOTOR_M2_IN2 13       //Define the negative pole of M2
#define PIN_MOTOR_M3_IN1 11      //Define the positive pole of M3
#define PIN_MOTOR_M3_IN2 10      //Define the negative pole of M3
#define PIN_MOTOR_M4_IN1 8      //Define the positive pole of M4
#define PIN_MOTOR_M4_IN2 9      //Define the negative pole of M4

/////////////////////////Matrix///////////////////////
#define SCL_Pin  12  //Set clock pin to 12
#define SDA_Pin  15  //Set data pin to 15

/////////////////////Ultrasonic drive area//////////////////////////////
#define PIN_SONIC_TRIG    12            //define Trig pin
#define PIN_SONIC_ECHO    15            //define Echo pin
#define MAX_DISTANCE      300           //cm
#define SONIC_TIMEOUT (MAX_DISTANCE*60) // calculate timeout 
#define SOUND_VELOCITY    340           //soundVelocity: 340m/s

//////////////Photosensitive/////////////////
#define PHOTOSENSITIVE_PIN 33 //Define the pins that ESP32 reads photosensitive

/////////////////////PCF8574 drive area//////////////////////////////
#define PCF8574_ADDRESS 0x23     //Tracking module IIC address
#define PCF8574_SDA     13       //Define the SDA pin number
#define PCF8574_SCL     14       //Define the SCL pin number
unsigned char sensorValue[4] = {0};
PCF8574 TRACK_SENSOR(PCF8574_ADDRESS);

////////////////OLED//////////////////
#define SDA   13
#define SCL   14
SSD1306Wire OLED_display(0x3c, SDA, SCL);

////////////////////Battery drive area/////////////////////////////////////
float batteryVoltage = 0;       //Battery voltage variable
float batteryCoefficient = 4;   //Set the proportional coefficient


PCA9685 pca9685;//Instantiate a PCA9685 object

//PCA9685 initialization
void PCA9685_Setup(void)
{
  Wire.begin(PCA9685_SDA, PCA9685_SCL);  
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  pca9685.setupSingleDevice(Wire, PCA9685_ADDRESS);
  pca9685.setToFrequency(SERVO_FREQUENCY);
}

////////////////////Servo///////////////////////
//Set the rotation parameters of servo 1, and the parameters are 0-180 degrees
void Servo_1_Angle(float angle)
{
  angle = constrain(angle, 0, 180);
  angle = map(angle, 0, 180, 102, 512);
  pca9685.setChannelPulseWidth(PCA9685_CHANNEL_0, int(angle));
}

//Set the rotation parameters of servo 2, and the parameters are 0-180 degrees
void Servo_2_Angle(float angle)
{
  angle = constrain(angle, 0, 180);
  angle = map(angle, 0, 180, 102, 512);
  pca9685.setChannelPulseWidth(PCA9685_CHANNEL_1, int(angle));
}

// Set the rotation parameters of servo 2, and the parameters are 0-180 degrees
void Servo_3_Angle(float angle)
{
  angle = constrain(angle, 0, 180);
  angle = map(angle, 0, 180, 102, 512);
  pca9685.setChannelPulseWidth(PCA9685_CHANNEL_2, int(angle));
}

//Servo sweep function
void Servo_Sweep(int servo_id, int angle_start, int angle_end)
{
    angle_start = constrain(angle_start, 0, 180);
    angle_end = constrain(angle_end, 0, 180);

  if (angle_start > angle_end)
  {
    for (int i = angle_start; i >= angle_end; i--)
    {
      if (servo_id == 1)
        Servo_1_Angle(i);
      else if (servo_id == 2)
        Servo_2_Angle(i);
      else if (servo_id == 3)
        Servo_3_Angle(i);
      delay(10);
    }
  }
  if (angle_start < angle_end)
  {
    for (int i = angle_start; i <= angle_end; i++)
    {
      if (servo_id == 1)
        Servo_1_Angle(i);
      else if (servo_id == 2)
        Servo_2_Angle(i);
      delay(10);
    }
  }
}


//////////////////////Motor////////////////////////////
// A function to control the car motor
void Motor(int Motor_ID, int dir, int Motor_speed)
{
  if(dir > 0){dir = 1;}
  else {dir = -1;} 
  Motor_speed = constrain(Motor_speed, 0, 100);
  Motor_speed = map(Motor_speed, 0,100, 0,4095);

  if(Motor_ID == 1)
  {
    if(dir == 1){
      pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN1, Motor_speed);
      pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN2, 0);
    }
    else{
      pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN1, 0);
      pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN2, Motor_speed);
      }
    }
  else if(Motor_ID == 2)
   {
    if(dir == 1){
      pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN1, 0);
      pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN2, Motor_speed);
    }
    else{
      pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN1, Motor_speed);
      pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN2, 0);
      }
    }
  else if(Motor_ID == 3)
   {
    if(dir == 1){
      pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN1, Motor_speed);
      pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN2, 0);
    }
    else{
      pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN1, 0);
      pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN2, Motor_speed);
      }
    }
   else if(Motor_ID == 4)
   {
    if(dir == 1){
      pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN1, Motor_speed);
      pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN2, 0);
    }
    else{
      pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN1, 0);
      pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN2, Motor_speed);
      }
    }
}

//A function to control the car motor
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed) {
  m1_speed = MOTOR_1_DIRECTION * constrain(m1_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX); // Adjust the direction of the motor.
  m2_speed = MOTOR_2_DIRECTION * constrain(m2_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m3_speed = MOTOR_3_DIRECTION * constrain(m3_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  m4_speed = MOTOR_4_DIRECTION * constrain(m4_speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

  if (m1_speed >= 0) {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN1, m1_speed);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN2, 0);
  } else {
    m1_speed = -m1_speed;
    pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN1, 0);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M1_IN2, m1_speed);
  }
  if (m2_speed >= 0) {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN1, m2_speed);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN2, 0);
  } else {
    m2_speed = -m2_speed;
    pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN1, 0);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M2_IN2, m2_speed);
  }
  if (m3_speed >= 0) {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN1, m3_speed);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN2, 0);
  } else {
    m3_speed = -m3_speed;
    pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN1, 0);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M3_IN2, m3_speed);
  }
  if (m4_speed >= 0) {
    pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN1, m4_speed);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN2, 0);
  } else {
    m4_speed = -m4_speed;
    pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN1, 0);
    pca9685.setChannelPulseWidth(PIN_MOTOR_M4_IN2, m4_speed);
  }
}

//////////////////////Buzzer drive area///////////////////////////////////
//Buzzer pin definition             
#define PIN_BUZZER 2                    //Define the pins for the ESP32 control buzzer.
#define BUZZER_CHN 0                    //Define the PWM channel for ESP32.
#define BUZZER_FREQUENCY 2000           //Define the resonant frequency of the buzzer.

//Buzzer initialization
void Buzzer_Setup(void)
{
  pinMode(PIN_BUZZER, OUTPUT);
  ledcSetup(BUZZER_CHN, BUZZER_FREQUENCY, 10);
  ledcAttachPin(PIN_BUZZER, BUZZER_CHN);
  ledcWriteTone(BUZZER_CHN, 0);
  delay(10);
}

//Buzzer Alert function
void Buzzer_Alarm(bool enable)
{
  if (enable == 0)
    ledcWriteTone(BUZZER_CHN, 0);
  else
    ledcWriteTone(BUZZER_CHN, BUZZER_FREQUENCY);
}

//Buzzer alarm function
void Buzzer_Alert(int beat, int rebeat)
{
  beat = constrain(beat, 1, 9);
  rebeat = constrain(rebeat, 1, 255);
  for (int j = 0; j < rebeat; j++)
  {
    for (int i = 0; i < beat; i++)
    {
      ledcWriteTone(BUZZER_CHN, BUZZER_FREQUENCY);
      delay(100);
      ledcWriteTone(BUZZER_CHN, 0);
      delay(100);
    }
    delay(500);
  }
  ledcWriteTone(BUZZER_CHN, 0);
}

/////////////////////Matrix////////////////////////////////
void matrix_setup(){
  pinMode(SCL_Pin,OUTPUT);
  pinMode(SDA_Pin,OUTPUT);
  delay(10);
  }
void matrix_display(unsigned char matrix_value[])
{
  IIC_start();  // use the function of the data transmission start condition
  IIC_send(0xc0);  //select address
  for(int i = 0;i < 16;i++) //pattern data has 16 bits
  {
     IIC_send(matrix_value[i]); //convey the pattern data
  }
  IIC_end();   //end the transmission of pattern data  
  IIC_start();
  IIC_send(0x8A);  //display control, set pulse width to 4/16 s
  IIC_end();
}

//the condition to start conveying data
void IIC_start()
{
  digitalWrite(SCL_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,LOW);
  delayMicroseconds(3);
}
//Convey data
void IIC_send(unsigned char send_data)
{
  for(char i = 0;i < 8;i++)  //Each byte has 8 bits 8bit for every character
  {
      digitalWrite(SCL_Pin,LOW);  // pull down clock pin SCL_Pin to change the signal of SDA
      delayMicroseconds(3);
      if(send_data & 0x01)  //set high and low level of SDA_Pin according to 1 or 0 of every bit
      {
        digitalWrite(SDA_Pin,HIGH);
      }
      else
      {
        digitalWrite(SDA_Pin,LOW);
      }
      delayMicroseconds(3);
      digitalWrite(SCL_Pin,HIGH); //pull up the clock pin SCL_Pin to stop transmission
      delayMicroseconds(3);
      send_data = send_data >> 1;  // detect bit by bit, shift the data to the right by one
  }
}

//The sign of ending data transmission
void IIC_end()
{
  digitalWrite(SCL_Pin,LOW);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,HIGH);
  delayMicroseconds(3);
}

/////////////////////////////Ultrasonic /////////////////////
//Ultrasonic initialization
void Ultrasonic_Setup(void)
{
  pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
  pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode
}

//Obtain ultrasonic distance data
float Get_Sonar(void) 
{
  unsigned long pingTime;
  float distance;
  digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high level lasting for 10μs to triger HC_SR04,
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waitting time
  if (pingTime != 0)
    distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
  else
    distance = MAX_DISTANCE;
  return distance; // return the distance value
}

/////////////////////Photosensitive//////////////////////////////
void photosensitive_Setup()
{
  pinMode(PHOTOSENSITIVE_PIN, INPUT);//Configure the pins for input mode
}


//Trace module initialization
void Track_Setup(void)
{
  TRACK_SENSOR.begin(PCF8574_SDA, PCF8574_SCL);
}

//Tracking module reading
void Track_Read(void)
{
  sensorValue[3] = (TRACK_SENSOR.read8() & 0x07);//composite value
  sensorValue[2] = (sensorValue[3] & 0x01) >> 0; //On the left - 1
  sensorValue[1] = (sensorValue[3] & 0x02) >> 1; //In the middle - 2
  sensorValue[0] = (sensorValue[3] & 0x04) >> 2; //On the right - 4
}
//OLED
void OLED_Setup(){
  OLED_display.init();
  OLED_display.flipScreenVertically();
  }
  
void OLED_clear(){
  OLED_display.clear();
  }
  
void OLED(int font, int x, int y, char text[]){
  if (font > 3){
    font = 3;}
  else if(font <1){
    font = 1;}
  if (font == 1){
    OLED_display.setFont(ArialMT_Plain_10);
    OLED_display.drawString(x,y,text);
    OLED_display.display();
    }
  else if (font == 2){
    OLED_display.setFont(ArialMT_Plain_16);
    OLED_display.drawString(x,y,text);
    OLED_display.display();
  }
  else {
    OLED_display.setFont(ArialMT_Plain_24);
    OLED_display.drawString(x,y,text);
    OLED_display.display();
  }
}


//Gets the battery ADC value
int Get_Battery_Voltage_ADC(void)
{
  pinMode(PIN_BATTERY, INPUT);
  int batteryADC = 0;
  for (int i = 0; i < 5; i++)
    batteryADC += analogRead(PIN_BATTERY);
  return batteryADC / 5;
}

//Get the battery voltage value
float Get_Battery_Voltage(void)
{
  int batteryADC = Get_Battery_Voltage_ADC();
  batteryVoltage = (batteryADC / 4096.0  * 3.9 ) * batteryCoefficient;
  return batteryVoltage;
}

void Set_Battery_Coefficient(float coefficient)
{
  batteryCoefficient = coefficient;
}

///////////////////Camera drive area///////////////////////////////////
//framesize_t frame_size   =   FRAMESIZE_CIF;      //The default is to use the image size of FRAMESIZE CIF
//Camera initialization
bool cameraSetup(void)
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_1;//From modification
  config.ledc_timer = LEDC_TIMER_1;//From modification
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  /*frame_size
  FRAMESIZE_UXGA (1600 x 1200)
  FRAMESIZE_QVGA (320 x 240)
  FRAMESIZE_CIF (352 x 288)
  FRAMESIZE_VGA (640 x 480)
  FRAMESIZE_SVGA (800 x 600)
  FRAMESIZE_XGA (1024 x 768)
  FRAMESIZE_SXGA (1280 x 1024)
*/
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return 0;
  }
  Serial.println("Camera configuration complete!");
  return 1;
}
//Set the camera to flip up and down
void camera_vflip(bool enable)
{
  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, enable);
}
//Set the camera to flip left and right
void camera_hmirror(bool enable)
{
  sensor_t * s = esp_camera_sensor_get();
  s->set_hmirror(s, enable);
}

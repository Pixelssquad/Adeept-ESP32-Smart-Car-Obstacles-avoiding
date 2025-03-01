#ifndef _Adeept_CAR_H
#define _Adeept_CAR_H

#include <Arduino.h>
#include <PCA9685.h>
#include <PCF8574.h>
#include "esp_camera.h"

#define MOTOR_1_DIRECTION     1 //If the direction is reversed, change 1 to -1
#define MOTOR_2_DIRECTION     1 //If the direction is reversed, change 1 to -1
#define MOTOR_3_DIRECTION     1 //If the direction is reversed, change 1 to -1
#define MOTOR_4_DIRECTION     1 //If the direction is reversed, change 1 to -1

void PCA9685_Close_Com_Address(void);//Close the PCA9685 public address

/////////////////////PCA9685 drive area//////////////////////////////////////
void PCA9685_Setup(void);              //servo initialization

void Servo_1_Angle(float angle);     //Set the rotation parameters of servo 1, and the parameters are 0-180 degrees
void Servo_2_Angle(float angle);     //Set the rotation parameters of servo 2, and the parameters are 0-180 degrees
void Servo_3_Angle(float angle);     //Set the rotation parameters of servo 2, and the parameters are 0-180 degrees
void Servo_Sweep(int servo_id, int angle_start, int angle_end);//Servo sweep function;
// Motor
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed);//A function to control the car motor
void Motor(int Motor_ID, int dir, int Motor_speed);//A function to control the car motor

//////////////////////Buzzer drive area///////////////////////////////////

void Buzzer_Setup(void);                //Buzzer initialization.
void Buzzer_Alarm(bool enable);
void Buzzer_Alert(int beat, int rebeat);//Buzzer alarm function.


/////////////////////////matrix////////////////////////////
void matrix_setup();
void matrix_display(unsigned char matrix_value[]);  //matrix function.
void IIC_start();
void IIC_send(unsigned char send_data);
void IIC_end();

/////////////////////Ultrasonic drive area/////////////////////////////////
void Ultrasonic_Setup(void);//Ultrasonic initialization.
float Get_Sonar(void);//Obtain ultrasonic distance data.

/////////////////////Photosensitive//////////////////////////////
void photosensitive_Setup();

////////////////////PCF8574 drive area//////////////////////////////
extern unsigned char sensorValue[4];
void Track_Setup(void);               //Trace module initialization.
void Track_Read(void);                //Tracking module reading.

/////////////////////OLED /////////////////////
void OLED_Setup();
void OLED_clear();
void OLED(int font, int x, int y, char *text);


////////////////////Battery drive area/////////////////////////////////////
#define PIN_BATTERY        32        //Set the battery detection voltage pin.
#define LOW_VOLTAGE_VALUE  2100      //Set the minimum battery voltage.

extern float batteryCoefficient;    //Set the proportional coefficient.

int Get_Battery_Voltage_ADC(void);   //Gets the battery ADC value.
float Get_Battery_Voltage(void);     //Get the battery voltage value.
void Set_Battery_Coefficient(float coefficient);//Set the partial pressure coefficient.


///////////////////Camera drive area////////////////////////////////////////
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

//extern framesize_t frame_size;
bool cameraSetup(void);                    //Camera initialization
void camera_vflip(bool enable);            //Set the camera to flip up and down
void camera_hmirror(bool enable);          //Set the camera to flip left and right

//void loopTask_WTD(void *pvParameters);
#endif

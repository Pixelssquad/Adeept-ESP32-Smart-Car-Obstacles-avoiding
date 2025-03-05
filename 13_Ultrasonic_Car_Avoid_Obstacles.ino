/**********************************************************************
  Description : Ultrasonic Avoid Obstacles.
  Author      : www.Adeept.com
  Modification: 2025/03/1 by Diego "Diegooz" Pellacani
**********************************************************************/

#include "Adeept_Car_For_ESP32.h"
#include "Adeept_WS2812_for_ESP32.h"


/////////////WS2812 LED ////////////
#define LEDS_COUNT   7     //Define the count of WS2812,When adding WS2812 LED, you can modify this value.
#define LEDS_PIN    32    //Define the pin number for ESP32
#define CHANNEL     0     //Define the channels that control WS2812
Adeept_ESP32_WS2812 strip = Adeept_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

float distance;
float midDist;
float leftDist;
float rightDist;
int avoid_Dist = 128; // cm
int minDist = 110; // cm

int wheel_Steering = 40;
int deviation = 0;
int servo_Init = 90;
int Speed =35          ;  // value:0-100
int steering_Speed = 80;  // value:0-100

void setup() {
  Serial.begin(115200); //Open the serial port and set the baud rate to 115200
  PCA9685_Setup();       //Initializes the chip that controls the motor
  Ultrasonic_Setup();  //Ultrasonic module initialization.
  // Added some light effects
  strip.begin();      //Initialize WS2812.
  strip.setBrightness(2); //Set the brightness of WS2812（0-255）.
  strip.set_pixel(0, 0, 0, 255); 
  strip.set_pixel(1, 0, 0, 0); 
  strip.set_pixel(2, 0, 0, 0); 
  strip.set_pixel(3, 0, 0, 255); 
  strip.set_pixel(4, 0, 255, 0);
  strip.set_pixel(5, 255, 0, 0);
  strip.set_pixel(6, 0, 255, 0);
  strip.show();
  delay(3000);       
    strip.set_pixel(0, 0, 0, 0); 
  strip.set_pixel(1, 0, 128, 255); 
  strip.set_pixel(2, 0, 128, 255); 
  strip.set_pixel(3, 0, 0, 0); 
  strip.set_pixel(4, 0, 255, 0);
  strip.set_pixel(5, 255, 0, 0);
  strip.set_pixel(6, 0, 255, 0);
  strip.show();
    delay(3000); 
    strip.set_pixel(0, 0, 0, 255); 
  strip.set_pixel(1, 0, 0, 0); 
  strip.set_pixel(2, 0, 0, 0); 
  strip.set_pixel(3, 0, 0, 255); 
  strip.set_pixel(4, 0, 255, 0);
  strip.set_pixel(5, 255, 0, 0);
  strip.set_pixel(6, 0, 255, 0);
  strip.show();
  delay(1000);        
    strip.set_pixel(0, 0, 0, 0); 
  strip.set_pixel(1, 0, 128, 255); 
  strip.set_pixel(2, 0, 128, 255); 
  strip.set_pixel(3, 0, 0, 0); 
  strip.set_pixel(4, 0, 255, 0);
  strip.set_pixel(5, 255, 0, 0);
  strip.set_pixel(6, 0, 255, 0);
  strip.show();
    delay(1000);   
      strip.set_pixel(0, 0, 0, 255); 
  strip.set_pixel(1, 0, 0, 0); 
  strip.set_pixel(2, 0, 0, 0); 
  strip.set_pixel(3, 0, 0, 255); 
  strip.set_pixel(4, 0, 255, 0);
  strip.set_pixel(5, 255, 0, 0);
  strip.set_pixel(6, 0, 255, 0);
  strip.show();
  delay(500);          
  strip.set_pixel(0, 0, 0, 0); 
  strip.set_pixel(1, 0, 128, 255); 
  strip.set_pixel(2, 0, 128, 255); 
  strip.set_pixel(3, 0, 0, 0); 
  strip.set_pixel(4, 0, 255, 0);
  strip.set_pixel(5, 255, 0, 0);
  strip.set_pixel(6, 0, 255, 0);
  strip.show();
    delay(500); 
  strip.set_pixel(0, 0, 255, 0); 
  strip.set_pixel(1, 0, 0, 0); 
  strip.set_pixel(2, 0, 0, 0); 
  strip.set_pixel(3, 0, 0, 0); 
  strip.show();
    delay(250); 
    strip.set_pixel(0, 0, 255, 0); 
  strip.set_pixel(1, 0, 255, 0); 
  strip.set_pixel(2, 0, 0, 0); 
  strip.set_pixel(3, 0, 0, 0); 
  strip.show();
    delay(250); 
      strip.set_pixel(0, 0, 255, 0); 
  strip.set_pixel(1, 0, 255, 0); 
  strip.set_pixel(2, 0, 255, 0); 
  strip.set_pixel(3, 0, 0, 0); 
  strip.show();
    delay(250); 
      strip.set_pixel(0, 0, 255, 0); 
  strip.set_pixel(1, 0, 255, 0); 
  strip.set_pixel(2, 0, 255, 0); 
  strip.set_pixel(3, 0, 255, 0); 
  strip.show();
    delay(250);   
          strip.setAllLedsColorData(255,0,0); 
              delay(300); 
                  
        strip.setAllLedsColorData(0,0,0); 
  strip.show();  // Execute the light off command.
}

void loop() {
    Ultra_Avoid();
strip.setBrightness(25);
}

void Ultra_Avoid(){
  Servo_2_Angle(servo_Init);
  delay(80);
  int a = Get_Sonar();
  int b = Get_Sonar();
  int c = Get_Sonar();
  midDist = (a+b+c)/3;
  Serial.print("Mid:");
  Serial.println(midDist);
   Servo_1_Angle(servo_Init);  // front wheel
  Motor(1,1,0); //Stop the car
  Motor(2,1,0);
    strip.setAllLedsColorData(255,0,0); // 
  strip.show();  

  if (midDist > avoid_Dist){
    
      Serial.println("forward");
      Serial.print("servo1:");
     Serial.println(servo_Init);
      Servo_1_Angle(servo_Init+ deviation);  // front wheel
      Motor(1,1,Speed); //forward
      Motor(2,1,Speed);
  strip.setAllLedsColorData(255,0,0); 
  strip.show(); 
  }
  else if (midDist <= avoid_Dist){
  strip.setAllLedsColorData(255,0,0);
  strip.show();  
      Servo_1_Angle(servo_Init + deviation);  // front wheel
      Motor(1,1,0); //Stop the car
      Motor(2,1,0);

      Servo_2_Angle(servo_Init - 60); // left distance.
      delay(400);
      int a = Get_Sonar();
      int b = Get_Sonar();
      int c = Get_Sonar();
      leftDist = (a+b+c)/3;
      Serial.print("Left:");
      Serial.println(leftDist);
       strip.setAllLedsColorData(255,255,0); 
       strip.show();  
      Servo_2_Angle(servo_Init + 60); // right distance.
      delay(400);
      a = Get_Sonar();
      b = Get_Sonar();
      c = Get_Sonar();
      rightDist = (a+b+c)/3;
      Serial.print("Right:");
      Serial.println(rightDist);
       strip.setAllLedsColorData(0,255,255); 
       strip.show();  
      Servo_2_Angle(servo_Init); // back to mid.
  
    if ((leftDist < avoid_Dist)&&(rightDist < avoid_Dist)){ // Judgment left and right.
        if (leftDist >= rightDist){
          // There are obstacles on the right backward to the left. 
          Servo_1_Angle(servo_Init + wheel_Steering + deviation); //turn left backward
          Motor(1,-1,Speed); //backward
          Motor(2,-1,Speed);
          strip.setAllLedsColorData(0,0,255); 
          strip.show();  
          delay(500);
        }
        else{ //There are obstacles on the left.
          
        Serial.println("turn right backward");
          Servo_1_Angle(servo_Init - wheel_Steering + deviation); //turn right backward
          Motor(1,-1,Speed); //backward
          Motor(2,-1,Speed);
          strip.setAllLedsColorData(0,0,255); 
          strip.show();  
          delay(500);
        }
    }
    else if ((leftDist > avoid_Dist)&&(rightDist <= avoid_Dist)){
        if (midDist < minDist){ // Obstacle ahead
        Serial.println("backward");
          Servo_1_Angle(servo_Init+ deviation); // backward
          Motor(1,-1,Speed); 
          Motor(2,-1,Speed);
          strip.setAllLedsColorData(0,0,255); 
          strip.show();  
          delay(400);
        }
        Serial.println("turn left backward");
        Servo_1_Angle(servo_Init + wheel_Steering + deviation); // turn left backward
        Motor(1,-1,Speed); 
        Motor(2,-1,Speed);
          strip.setAllLedsColorData(0,0,255); 
          strip.show();  
        delay(500);
    }
    else if ((leftDist <= avoid_Dist) &&(rightDist > avoid_Dist)){ // There are obstacles on the left.
        if (midDist < minDist){ // Obstacle ahead
        Serial.println(" backward");
            Servo_1_Angle(servo_Init + deviation); // backward
            Motor(1,-1,Speed); 
            Motor(2,-1,Speed);
           strip.setAllLedsColorData(0,0,255); 
          strip.show();  
            delay(500);
        }
        Serial.println(" turn right backward");
        Servo_1_Angle(servo_Init - wheel_Steering + deviation); //turn right backward
        Motor(1,-1,Speed); //backward
        Motor(2,-1,Speed);
          strip.setAllLedsColorData(0,0,255); 
          strip.show();  
        delay(400);
    }
    else if ((leftDist >= avoid_Dist) &&( rightDist >= avoid_Dist)){
        if (leftDist > rightDist){ // The distance to the right is greater than the left
            if (midDist < minDist){
                Servo_1_Angle(servo_Init+ deviation); // backward
                Motor(1,-1,Speed); 
                Motor(2,-1,Speed);
          strip.setAllLedsColorData(0,0,255); 
          strip.show();  
                delay(500);
            }
            Servo_1_Angle(servo_Init + wheel_Steering + deviation); // turn left backward
            Motor(1,-1,Speed); 
            Motor(2,-1,Speed);
          strip.setAllLedsColorData(0,0,255); 
          strip.show();  
            delay(400);
        }
        else{
            if (midDist < minDist){
                Servo_1_Angle(servo_Init+ deviation); // backward
                Motor(1,-1,Speed); 
                Motor(2,-1,Speed);
          strip.setAllLedsColorData(0,0,255); 
          strip.show();  
                delay(500);
            }
            Servo_1_Angle(servo_Init + wheel_Steering + deviation); // turn left backward
            Motor(1,-1,Speed); 
            Motor(2,-1,Speed);
          strip.setAllLedsColorData(0,0,255); 
          strip.show(); 
            delay(400);
       }
    }
  }
}

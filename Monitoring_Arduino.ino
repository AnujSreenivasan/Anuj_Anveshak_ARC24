#include <ros.h>

//importing message types
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

//other libraries

#include <Wire.h>

//MLX90614
#include <Adafruit_MLX90614.h>

//MLX1700



//defining constants for temperature
#define IR1 0x5C
#define IR2 0x5A
#define IR3 0x5B                                  

Adafruit_MLX90614 mlx1;
Adafruit_MLX90614 mlx2;
Adafruit_MLX90614 mlx3;

//initializing voltage publisher
ros::NodeHandle nh;
std_msgs::Float32MultiArray voltdisp_msg;
std_msgs::MultiArrayDimension voltdisp_dim;
std_msgs::MultiArrayLayout voltdisp_layout;
ros::Publisher voltpub("voltpubl", &voltdisp_msg);

//initializing temperature publisher
std_msgs::Float32MultiArray tempdisp_msg;
std_msgs::MultiArrayDimension tempdisp_dim;
std_msgs::MultiArrayLayout tempdisp_layout;
ros::Publisher temppub("temppubl", &tempdisp_msg);

//initializing current publisher
std_msgs::Float32 currentdisp_msg;
ros::Publisher currentpub("currentpubl", &currentdisp_msg);

//MOSFET variable
bool mosfet;

//MOSFET callback
void mosfet_callback(std_msgs::Bool& msg){
  
  mosfet = msg.data;
  Serial.println(mosfet);
  
  }

//MOSFET activation condition
void OnOff(){
  if (mosfet == 1){
    
    analogWrite(3, 153);
    
    }
    
  else {
    
    analogWrite(3, 0);
    
    }
  }

//initializing subscriber for MOSFET
ros::Subscriber<std_msgs::Bool> mosfet_sub("mosfet", &mosfet_callback);

//cell values
float C1 = 0.0; float C2 = 0.0; float C3 = 0.0; float C4 = 0.0;
float C5 = 0.0; float C6 = 0.0; float C7 = 0.0; float C8 = 0.0;
float C9 = 0.0; float C10 = 0.0; float C11 = 0.0; float C12 = 0.0; float C13 = 0.0;


float battery[13] = {0.0};

//temperature values
float T1 = 0.0; float T2 = 0.0; float T3 = 0.0;

float temp[3] = {0.0};

//current values

float current_val = 0.0;

float current(){
  float current_val1 = 0.0;
  for(int i = 0; i < 100; i++){
    current_val1 +=  ((analogRead(A13)) * (5000.0 / 1023.0) - (0.5 * 5000.0) + 164.0) / 33.0;
    }
  current_val1 /= 100;
  return current_val1;
}

void setup(){
  Serial.begin(9600);
  
  pinMode(A0,INPUT); pinMode(A1,INPUT); pinMode(A2,INPUT);
  pinMode(A3,INPUT); pinMode(A4,INPUT); pinMode(A5,INPUT);
  pinMode(A6,INPUT); pinMode(A7,INPUT); pinMode(A8,INPUT);
  pinMode(A9,INPUT); pinMode(A10,INPUT); pinMode(A11,INPUT);
  pinMode(A12,INPUT); pinMode(A13,INPUT);
  pinMode(3,OUTPUT);
  
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  
  nh.advertise(voltpub);
  voltdisp_msg.data_length = 13;
  voltdisp_msg.layout = voltdisp_layout;

  nh.advertise(temppub);
  tempdisp_msg.data_length = 3;
  tempdisp_msg.layout = tempdisp_layout;

  nh.advertise(currentpub);

  nh.subscribe(mosfet_sub);

  mlx1.begin(IR1);
  mlx2.begin(IR2);
  mlx3.begin(IR3);
  }
void loop(){

  OnOff();
  
  C1 = analogRead(A5)*(5.0/1023.0)*11.0; C2 = analogRead(A4)*(5.0/1023.0)*11.0; C3 = analogRead(A3)*(5.0/1023.0)*11.0;
  C4 = analogRead(A2)*(5.0/1023.0)*11.0; C5 = analogRead(A1)*(5.0/1023.0)*11.0; C6 = analogRead(A0)*(5.0/1023.0)*11.0;

  C7 = analogRead(A7)*(5.0/1023.0)*11.0; C8 = analogRead(A6)*(5.0/1023.0)*11.0; C9 = analogRead(A11)*(5.0/1023.0)*11.0;
  C10 = analogRead(A10)*(5.0/1023.0)*11.0;

  C11 = analogRead(A9)*(5.0/1023.0)*11.0; C12 = analogRead(A8)*(5.0/1023.0)*11.0; C13 = analogRead(A12)*(5.0/1023.0)*11.0;



  
  battery[0] = C1; battery[1] = C2 - C1; battery[2] = C3 - C2; battery[3] = C4 - C3; battery[4] = C5 - C4; battery[5] = C6 - C5;

  battery[6] = C7; battery[7] = C8 - C7; battery[8] = C9 - C8; battery[9] = C10 - C9;
  
  battery[10] = C11; battery[11] = C12 - C11; battery[12] = C13 - C12;



  temp[0] = mlx1.readObjectTempC(); temp[1] = mlx2.readObjectTempC(); temp[2] = mlx3.readObjectTempC();



  current_val = current();
  
  
  voltdisp_msg.data = battery;
  tempdisp_msg.data = temp;
  currentdisp_msg.data = current_val;
  nh.spinOnce();
  voltpub.publish(&voltdisp_msg);
  temppub.publish(&tempdisp_msg);
  currentpub.publish(&currentdisp_msg);
  delay(1000);
  
  }

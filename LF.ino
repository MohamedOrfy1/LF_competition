#include "LF.h"
int sensor_1 , sensor_2 , sensor_3 , sensor_4 , sensor_5 ;
int flag = 0 ;
//extern double startangle,newangle;
void setup ( ) {
  INIT();
  Serial.begin(9600);
}
void loop ( ) {
  

  Read(&sensor_1,&sensor_2,&sensor_3,&sensor_4,&sensor_5);
  if(sensor_1==0 && sensor_2==0 ){
    left(); 
  } else if(sensor_5==0 && sensor_4==0 && sensor_3==0 ){
    right();
     Stop();
    delay(10);
    Serial.println("R");
 }
  else if((sensor_3==0) ){
    Forward_PID2(); 
    Serial.println("f");
    Serial.println(sensor_1);
    Serial.println(sensor_2);
    Serial.println(sensor_3);
    Serial.println(sensor_4);
    Serial.println(sensor_5);
  }else if ((sensor_1==1) && (sensor_2==0)){
    Forward_PID2(); 
  }
  else if ((sensor_5==1) && (sensor_4==0)){
    Forward_PID2(); 
  }

  else if(sensor_5==0 && sensor_4==0 ){
   
    right();
    Stop();
    delay(10);
    Serial.println("R");
 }
 else if ((sensor_5==1) && (sensor_4==1 && sensor_3==1 && sensor_2==1 && sensor_1==1)){
    Uturn(); 
  }/*else if (sensor_5==0 && sensor_4==0 && sensor_3==0 && sensor_1==0 && sensor_2==0){
      forward();
      delay(100);
      if (sensor_5==0 && sensor_4==0 && sensor_3==0 && sensor_1==0 && sensor_2==0){
       Stop(); 
      }else {
      left();
  }*/
 }
/*  else if(sensor_1==1&&sensor_2==1&&sensor_3==1&&sensor_4==1&&sensor_5==1){
    Stop();
    delay(1000);
    //Uturn();
  }
  
}*/




//if(sensor_1==0&&sensor_2==0){
//  Turn_Left();
//  Serial.println("L"); 
//  }
//else if(sensor_3==0){
//  //double sp1=70,sp2=70;
//Forward_PID2();
//Serial.println("F");
// }
// else if(sensor_1==1&&sensor_2==1&&sensor_3==1&&sensor_4==0&&sensor_5==0){
//  right();
//  Serial.println("R"); 
//  }
//  else if(sensor_1==1&&sensor_2==1&&sensor_3==1&&sensor_4==1&&sensor_5==1){
//  right();
//  Serial.println("U-turn");
//  }

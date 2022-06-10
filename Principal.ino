#include <LEANTEC_ControlMotor.h>   //Incluimos la librería control de motores  
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
 #include <std_msgs/String.h>
#include "TimerOne.h"

//////////////////////////////////////////////////////////// Configuración de pines

ControlMotor control(3,4,5,6,2,7);    
int MotorDer1 = 3; //El pin 3 de arduino se conecta con el pin In1 del L298N 
int MotorDer2 = 4; //El pin 4 de arduino se conecta con el pin In2 del L298N 
int MotorIzq1 = 5; //El pin 5 de arduino se conecta con el pin In3 del L298N 
int MotorIzq2 = 6; //El pin 6 de arduino se conecta con el pin In4 del L298N 
int PWM_Derecho = 2; //El pin 2 de arduino se conecta con el pin EnA del L298N 
int PWM_Izquierdo = 7; //El pin 7 de arduino se conecta con el pin EnB del L298N

const byte Encoder1 = 8;  // encoder1 Interrupt Pin - INT 0
const byte Encoder2 = 9;  // encoder 2 Interrupt Pin - INT 1 

//////////////////////////////////////////////////////////// Variables de estado

int velocidad_der = 0; 
int velocidad_izq = 0;  
int velocidad_time = 0;  
unsigned long startMillis = 0;

//////////////////////////////////////////////////////////// Comunicación Ros: Publisher y subscriber

ros::NodeHandle nh;
std_msgs::String stop_info;
std_msgs::String arduino_info;
geometry_msgs::Twist data_motores;
int pmw_der;
int pmw_izq;
int time_motors;

ros::Publisher stop_arduino("StopArduino_info", &stop_info);
ros::Publisher publish_arduino("publish_arduino", &arduino_info);
ros::Publisher data_arduino("data_arduino", &data_motores);

//////////////////////////////////////////////////////////// Configuración de constantes

//////////////////////////////////////////////////////////// Encoders

//////////////////////////////////////////////////////////// Control servos (Callbback subsriber)

//////////////////////////////////////////////////////////// Control motores (Callbback subsriber)

void callback_Motores(const geometry_msgs::Twist& motor_data){
  
  velocidad_der = motor_data.linear.y; 
  velocidad_izq = motor_data.linear.x;  
  velocidad_time = motor_data.linear.z; 

  data_motores.linear.x = velocidad_der;
  data_motores.linear.y = velocidad_izq;
  data_motores.linear.z = velocidad_time;
  
  startMillis = millis();

  data_motores.angular.x = startMillis;
}

void avanzar(int velocidad_der, int velocidad_izq){
  digitalWrite(MotorDer1,HIGH);   
  digitalWrite(MotorDer2,LOW);   
  digitalWrite(MotorIzq1,HIGH);   
  digitalWrite(MotorIzq2,LOW);
  analogWrite(PWM_Derecho,velocidad_der);//Velocidad motor 
  analogWrite(PWM_Izquierdo,velocidad_izq);//Velocidad motor 
}
void retroceder(int velocidad_der, int velocidad_izq){
  digitalWrite(MotorDer1,LOW);   
  digitalWrite(MotorDer2,HIGH);   
  digitalWrite(MotorIzq1,LOW);   
  digitalWrite(MotorIzq2,HIGH);
  analogWrite(PWM_Derecho,velocidad_der);//Velocidad motor 
  analogWrite(PWM_Izquierdo,velocidad_izq);//Velocidad motor 
}
void giro_horario(int velocidad_der, int velocidad_izq){
  digitalWrite(MotorDer1,HIGH);   
  digitalWrite(MotorDer2,LOW);   
  digitalWrite(MotorIzq1,LOW);   
  digitalWrite(MotorIzq2,HIGH);
  analogWrite(PWM_Derecho,velocidad_der);//Velocidad motor 
  analogWrite(PWM_Izquierdo,velocidad_izq);//Velocidad motor 
}

void giro_antihorario(int velocidad_der, int velocidad_izq){    
  digitalWrite(MotorDer1,LOW);   
  digitalWrite(MotorDer2,HIGH);   
  digitalWrite(MotorIzq1,HIGH);   
  digitalWrite(MotorIzq2,LOW);
  analogWrite(PWM_Derecho,velocidad_der);//Velocidad motor 
  analogWrite(PWM_Izquierdo,velocidad_izq);//Velocidad motor 
} 
void parar(int velocidad_der, int velocidad_izq){  
  digitalWrite(MotorDer1,LOW);   
  digitalWrite(MotorDer2,LOW);   
  digitalWrite(MotorIzq1,LOW);   
  digitalWrite(MotorIzq2,LOW);
  analogWrite(PWM_Derecho,velocidad_der);//Velocidad motor 
  analogWrite(PWM_Izquierdo,velocidad_izq);//Velocidad motor 
}

ros::Subscriber<geometry_msgs::Twist> receive_vel("Motors_Instructions", &callback_Motores);

void setup()  {    //Configuramos los pines como salida
   Serial.begin(57600);
   pinMode(MotorDer1, OUTPUT);    pinMode(MotorDer2, OUTPUT);
   pinMode(MotorIzq1, OUTPUT);    pinMode(MotorIzq2, OUTPUT);
   pinMode(PWM_Derecho, OUTPUT);   pinMode(PWM_Izquierdo, OUTPUT); 

   //Se inicializa el nodo que va a ser de publisher para enviar info de encoders
   nh.initNode();
   nh.advertise(stop_arduino);
   nh.advertise(publish_arduino);
   nh.advertise(data_arduino);

   //Se inicializa nodo que va a ser de subscriber para inicializar las variables de PMW
   nh.initNode();
   nh.subscribe(receive_vel);
}  

void loop()  {

  nh.spinOnce();
  //delay(500);
  
  unsigned long currentMillis = millis();

  data_motores.angular.y = velocidad_der;
  data_motores.angular.z = velocidad_izq;
  data_arduino.publish(&data_motores);

  if ((currentMillis - startMillis)/1000 <= velocidad_time) {
    
    if(velocidad_der > 0 & velocidad_izq > 0){
      avanzar(velocidad_der,velocidad_izq);
    }
    else if(velocidad_der > 0 & velocidad_izq < 0){
      giro_antihorario(velocidad_der,velocidad_izq);
    }
    else if(velocidad_der < 0 & velocidad_izq > 0){
      giro_horario(velocidad_der,velocidad_izq);
    }
    else{
      parar(0,0);
    }
  }
  else{
    parar(0,0);
  } 
}

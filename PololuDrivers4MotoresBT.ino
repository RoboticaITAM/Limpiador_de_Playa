#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define M1L_PWM 13
#define M1L_Dir 12
#define M2R_PWM 14
#define M2R_Dir 27
#define M3L_PWM 26
#define M3L_Dir 25
#define M4R_PWM 33
#define M4R_Dir 32

BluetoothSerial SerialBT;

//-----------------variables-----------------

// Configurando los parámetros de PWM
const int channel_M1L = 0;
const int channel_M2R = 1;
const int channel_M3L = 2;
const int channel_M4R = 3;
int freq = 1000;
int resolution = 12;

char dato;
bool activacion = false;

int vel_pwm = 1500;

void setup() {
  // Realizando la configuración de los pines a ocupar
  pinMode(M1L_Dir, OUTPUT);
  pinMode(M2R_Dir, OUTPUT);
  pinMode(M3L_Dir, OUTPUT);
  pinMode(M4R_Dir, OUTPUT);
  pinMode(M1L_PWM, OUTPUT);
  pinMode(M2R_PWM, OUTPUT);
  pinMode(M3L_PWM, OUTPUT);
  pinMode(M4R_PWM, OUTPUT);

  ledcSetup(channel_M1L, freq, resolution);
  ledcSetup(channel_M2R, freq, resolution);
  ledcSetup(channel_M3L, freq, resolution);
  ledcSetup(channel_M4R, freq, resolution);

  ledcAttachPin(M1L_PWM, channel_M1L);
  ledcAttachPin(M2R_PWM, channel_M2R);
  ledcAttachPin(M3L_PWM, channel_M3L);
  ledcAttachPin(M4R_PWM, channel_M4R);

  Serial.begin(115200);
  SerialBT.begin("Robot_Playa"); //Bluetooth device name

}

//-----------------loop----------------
void loop() {
  if (SerialBT.available()) {
    dato = SerialBT.read();
  
    if (dato == '0'){
      Stop();
    }
  
    if (dato == '1'){
      Forward();
    }

    if (dato == '2'){
      Right();
    }

    if (dato == '3'){
      Backward();
    }

    if (dato == '4'){
      Left();
    }

    

}
 
 
}

//------------------Funciones----------------

void Forward(){
  ledcWrite(channel_M1L, vel_pwm);
  ledcWrite(channel_M2R, vel_pwm);
  ledcWrite(channel_M3L, vel_pwm);
  ledcWrite(channel_M4R, vel_pwm);
  
  digitalWrite(M1L_Dir,HIGH);
  digitalWrite(M2R_Dir,LOW);
  digitalWrite(M3L_Dir,HIGH);
  digitalWrite(M4R_Dir,LOW);
  Serial.println("Forward");
}

void Backward(){
  ledcWrite(channel_M1L, vel_pwm);
  ledcWrite(channel_M2R, vel_pwm);
  ledcWrite(channel_M3L, vel_pwm);
  ledcWrite(channel_M4R, vel_pwm);
  
  digitalWrite(M1L_Dir,LOW);
  digitalWrite(M2R_Dir,HIGH);
  digitalWrite(M3L_Dir,LOW);
  digitalWrite(M4R_Dir,HIGH);
  Serial.println("Backward");
}

void Stop(){
  ledcWrite(channel_M1L, 0);
  ledcWrite(channel_M2R, 0);
  ledcWrite(channel_M3L, 0);
  ledcWrite(channel_M4R, 0);
  Serial.println("Stop");
}

void Right(){
  ledcWrite(channel_M1L, vel_pwm);
  ledcWrite(channel_M2R, vel_pwm);
  ledcWrite(channel_M3L, vel_pwm);
  ledcWrite(channel_M4R, vel_pwm);
  
  digitalWrite(M1L_Dir,HIGH);
  digitalWrite(M2R_Dir,HIGH);
  digitalWrite(M3L_Dir,HIGH);
  digitalWrite(M4R_Dir,HIGH);
  Serial.println("Right");
}

void Left(){
  ledcWrite(channel_M1L, vel_pwm);
  ledcWrite(channel_M2R, vel_pwm);
  ledcWrite(channel_M3L, vel_pwm);
  ledcWrite(channel_M4R, vel_pwm);
  
  digitalWrite(M1L_Dir,LOW);
  digitalWrite(M2R_Dir,LOW);
  digitalWrite(M3L_Dir,LOW);
  digitalWrite(M4R_Dir,LOW);
  Serial.println("Left");
}

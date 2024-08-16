#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define M1R_Dir 13
#define M1R_PWM 12
#define M2L_Dir 14
#define M2L_PWM 27

BluetoothSerial SerialBT;

//-----------------variables-----------------

// Configurando los parámetros de PWM
const int freq = 500;
const int channel_M1R = 0;
const int channel_M2L = 1;
const int resolution = 12;

char dato;
int factor = 2;
int vel_pwm = 500;

void setup() {
  // Realizando la configuración de los pines a ocupar
  pinMode(M1R_PWM, OUTPUT);
  pinMode(M1R_Dir, OUTPUT);

  pinMode(M2L_PWM, OUTPUT);
  pinMode(M2L_Dir, OUTPUT);
  
  ledcSetup(channel_M1R, freq, resolution);
  ledcSetup(channel_M2L, freq, resolution);
  
  ledcAttachPin(M1R_PWM, channel_M1R);
  ledcAttachPin(M2L_PWM, channel_M2L);

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

    if (dato == '5'){
      factor++;
      if (factor == 9){
        factor = 8;
      }
      
    }

    if (dato == '6'){
      factor--;
      if (factor == 1){
        factor = 2;
      }
    }
  }
  Serial.println(factor);
}
 
 

//------------------Funciones----------------

void Forward(){
  ledcWrite(channel_M1R, factor*vel_pwm);
  ledcWrite(channel_M2L, factor*vel_pwm);
  digitalWrite(M1R_Dir,LOW);
  digitalWrite(M2L_Dir,LOW);
  
  Serial.println("Forward");
}

void Backward(){
  ledcWrite(channel_M1R, factor*vel_pwm);
  ledcWrite(channel_M2L, factor*vel_pwm);
  digitalWrite(M1R_Dir,HIGH);
  digitalWrite(M2L_Dir,HIGH);
  
  Serial.println("Backward");
}

void Stop(){
  ledcWrite(channel_M1R, 0*vel_pwm);
  ledcWrite(channel_M2L, 0*vel_pwm);

  Serial.println("Stop");
}

void Right(){
  ledcWrite(channel_M1R, factor*vel_pwm);
  ledcWrite(channel_M2L, factor*vel_pwm);
  digitalWrite(M1R_Dir,HIGH);
  digitalWrite(M2L_Dir,LOW);

  Serial.println("Right");
}

void Left(){
  ledcWrite(channel_M1R, factor*vel_pwm);
  ledcWrite(channel_M2L, factor*vel_pwm);
  digitalWrite(M1R_Dir,LOW);
  digitalWrite(M2L_Dir,HIGH);

  Serial.println("Left");
}

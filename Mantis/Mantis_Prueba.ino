#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define M1R_Dir 19
#define M1R_PWM 28
#define M2L_Dir 5
#define M2L_PWM 17
#define Barredora_Dir 25
#define Barredora_PWM 26
#define Mecanismo_Dir 27
#define Mecanismo_PWM 14

BluetoothSerial SerialBT;

//-----------------variables-----------------

// Configurando los parámetros de PWM
const int freq = 500;
const int channel_M1R = 2;
const int channel_M2L = 3;
const int channel_Barredora = 4;
const int channel_Mecanismo = 5;
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

  pinMode(Barredora_PWM, OUTPUT);
  pinMode(Barredora_Dir, OUTPUT);

  pinMode(Mecanismo_PWM, OUTPUT);
  pinMode(Mecanismo_Dir, OUTPUT);
  
  ledcSetup(channel_M1R, freq, resolution);
  ledcSetup(channel_M2L, freq, resolution);
  ledcSetup(channel_Barredora, freq, resolution);
  ledcSetup(channel_Mecanismo, freq, resolution);
  
  ledcAttachPin(M1R_PWM, channel_M1R);
  ledcAttachPin(M2L_PWM, channel_M2L);
  ledcAttachPin(Barredora_PWM, channel_Barredora);
  ledcAttachPin(Mecanismo_PWM, channel_Mecanismo);

  Serial.begin(115200);
  SerialBT.begin("Mantis"); //Bluetooth device name
}


//-----------------loop----------------
void loop() {
  if (SerialBT.available()) {
    dato = SerialBT.read();
  
    if (dato == '0'){
      Stop();
      Alto_Barredora();
      Alto_Mecanismo();
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
      Baja_Mecanismo();
      Mete_Lata();
    }
    
    if (dato == '6'){
      Sube_Mecanismo();
    }

    if (dato == '7'){
      Saca_Lata();
    }
    
//    if (dato == '8'){
//      Mete_Lata();
//    }
    

    if (dato == '8'){
      factor++;
      if (factor == 9){
        factor = 8;
      }
      
    }

    if (dato == '9'){
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

void Baja_Mecanismo(){
  ledcWrite(channel_Mecanismo, factor*vel_pwm);
  digitalWrite(Mecanismo_Dir,LOW);

  Serial.println("Baja_Mecanismo");
}

void Sube_Mecanismo(){
  ledcWrite(channel_Mecanismo, factor*vel_pwm);
  digitalWrite(Mecanismo_Dir,HIGH);

  Serial.println("Sube_Mecanismo");
}

void Alto_Mecanismo(){
  ledcWrite(channel_Mecanismo, 0*vel_pwm);
  
  Serial.println("Alto_Mecanismo");
}

void Saca_Lata(){
  ledcWrite(channel_Barredora, factor*vel_pwm);
  digitalWrite(Barredora_Dir,LOW);

  Serial.println("Saca_Lata");
}

void Mete_Lata(){
  ledcWrite(channel_Barredora, factor*vel_pwm);
  digitalWrite(Barredora_Dir,HIGH);

  Serial.println("Mete_Lata");
}

void Alto_Barredora(){
  ledcWrite(Barredora_Dir, 0*vel_pwm);
  
  Serial.println("Alto_Barredora");
}

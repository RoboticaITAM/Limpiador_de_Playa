#include <Servo.h>

//-----------------Pines-----------------
#include <Wire.h>
#include "BNO055_support.h" 


Servo rampa;
int pos = 0; 


#define motor_izq_pwm 18
#define motor_der_pwm 17
#define motor_der_dir 5 
#define motor_izq_dir 19

#define encoder_izq_A 16    // Amarillo
#define encoder_izq_B 4    // Blanco
#define encoder_der_A 2    // Amarillo
#define encoder_der_B 15    // Blanco

#define end_stop1 36
#define end_stop2 39 

//SENSORES ULTRASÓNICOS
#define echoPin_1 34 
#define trigPin_1 35 
#define echoPin_2  32
#define trigPin_2 33 

#define dir_mecanismo 26 
#define pwm_mecanismo 25 

#define dir_barredora 14 
#define pwm_barredora 27

//NUEVO 
#define BNO055_SAMPLERATE_DELAY_MS (300)



//-----------------Variables-----------------

//ULTRASONICOS DISTANCIAS
const int distanciaMaxima = 400;
const int distanciaSeguridad = 15; 

// Configurando los parámetros de PWM
int pwm_L = 0;
int dir_L = 0;
int pwm_R = 0;
int dir_R = 0;

int channel_Left = 0;
int channel_Right = 1;
int channel_mec = 2; 
int channel_barredora = 3; 

int freq = 1000;
int resolution = 12;
float left_wheel = 0;
float right_wheel = 0;

//Variables para calcular la velocidad (diferencia de pulsos/tiempo)
volatile long L_pulses = 0; 
volatile long R_pulses = 0; 
int previos_L_pulses = 0;
int previos_R_pulses = 0;
float velocity_L = 0;
float velocity_R = 0;

//Medicion del tiempo (usadas en varias partes del código)
long previous_Time = 0;
float delta_Time;
long current_Time;

//Variables para conversión de la velocidad a rpm y rps
float PPR = 480;
float rps_L = 0;
float rpm_L = 0;
float rps_R = 0;
float rpm_R = 0;

//Medidas llanta
float W_Radius = 0.03;
float Width = 0.2;

//Velocidad angular y lineal 
float Omega_L = 0;
float Omega_R = 0;
float VL = 0;
float VR = 0;

//Velocidad angular y lineal filtrada 
float VL_Filt = 0;
float VL_Prev = 0;
float VR_Filt = 0;
float VR_Prev = 0;

//Constantes de integración y proporcional 
float kp_L = 30000;//300;
float ki_L = 100;//6.5;
float kp_R = 38000;//300;
float ki_R = 500;//6.5;

//Variables para la medición de errores 
float e_L = 0;
float u_L = 0;
float e_L_integral = 0; 
float e_R = 0;
float u_R = 0;
float e_R_integral = 0;

//Variables IMU 
long tiempo_prev;
float dt;
float ang_z;
float ang_z_prev;

//Nuevas variables IMU 
struct bno055_t myBNO;
struct bno055_euler myEulerData; 
unsigned long lastTime = 0;

volatile bool objetoDetectado = false; // Variable que indica si se detectó un objeto

int estado= 0;  
int estado_recoge = 0; 
int estado_deposita = 0; 
int contador_latas = 0; 

int ENTRADA = 0; 
int bit_1;
int bit_2;
int bit_3;
int bit_4;
int bit_5;

//-----------------INTERRUPCIONES-------------------
void IRAM_ATTR encoderL(){
  if (digitalRead(encoder_izq_B) == HIGH){     // si B es HIGH, sentido horario
    L_pulses-- ;        // decrementa L_PULSES en 1
  }
  else {          // si B es LOW, sentido anti horario
    L_pulses++ ;        // incrementa L_PULSES en 1
  }
}
void IRAM_ATTR encoderR(){
  if (digitalRead(encoder_der_B) == HIGH){     // si B es HIGH, sentido horario
    R_pulses-- ;        // incrementa R_PULSES en 1
  }
  else {          // si B es LOW, sentido anti horario
    R_pulses++ ;        // decrementa R_PULSES en 1
  }
}
//FUNCIONES-----------------------------------------------------------------
//---------------------Calculos----------------
void Calcula_velocidad(){
  current_Time = micros();                                        //Se lee el tiempo en este instante
  delta_Time = ((float)(current_Time - previous_Time))/1.0e6;     //Se calcula la diferencia de tiempo
  velocity_L = (L_pulses - previos_L_pulses)/delta_Time;          //Se calcula la velocidad de la rueda Izquierda (diferencia de pulsos/tiempo)
  velocity_R = (R_pulses - previos_R_pulses)/delta_Time;          //Se calcula la velocidad de la rueda Derecha (diferencia de pulsos/tiempo)
  previous_Time = current_Time;                                   //Se actualiza el valor del tiempo para la siguiente iteración
  previos_L_pulses = L_pulses;                                    //Se actualiza el valor de los pulsos izquierdos para la siguiente iteración
  previos_R_pulses = R_pulses;                                    //Se actualiza el valor de los pulsos derechos para la siguiente iteración

  //Convertimos pulsos por segundo a RPS, RPM y Rad/s, y m/s
  rps_L = velocity_L / PPR;     //Velocidad angular rueda izquierda en RPS
  rps_R = velocity_R / PPR;     //Velocidad angular rueda derecha en RPS
  rpm_L = rps_L * 60;           //Velocidad angular rueda izquierda en RPM
  rpm_R = rps_R * 60;           //Velocidad angular rueda derecha en RPM
  Omega_L = rps_L * 2 * PI;     //Velocidad angular rueda izquierda en rad/s
  Omega_R = rps_R * 2 * PI;     //Velocidad angular rueda derecha en rad/s
  VL = W_Radius * Omega_L;         //Velocidad lineal rueda izquierda en m/s
  VR = W_Radius * Omega_R;         //Velocidad lineal rueda derecha en m/s
  
  //Filtro pasa-bajas (25Hz)
  VL_Filt = 0.854*VL_Filt + 0.0728*VL + 0.0728*VL_Prev;
  VL_Prev = VL;
  VR_Filt = 0.854*VR_Filt + 0.0728*VR + 0.0728*VR_Prev;
  VR_Prev = VR;
}

//------------FUNCION PRINCIPAL MOVIMIENTO----------
void Control(float VL_Target,float VW_Target){
  bno055_read_euler_hrp(&myEulerData);   
  
  Calcula_velocidad();//CAMBIÉ ESTO 
  //Obtenemos valores de la IMU en el eje z
//  float gyroZ = imu.g.z;
//  float deg_Z = map(gyroZ,-32768,32767,-245,245);

  float deg_Z = (myEulerData.h / 16.00)-180;
  //Calculamos la velocidad angular (w) en rad/s
  float rad_Z= deg_Z*PI/180;
  
  //Variables de constante proporcional en IMU 
  float kp_w=10000; 
  float e_w, u_w;
  
  //Control PI (sin IMU) 
  e_L = VL_Target - VL_Filt;
  e_L_integral = e_L_integral + e_L * delta_Time;
  e_R = VL_Target - VR_Filt;
  e_R_integral = e_R_integral + e_R * delta_Time;
  e_w = VW_Target - rad_Z;
  u_w= kp_w * e_w;   

  //Control PI (con IMU) 
  u_L = kp_L*e_L + ki_L*e_L_integral - u_w;
  u_R = kp_R*e_R + ki_R*e_R_integral + u_w;

  //---MOTOR IZQUIERDO (PWM Y DIRECCIÓN)---
  //Dirección 
  if (u_L > 0){
    dir_L = 1; // hacia adelante
  }else if (u_L < 0){
    dir_L = -1;   //hacia atrás
  }else {
    dir_L = 0;   //Detenido
  }
  
  //PWM
  pwm_L = (int) fabs(u_L);
  if(pwm_L > 4095){
    pwm_L = 4095;
  }
  setMotor_L(dir_L,pwm_L);
  
  //----MOTOR DERECHO(DIRECCIÓN Y PWM)----
  
  //Direccion 
  if (u_R > 0){
    dir_R = 1; // hacia adelante
  }else if (u_R < 0){
    dir_R = -1;   //hacia atrás
  }else {
    dir_L = 0;   //Detenido
  }
  
  //PWM
  pwm_R = (int) fabs(u_R);
  if(pwm_R > 4095){
    pwm_R = 4095;
  }
  setMotor_R(dir_R,pwm_R);
}

//-----------------------MOVIMIENTO MOTORES--------------------------
void setMotor_L(int dir_L, int pwmVal_L){
  // Motor speed
  if(dir_L == 1){ 
    // Turn forward
    ledcWrite(channel_Left, pwmVal_L);
    digitalWrite(motor_izq_dir, HIGH);
    
  }
  else if(dir_L == -1){
    // Turn the other way
    ledcWrite(channel_Left, pwmVal_L);
    digitalWrite(motor_izq_dir, LOW);
  }
  else if(dir_L ==0){
    // Or dont turn
    ledcWrite(channel_Left, 0);  
  }
}

void setMotor_R(int dir_R, int pwmVal_R){
  // Motor speed
  if(dir_R == 1){ 
    ledcWrite(channel_Right, pwmVal_R);
    // Turn forward
    digitalWrite(motor_der_dir , HIGH);
  }
  else if(dir_R == -1){
    ledcWrite(channel_Right, pwmVal_R);
    // Turn the other way
    digitalWrite(motor_der_dir, LOW);
  }
  else if(dir_R ==0){
    ledcWrite(channel_Right, 0);  
  }
}

void setMotor_mec(int dir_M, int pwmVal_M){
  // Motor speed
  if(dir_M == 1){ 
    ledcWrite(channel_mec, pwmVal_M);
    // Turn forward
    digitalWrite(dir_mecanismo , HIGH);
  }
  else if(dir_M == -1){
    ledcWrite(channel_mec, pwmVal_M);
    // Turn the other way
    digitalWrite(dir_mecanismo, LOW);
  }
  else if(dir_M ==0){
    ledcWrite(channel_mec, 0);  
  }
}

void setMotor_barredora(int dir_B, int pwmVal_B){
  // Motor speed
  if(dir_B == 1){ 
    ledcWrite(channel_barredora, pwmVal_B);
    // Turn forward
    digitalWrite(dir_barredora , HIGH);
  }
  else if(dir_B == -1){
    ledcWrite(channel_barredora, pwmVal_B);
    // Turn the other way
    digitalWrite(dir_barredora, LOW);
  }
  else if(dir_B ==0){
    ledcWrite(channel_barredora, 0);  
  }
}

void arriba_mec(){
  setMotor_mec(-1,4095); 
}
void abajo_mec(){
  setMotor_mec(1,4095); 
}
void detiene_mec(){
  setMotor_mec(0,0); 
}

void detiene_barredora(){
  setMotor_barredora(0,0); 
}

void recoge(){
  setMotor_barredora(1,1025);
  Control(0.5,0); 
  
}

void deposita(){
  setMotor_barredora(-1,1025); 
}

void baja_rampa_recoge(){
  rampa.write(135);
}

void baja_rampa_deposita(){
  rampa.write(90); 
}

void sube_rampa(){
  rampa.write(0);
}
float medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2; // Convertir el tiempo a distancia (en cm)
}

void girarDerecha() {
  Control(0, 1); // Girar hacia la derecha
  delay(1000);
  Control(0, 0);
}
void detectarObstaculo() {
  // Verificar la distancia de los sensores
  if (medirDistancia(trigPin_1, echoPin_1) < distanciaSeguridad ||
      medirDistancia(trigPin_2, echoPin_2) < distanciaSeguridad) {
    objetoDetectado = true;
  }
}
//-----------------Setup-------------------------
void setup() {

  Serial.begin(115200);
  rampa.attach(13); 
  
  //Sensores ultrasónicos
  pinMode(trigPin_1, OUTPUT);
  pinMode(echoPin_1, INPUT);
  pinMode(trigPin_2, OUTPUT);
  pinMode(echoPin_2, INPUT);
  
  //Configuracion de pines
  pinMode(motor_izq_pwm, OUTPUT);
  pinMode(motor_der_pwm, OUTPUT);
  pinMode(motor_izq_dir, OUTPUT);
  pinMode(motor_der_dir, OUTPUT);
  
  pinMode(encoder_izq_A, INPUT);    
  pinMode(encoder_izq_B, INPUT);
  pinMode(encoder_der_A, INPUT);    
  pinMode(encoder_der_B, INPUT);

  pinMode(dir_mecanismo, OUTPUT);
  pinMode(pwm_mecanismo, OUTPUT);

  pinMode(end_stop1, INPUT_PULLUP);
  pinMode(end_stop2, INPUT_PULLUP); 
  
  ledcSetup(channel_mec, freq, resolution);
  ledcAttachPin(pwm_mecanismo, channel_mec);

  ledcSetup(channel_barredora, freq, resolution);
  ledcAttachPin(pwm_barredora, channel_barredora);
  //Configuración PWM
  ledcSetup(channel_Left, freq, resolution);
  ledcSetup(channel_Right, freq, resolution);
  ledcAttachPin(motor_izq_pwm, channel_Left);
  ledcAttachPin(motor_der_pwm, channel_Right);

  //Activa interrupciones 
  attachInterrupt(digitalPinToInterrupt(encoder_izq_A), encoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_der_A), encoderR, RISING);
  delay(1000);
  
  //Iniciar el BNO055
  BNO_Init(&myBNO);
 
  //Configuramos el modo en el que vamos a trabajar 
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
 
  delay(1);
  
  //Inicializo I2C
  Wire.begin();
  
  attachInterrupt(digitalPinToInterrupt(echoPin_1), detectarObstaculo, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPin_2), detectarObstaculo, CHANGE);

}
//-----------------Loop-----------------
void loop() {
  
  if(Serial.available()>0){
    String data = Serial.readStringUntil(']');
    String valores[5];
    int posIni = 1;
    int posComa;
    for(int i=0; i<5; i++){
      posComa = data.indexOf(',',posIni);
      if (posComa !=-1){
        valores[i] = data.substring(posIni, posComa);
        posIni = posComa+1;
      }else{
        valores[i] = data.substring(posIni);
        break;
      }
    }
  
    bit_1= valores[0].toInt();
    bit_2 = valores[1].toInt();
    bit_3 = valores[2].toInt();
    bit_4 = valores[3].toInt();
    bit_5 = valores[4].toInt();
    
  }
  //DAR LA VUELTA SI DETECTAMOS EL OBJETO 
  if (objetoDetectado) {
    girarDerecha();
    objetoDetectado = false; // Reiniciar la detección de objeto
  }
  
  switch(estado){
    case 0: 
      //Todo en estado inicial 
      detiene_mec();
      detiene_barredora();
      sube_rampa(); 
      contador_latas= 0; 
      setMotor_R(0,0);
      setMotor_L(0,0);
      estado = 1; 
      break;
      
    case 1: 
      //BUSCAMOS UNA LATA 
      setMotor_R(1,2000);
      setMotor_L(1,2000);
      if (bit_2 != 2){
        estado = 2; 
      }
      break; 
      
    case 2: 
    //CENTRAMOS HACIA LA LATA 
      if (bit_2 == -1){
        setMotor_R(1,2000);
        setMotor_L(-1,2000);
      }else if (bit_2 == 0){
        setMotor_R(0,0);
        setMotor_L(0,0);
      }else if (bit_2 == 1){
        setMotor_R(-1,2000);
        setMotor_L(1,2000);
      }
      if (bit_3 == 1){
        estado=3; 
      }
      break; 
      
    case 3:
      //PREPARACIÓN PARA RECOGER 
      baja_rampa_recoge();
      delay(1000);
      abajo_mec();
      if (digitalRead(end_stop1)== LOW){
        detiene_mec();
        estado = 4; 
      }
      break; 
      
    case 4: 
      //AVANZAMOS Y ACTIVAMOS LAS BANDAS DE RECOLECCIÓN 
      recoge();
      setMotor_R(1,400);
      setMotor_L(1,400); 
      delay(3000); 
      estado=5;
      break; 
      
    case 5: 
      //AUMENTAMOS EL CONTADOR DE LATAS Y SUBIMOS EL MECANISMO
      //NO APAGAMOS LA BARREDORA PARA MANTENER LAS LATAS 
      contador_latas=contador_latas + 1; 
      setMotor_R(0,0);
      setMotor_L(0,0);
      arriba_mec(); 
      if (digitalRead(end_stop2)== LOW){ 
        estado=6; 
      }
      break; 
      
    case 6:  
      //DETENEMOS TODOS LOS MOTORES
      detiene_mec(); 
      detiene_barredora(); 
      sube_rampa(); 
      //SI TENEMOS MENOS DE 3 LATAS ENTONCES SEGUIMOS BUSCANDO 
      //SI YA TENEMOS 3 ENTONCES BUSCAMOS EL CONTENEDOR 
      if (contador_latas < 3){
        estado = 1; 
      }else if (contador_latas == 3){
        estado = 7; 
      }
      break;
        
    case 7: 
      //DOY VUELTAS PARA VER SI ENCUENTRO EL CONTENEDOR 
      setMotor_R(-1,2000);
      setMotor_L(1,2000);
      delay(2000);  
      if (bit_4 == 2){
        setMotor_R(1,2000);
        setMotor_L(1,2000);
        delay(5000);  
        estado = 7; 
      }else if (bit_4 != 2){
        estado = 8; 
      }
      break;
      
    case 8:
      //YA ENCONTRÉ EL CONTENEDOR Y ME CENTRO
      if (bit_4 == -1){
        setMotor_R(1,2000);
        setMotor_L(-1,2000);
      }else if (bit_4 == 0){
        estado = 9; 
      }else if (bit_4 == 1){
        setMotor_R(-1,2000);
        setMotor_L(1,2000); //ACAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
      }
      break;
       
    case 9: 
      //ACERCO AL CONTENEDOR 
      setMotor_R(1,2000);
      setMotor_L(1,2000); 
      if (bit_5 == 1){
        estado = 10;
      }
      break; 
      
    case 10: 
      //DEPOSITO LAS LATAS
      baja_rampa_deposita(); 
      delay(1000); 
      deposita(); 
      delay(5000); 
      estado = 11; 
      
    case 11: 
      //RETROCEDO Y CAMBIO DE DIRECCIÓN 
      setMotor_R(-1,2000);
      setMotor_L(-1,2000);
      delay(2000); 
      setMotor_R(1,2000);
      setMotor_L(-1,2000);
      delay(5000); 
      estado = 0; 
  }
  Serial.println(estado);
}


//----------------------------------------------------------------------------
//-----------ULTRASÓNICOS----------------------------------------


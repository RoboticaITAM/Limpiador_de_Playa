//-----------------pins-----------------
#include <Wire.h>
#include <LSM6.h>

#define ML_pwm 26
#define MR_pwm 33
#define ML 25 
#define MR 32

#define MLenc_A 16    // Amarillo
#define MLenc_B 4    // Blanco
#define MRenc_A 2    // Amarillo
#define MRenc_B 15    // Blanco

LSM6 imu;
//-----------------variables-----------------

// Configurando los parámetros de PWM
int channel_Left = 0;
int channel_Right = 1;
int freq = 1000;
int resolution = 12;

float left_wheel = 0;
float right_wheel = 0;

volatile long L_pulses = 0; 
volatile long R_pulses = 0; 

long previous_Time = 0;
int previos_L_pulses = 0;
int previos_R_pulses = 0;
float velocity_L = 0;
float velocity_R = 0;
float delta_Time;
long current_Time;

float PPR = 480;
float rps_L = 0;
float rpm_L = 0;
float rps_R = 0;
float rpm_R = 0;

float Omega_L = 0;
float Omega_R = 0;

float VL = 0;
float VR = 0;

float VL_Filt = 0;
float VL_Prev = 0;

float VR_Filt = 0;
float VR_Prev = 0;

float W_Radius = 0.03;
float Width = 0.2;

//float VL_Target = 0.3;
//float VR_Target = 0.3;

float kp_L = 30000;//300;
float ki_L = 100;//6.5;

float kp_R = 38000;//300;
float ki_R = 500;//6.5;

float e_L = 0;
float u_L = 0;
float e_L_integral = 0;
  
int pwm_L = 0;
int dir_L = 0;


float e_R = 0;
float u_R = 0;
float e_R_integral = 0;
  
int pwm_R = 0;
int dir_R = 0;

//---------------Variables IMU---------------
long tiempo_prev;
float dt;
float ang_z;
float ang_z_prev;


//-----------------interrupts-----------------
void IRAM_ATTR encoderL(){
  if (digitalRead(MLenc_B) == HIGH){     // si B es HIGH, sentido horario
    L_pulses-- ;        // decrementa L_PULSES en 1
  }
  else {          // si B es LOW, sentido anti horario
    L_pulses++ ;        // incrementa L_PULSES en 1
  }
}

void IRAM_ATTR encoderR(){
  if (digitalRead(MRenc_B) == HIGH){     // si B es HIGH, sentido horario
    R_pulses-- ;        // incrementa R_PULSES en 1
  }
  else {          // si B es LOW, sentido anti horario
    R_pulses++ ;        // decrementa R_PULSES en 1
  }
}
//-----------------setup-----------------

void setup() {
  pinMode(ML_pwm, OUTPUT);
  pinMode(MR, OUTPUT);
  pinMode(ML, OUTPUT);
  pinMode(MR_pwm, OUTPUT);

  pinMode(MLenc_A, INPUT);    
  pinMode(MLenc_B, INPUT);
  pinMode(MRenc_A, INPUT);    
  pinMode(MRenc_B, INPUT);

  ledcSetup(channel_Left, freq, resolution);
  ledcSetup(channel_Right, freq, resolution);

  ledcAttachPin(ML_pwm, channel_Left);
  ledcAttachPin(MR_pwm, channel_Right);

  attachInterrupt(digitalPinToInterrupt(MLenc_A), encoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(MRenc_A), encoderR, RISING);
  
  delay(1000);

  //PARTE IMU-V5
  Wire.begin();

  if (!imu.init()) {
    Serial.println("Failed to detect the LSM6.");
    while (1);
  }

  Serial.println("LSM6 detected!");

  imu.enableDefault();
  tiempo_prev=millis();

  Serial.begin(115200);
}


//-----------------loop-----------------

void loop() {
  Control(0.3,0);
  delay(2000); 
  Control(0,0);
  delay(1000); 
  Control(-0.3,0);
  delay(2+000);
  Control(0,0);
  delay(1000);
  Control(0, 0.3);
  delay(5000);
  Control(0,0);
  delay(1000);
  Control(0,-0.3);
  delay(5000); 
  Control(0,0);
  delay(1000);
  
}

void Measure_velocity(){
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
  

// Low-pass filter (25 Hz cutoff)
  VL_Filt = 0.854*VL_Filt + 0.0728*VL + 0.0728*VL_Prev;
  VL_Prev = VL;
  
  VR_Filt = 0.854*VR_Filt + 0.0728*VR + 0.0728*VR_Prev;
  VR_Prev = VR;
  
}

void Control(float VL_Target,float VW_Target){

  imu.init();

  float gyroZ = imu.g.z;

  float deg_Z = map(gyroZ,-32768,32767,-245,245);

  //CALCULA W EN RAD/S
  float rad_Z= deg_Z*PI/180;
  float kp_w=10000; 
  float e_w, u_w;
  //Manipulación de la señal de control u
  e_L = VL_Target - VL_Filt;
  e_L_integral = e_L_integral + e_L * delta_Time;

  e_R = VL_Target - VR_Filt;
  e_R_integral = e_R_integral + e_R * delta_Time;

  e_w = VW_Target - rad_Z;
  u_w= kp_w * e_w; 
  
  u_L = kp_L*e_L + ki_L*e_L_integral - u_w;
  u_R = kp_R*e_R + ki_R*e_R_integral + u_w;


   // Set the motor speed and direction
  if (u_L > 0){
    dir_L = 1; // hacia adelante
  }
    
  else if (u_L < 0){
    dir_L = -1;   //hacia atrás
  }
    
  else {
    dir_L = 0;   //Detenido
  }

  //Configuras PWM motorL
  pwm_L = (int) fabs(u_L);
  if(pwm_L > 4095){
    pwm_L = 4095;
  }
  
  setMotor_L(dir_L,pwm_L);
  // Set the motor speed and direction
  if (u_R > 0){
    dir_R = 1; // hacia adelante
  }
    
  else if (u_R < 0){
    dir_R = -1;   //hacia atrás
  }
    
  else {
    dir_L = 0;   //Detenido
  }

  //Configuras el PWM motorR
  pwm_R = (int) fabs(u_R);
  if(pwm_R > 4095){
    pwm_R = 4095;
  }
  setMotor_R(dir_R,pwm_R);
}

void setMotor_L(int dir_L, int pwmVal_L){
  // Motor speed
  if(dir_L == 1){ 
    // Turn forward
    ledcWrite(channel_Left, pwmVal_L);
    digitalWrite(ML, HIGH);
    
  }
  else if(dir_L == -1){
    // Turn the other way
    ledcWrite(channel_Left, pwmVal_L);
    digitalWrite(ML, LOW);
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
    digitalWrite(MR , HIGH);
  }
  else if(dir_R == -1){
    ledcWrite(channel_Right, pwmVal_R);
    // Turn the other way
    digitalWrite(MR, LOW);
  }
  else if(dir_R ==0){
    ledcWrite(channel_Right, 0);  
  }
}

void Robot(int v, int w){
  if(v == 1 and w == 0){
    Measure_velocity();
    Control(0.3,0.3); 
  }else if(v == -1 and w == 0){
    Measure_velocity(); 
    Control(-0.3,-0.3);
  }else if(v == 0 and w == 1){
    Measure_velocity();
    Control(-0.3,0.3);
  }else if(v == 0 and w == -1){
    Measure_velocity();
    Control(0.3,-0.3);
  }else{
    Measure_velocity();
    Control(0.0,0.0);
  }
}
void controlGiro(float angulo){
  posicion(); 
  if(angulo < 0){
    if (ang_z  > angulo){
      Robot(0,-1); 
    }else{
      Robot(0,0); 
    }
  }else{
    if (ang_z < angulo){
      Robot(0,1);
    }else{
      Robot(0,0); 
    }
    
  }
}
void posicion(){
  imu.read();

  float accelX = imu.a.x;
  float accelY = imu.a.y;
  float accelZ = imu.a.z;

  float gyroX = imu.g.x;
  float gyroY = imu.g.y;
  float gyroZ = imu.g.z;
  
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  float dif=0;
  
  // Calcula la magnitud del vector de aceleración
  float accelMagnitude = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));

  // Establece un umbral de movimiento
  float umbralMovimiento = 17000; // Ajusta este valor según sea necesario

  // Solo actualiza el ángulo si la magnitud del vector de aceleración supera el umbral0
  if (accelMagnitude > umbralMovimiento) {
    ang_z = ang_z_prev + (gyroZ / 131) * dt;
    ang_z_prev = ang_z;
  }
  Serial.println(ang_z);
}

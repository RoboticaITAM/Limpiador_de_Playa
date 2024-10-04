//-----------------Pines-----------------
#include <Wire.h>
#include <LSM6.h>

#define motor_izq_pwm 26
#define motor_der_pwm 33
#define motor_der_dir 25 
#define motor_izq_dir 32

#define encoder_izq_A 16    // Amarillo
#define encoder_izq_B 4    // Blanco
#define encoder_der_A 2    // Amarillo
#define encoder_der_B 15    // Blanco

//-----------------Variables-----------------

LSM6 imu; 

// Configurando los parámetros de PWM
int pwm_L = 0;
int dir_L = 0;
int pwm_R = 0;
int dir_R = 0;
int channel_Left = 0;
int channel_Right = 1;
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

//-----------------Setup-------------------------
void setup() {

  Serial.begin(115200);
  
  //Configuracion de pines
  pinMode(motor_izq_pwm, OUTPUT);
  pinMode(motor_der_pwm, OUTPUT);
  pinMode(motor_izq_dir, OUTPUT);
  pinMode(motor_der_dir, OUTPUT);
  pinMode(encoder_izq_A, INPUT);    
  pinMode(encoder_izq_B, INPUT);
  pinMode(encoder_der_A, INPUT);    
  pinMode(encoder_der_B, INPUT);

  //Configuración PWM
  ledcSetup(channel_Left, freq, resolution);
  ledcSetup(channel_Right, freq, resolution);
  ledcAttachPin(motor_izq_pwm, channel_Left);
  ledcAttachPin(motor_der_pwm, channel_Right);

  //Activa interrupciones 
  attachInterrupt(digitalPinToInterrupt(encoder_izq_A), encoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_der_A), encoderR, RISING);
  delay(1000);

  //Inicializo la IMU 
  Wire.begin();

  if (!imu.init()) {
    Serial.println("Failed to detect the LSM6.");
    while (1);
  }
  Serial.println("LSM6 detected!");
  imu.enableDefault();
  
  tiempo_prev=millis();
}


//-----------------Loop-----------------
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
  
  Calcula_velocidad();//CAMBIÉ ESTO 
  imu.init();

  //Obtenemos valores de la IMU en el eje z
  float gyroZ = imu.g.z;
  float deg_Z = map(gyroZ,-32768,32767,-245,245);
  
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

//----------------------------------------------------------------------------

//----------FUNCIONES QUE PUEDEN SERVIR EN UN FUTURO---------------------------

//FUNCIÓN PARA CONTROL DE GIRO 
void controlGiro(float angulo){
  posicion(); 
  if(angulo < 0){
    if (ang_z  > angulo){
      Control(0,-1); 
    }else{
      Control(0,0); 
    }
  }else{
    if (ang_z < angulo){
      Control(0,1);
    }else{
      Control(0,0); 
    }  
  }
}

//Obtiene la posición por la IMU
//Si no hay cambio significativo en la posición la variable deg_z no se actualiza
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

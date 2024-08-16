//-----------------definiciones----------------
#define EnML 15
#define ML_A 2
#define ML_B 4

#define EnMR 5
#define MR_A 16
#define MR_B 17

#define pot 39

#define LED_L 33
#define LED_F 32
#define LED_R 18

#define echoPin_L 12 
#define trigPin_L 13 

#define echoPin_F 27 
#define trigPin_F 14 

#define echoPin_R 25 
#define trigPin_R 26 

//-----------------variables----------------
long duration_L;
long duration_F;
long duration_R;

int dist_L;
int dist_F;
int dist_R;

// Configurando los parámetros de PWM
const int frecuencia = 5000;
const int canal_L = 0;
const int canal_R = 1;
const int resolucion = 12;

int vel = 0;

int estado = 0;

//-----------------setup----------------
void setup() {
  pinMode(EnML, OUTPUT);
  pinMode(ML_A, OUTPUT);
  pinMode(ML_B, OUTPUT);

  pinMode(EnMR, OUTPUT);
  pinMode(MR_A, OUTPUT);
  pinMode(MR_B, OUTPUT);
  
  ledcSetup(canal_L, frecuencia, resolucion);
  ledcSetup(canal_R, frecuencia, resolucion);
  
  ledcAttachPin(EnML, canal_L);
  ledcAttachPin(EnMR, canal_R);
  
  pinMode(LED_L, OUTPUT);
  pinMode(LED_F, OUTPUT);
  pinMode(LED_R, OUTPUT);
  
  pinMode(trigPin_L, OUTPUT);   // Se define el pin trigger como SALIDA
  pinMode(echoPin_L, INPUT);    // Se define el pin echo como entrada
  pinMode(trigPin_F, OUTPUT);   // Se define el pin trigger como SALIDA
  pinMode(echoPin_F, INPUT);    // Se define el pin echo como entrada
  pinMode(trigPin_R, OUTPUT);   // Se define el pin trigger como SALIDA
  pinMode(echoPin_R, INPUT);    // Se define el pin echo como entrada

  Serial.begin(115200);
}

//-----------------loop----------------
void loop() {
  switch (estado){

    case 0:
      Serial.println("Estado 0");
      estado = 1;
    break;

    case 1:
      Serial.println("Estado 1");
      US_front();
      if (dist_F > 12){
        estado = 2;
      }
      else if (dist_F < 12){
      //else  {estado = 5;}
        estado = 5;
      }
    break;

    case 2:
      //Forward
      Serial.println("Estado 2");
      Forward();
      US_left();
      US_right();
      if (dist_L < 7){
        estado = 3;
      }
      else if (dist_R < 7){
        estado = 4;
      }
      else{
        estado = 1;
      }
    break;

    case 3:
      //Corrige a la derecha
      Serial.println("Estado 3");
      Right();
      delay(50);
      estado = 1;
    break;

    case 4:
      //Corrige a la izquierda
      Serial.println("Estado 4");
      Left();
      delay(50);
      estado = 1;
    break;

    case 5:
      //Obstaculo de frente
      Serial.println("Estado 5");
      US_left();
      US_right();
      if (dist_L < 12){
        estado = 6;
      }
      else if (dist_R < 12){
        estado = 7;
      }
    break;

    case 6:
      //Giro 90 a la izquierda
      Serial.println("Estado 6");
      Left();
      delay(500);
      estado = 1;
    break;

    case 7:
      //Giro 90 a la derecha
      Serial.println("Estado 7");
      Right();
      delay(500);
      estado = 1;
    break;
  }

}

//-----------------funciones----------------
void Forward(){
  digitalWrite(ML_A, LOW);
  digitalWrite(ML_B, HIGH);
  digitalWrite(MR_A, LOW);
  digitalWrite(MR_B, HIGH);  
  ledcWrite(canal_L, 4000);
  ledcWrite(canal_R, 4000);
  delay(50);
  ledcWrite(canal_L, vel);
  ledcWrite(canal_R, vel);
  Serial.println("Forward");
}

void Back(){
  digitalWrite(ML_A, HIGH);
  digitalWrite(ML_B, LOW); 
  digitalWrite(MR_A, HIGH);
  digitalWrite(MR_B, LOW); 
  ledcWrite(canal_L, 3000);
  ledcWrite(canal_R, 3000);
  delay(20);
  ledcWrite(canal_L, vel);
  ledcWrite(canal_R, vel);
  Serial.println("Back");
}

void Stop(){
  digitalWrite(ML_A, LOW);
  digitalWrite(ML_B, LOW);
  digitalWrite(MR_A, LOW);
  digitalWrite(MR_B, LOW);
  Serial.println("Stopped");  
}

void Right(){
  digitalWrite(ML_A, LOW);
  digitalWrite(ML_B, HIGH);
  digitalWrite(MR_A, HIGH);
  digitalWrite(MR_B, LOW);
  ledcWrite(canal_L, 3000);
  ledcWrite(canal_R, 3000);
  delay(20);
  ledcWrite(canal_L, 0.56*vel);
  ledcWrite(canal_R, 0.56*vel);
  Serial.println("Right");
}

void Left(){
  digitalWrite(ML_A, HIGH);
  digitalWrite(ML_B, LOW);
  digitalWrite(MR_A, LOW);
  digitalWrite(MR_B, HIGH);
  ledcWrite(canal_L, 3000);
  ledcWrite(canal_R, 3000);
  delay(20);
  ledcWrite(canal_L, 0.56*vel);
  ledcWrite(canal_R, 0.56*vel);
  Serial.println("Left");
}

void Slow_Forward(){
  digitalWrite(ML_A, LOW);
  digitalWrite(ML_B, HIGH);
  digitalWrite(MR_A, LOW);
  digitalWrite(MR_B, HIGH);
  ledcWrite(canal_L, 3000);
  ledcWrite(canal_R, 3000);
  delay(20);
  ledcWrite(canal_L, 0.56*vel);
  ledcWrite(canal_R, 0.56*vel);
  Serial.println("Slow_Forward");
}

void Slow_Back(){
  digitalWrite(ML_A, HIGH);
  digitalWrite(ML_B, LOW); 
  digitalWrite(MR_A, HIGH);
  digitalWrite(MR_B, LOW);
  ledcWrite(canal_L, 3000);
  ledcWrite(canal_R, 3000);
  delay(20);
  ledcWrite(canal_L, 0.56*vel);
  ledcWrite(canal_R, 0.56*vel);
  Serial.println("Slow_Back");
}

int US_left(){
  digitalWrite(trigPin_L, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin_L, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_L, LOW);
  
  duration_L = pulseIn(echoPin_L, HIGH);
  dist_L = duration_L * 0.0343 / 2;

  if(dist_L < 7){
    digitalWrite(LED_L, HIGH);
  }
  else digitalWrite(LED_L, LOW);
  
  return dist_L;
}

int US_front(){
  digitalWrite(trigPin_F, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin_F, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_F, LOW);
  
  duration_F = pulseIn(echoPin_F, HIGH);
  dist_F = duration_F * 0.0343 / 2;

  if(dist_F < 10){
    digitalWrite(LED_F, HIGH);
  }
  else digitalWrite(LED_F, LOW);
  
  return dist_F;
}

int US_right(){
  digitalWrite(trigPin_R, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin_R, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_R, LOW);
  
  duration_R = pulseIn(echoPin_R, HIGH);
  dist_R = duration_R * 0.0343 / 2;

  if(dist_R < 7){
    digitalWrite(LED_R, HIGH);
  }
  else digitalWrite(LED_R, LOW);
  
  return dist_R;
}

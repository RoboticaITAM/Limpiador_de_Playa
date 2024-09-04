int bit_0;
int bit_1;
int bit_2;
int bit_3;
int bit_4;
String prueba;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
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
    prueba = valores[4];
    bit_0 = valores[0].toInt();
    bit_1 = valores[1].toInt();
    bit_2 = valores[2].toInt();
    bit_3 = valores[3].toInt();
    bit_4 = valores[4].toInt();
  
    Serial.print(bit_0);
    Serial.print(", ");
    Serial.print(bit_1);
    Serial.print(", ");
    Serial.print(bit_2);
    Serial.print(", ");
    Serial.print(bit_3);
    Serial.print(", ");
    Serial.println(bit_4);
  }
}
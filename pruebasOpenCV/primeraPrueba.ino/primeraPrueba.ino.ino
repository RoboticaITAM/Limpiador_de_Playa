void setup() {
  Serial.begin(115200);  // Asegúrate de que la tasa de baudios coincide con la de Python
}

void loop() {
  if (Serial.available() > 0) {
    String datos = Serial.readStringUntil('\n');  // Lee los datos hasta que encuentre un salto de línea
    Serial.println(datos);  // Solo para debug, quitar en versión final
    // Aquí deberías añadir tu lógica para manejar 'datos'
  }
}

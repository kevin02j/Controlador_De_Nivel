#define TRIGGER_PIN 4
#define ECHO_PIN 2

void setup() {
  Serial.begin(9600);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Envía un pulso corto al pin de trigger para activar el sensor
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration * 0.034) / 2;

  // Imprime la distancia medida
  Serial.println(distance);

  // Espera un momento antes de tomar otra lectura
  delay(1000);
}


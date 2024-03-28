#define TRIGGER_PIN 4
#define ECHO_PIN 2

#define NUM_READINGS 5

int readingsArray[NUM_READINGS];
int sumReadings = 0;
int filteredDistance;

void setup() {
  Serial.begin(9600);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Enviar pulso y obtener duración
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calcular distancia sin filtrar
  int distance = (duration * 0.034) / 2;

  // Almacenar nueva lectura en el arreglo
  for (int i = NUM_READINGS - 1; i > 0; i--) {
    readingsArray[i] = readingsArray[i - 1];
  }
  readingsArray[0] = distance;

  // Calcular suma y promedio de las lecturas
  sumReadings = 0;
  for (int i = 0; i < NUM_READINGS; i++) {
    sumReadings += readingsArray[i];
  }
  filteredDistance = sumReadings / NUM_READINGS;

  // Imprimir distancia filtrada
  Serial.println(filteredDistance);

  delay(100);
}


#define EchoPin 12
#define TriggerPin 13

void setup() {
  Serial.begin(115200);
  pinMode(PinElectroBomba, OUTPUT);
}

void loop() {
  DISTANCIA = 0.01723 * readUltrasonicDistance(TriggerPin, EchoPin);
  String DataSend = String(DISTANCIA);
  Serial.println(DataSend);
}

long readUltrasonicDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}

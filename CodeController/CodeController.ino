// #define TRIGGER_PIN 4
// #define ECHO_PIN 2
// Pins
const int echo = 2;
const int trig = 4;
// Constrains
const int maxdist = 335;
const float mindist = 2.5;

double distance, duration;
double kaldist;

double kalman(double U) {
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 100;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P * H / (H * P * H + R);
  U_hat += +K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}

void setup() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  Serial.begin(9600);
}

void usonic_transmit() {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
}

void loop() {
  usonic_transmit();

  duration = pulseIn(echo, HIGH);
  distance = (duration * .034) / 2;
  kaldist = kalman(distance);

  Serial.print("Distance (in cm): ");
  Serial.println(distance);
  Serial.print("Corrected distance (in cm): ");
  Serial.println(kaldist);

  delay(500);
}
#include <NewPing.h>

#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define MAX_DISTANCE 50  // Distancia máxima a medir (en centímetros)

// Objeto para manejar el sensor HC-SR04
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

const int bombaIn = 5;
const int bombaOut = 6;

String frameReceived;

// Definición de los coeficientes del filtro FIR
const int N = 3;                                                         // Longitud del filtro (número de coeficientes)
const float h[N] = { 0.1, 0.1, 0.1 };  // Coeficientes del filtro

// Variables para almacenar las muestras de entrada y salida
float x[N] = { 0 };  // Buffer circular para almacenar las muestras de entrada
float y = 0;         // Muestra de salida filtrada

// Variables de trabajo del PID
int lastTime;
int Input, Output, Setpoint, error, errorN;
int ITerm, lastInput;
double kp =35.0, ki = 5.8, kd = 0.71;
int SampleTime = 200;  // Tiempo de muestreo 0.5 segundos.
int outMin = 0, outMax = 255;
bool inAuto = false;
#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;

void setup() {
  pinMode(bombaIn, OUTPUT);
  pinMode(bombaOut, OUTPUT);
  analogWrite(bombaIn, 0);
  analogWrite(bombaOut, 0);
  Serial.begin(9600);
}

void loop() {
  unsigned int distance = sonar.ping_cm();
  //Serial.println(distance);
  int level = map(distance, 13, 3, 1, 100);
  // int level = filter(distance);
  Input=level;
  Setpoint = Lectura_SetPoint();
  SetTunings(kp, ki, kd);
  SetSampleTime(SampleTime);
  SetOutputLimits(outMin, outMax);
  SetMode(1);
  SetControllerDirection(1);
  Compute();
  AcciondeControl();
  // Serial.println("Set point: " + String(Setpoint));
  // Serial.println("Level: " + String(Input));
  //Serial.println("PWM1: " + String(Output));
  Serial.println(String(Input) + "," + String(Setpoint));
  delay(SampleTime);
}

float Lectura_SetPoint() {
  int ADC_Channel_1 = analogRead(A7);
  int reference = map(ADC_Channel_1, 0, 1023, 10, 90);
  return reference;
}

int filter(int input) {
  // Agregar la nueva muestra al buffer circular
  for (int i = N - 1; i > 0; i--) {
    x[i] = x[i - 1];
  }
  x[0] = input;

  // Calcular la muestra de salida filtrada
  y = 0;
  for (int i = 0; i < N; i++) {
    y += h[i] * x[i];
  }
  int level = map(y, 17, 3, 1, 100);
  return level;
}

void AcciondeControl() {
  //Serial.println("Error: " + String(error));
  int pwm_bomba_2 = map(Output, 0, 255, 255, 0);
  //Serial.println("PWM2: " + String(pwm_bomba_2));
  analogWrite(bombaOut, pwm_bomba_2);
  analogWrite(bombaIn, Output);
  // if (errorN >= 0) {
  //   Serial.println("llenar");
  //   analogWrite(bombaOut, Output);
  //   analogWrite(bombaIn, 0);
  // } else {
  //   Serial.println("Vaciar");
  //   analogWrite(bombaOut, 0);
  //   analogWrite(bombaIn, Output);
  // }
}


void Compute() {
  if (!inAuto) return;
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if (timeChange >= SampleTime) {
    // Calculamos todos los errores.
    error = Setpoint - Input;
    // if (errorN < 0) {
    //   error = abs(errorN);
    // } else {
    //   error = errorN;
    // }
    ITerm += (ki * error);
    if (ITerm > outMax) ITerm = outMax;
    else if (ITerm < outMin) ITerm = outMin;
    double dInput = (Input - lastInput);
    // Calculamos la función de salida del PID.
    Output = kp * error + ITerm - kd * dInput;
    if (Output > outMax) Output = outMax;
    else if (Output < outMin) Output = outMin;
    //Guardamos el valor de algunas variables para el próximo recálculo
    lastInput = Input;
    lastTime = now;
  }
}

void SetTunings(double Kp, double Ki, double Kd) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;
  double SampleTimeInSec = ((double)SampleTime) / 1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
  if (controllerDirection == REVERSE) {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
}

void SetSampleTime(int NewSampleTime) {
  if (NewSampleTime > 0) {
    double ratio = (double)NewSampleTime / (double)SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}

void SetOutputLimits(double Min, double Max) {
  if (Min > Max) return;
  outMin = Min;
  outMax = Max;
  if (Output > outMax) Output = outMax;
  else if (Output < outMin) Output = outMin;
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
}

void SetMode(int Mode) {
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto) {
    // Para cambiar de manual a automático, inicializamos algunos parámetros.
    Initialize();
  }
  inAuto = newAuto;
}

void Initialize() {
  lastInput = Input;
  ITerm = Output;
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
}

void SetControllerDirection(int Direction) {
  controllerDirection = Direction;
}

#include <NewPing.h>

#define TRIGGER_PIN  4  
#define ECHO_PIN     5 
#define MAX_DISTANCE 50 // Distancia máxima a medir (en centímetros)

// Objeto para manejar el sensor HC-SR04
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

const int bombaIn = 3;
const int bombaOut = 2;

String frameReceived;

// Definición de los coeficientes del filtro FIR
const int N = 10; // Longitud del filtro (número de coeficientes)
const float h[N] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; // Coeficientes del filtro

// Variables para almacenar las muestras de entrada y salida
float x[N] = {0}; // Buffer circular para almacenar las muestras de entrada
float y = 0; // Muestra de salida filtrada

// Variables de trabajo del PID
unsigned long lastTime;
double Input, Output, Setpoint, error;
double ITerm, lastInput;
double kp = 32.2, ki = 10.0, kd = 2.5;
int SampleTime = 500;  // Tiempo de muestreo 0.5 segundos.
double outMin = 0, outMax = 255;
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
  if (Serial.available() > 0) {
    unsigned int distance = sonar.ping_cm();
    Input  = filter(distance);
    int ModeOperation = ReadSerialPort();

    if (ModeOperation == 2) {
      //Modo Automatico
      String Adjust = frameReceived.substring(1);
      Setpoint = Adjust.toInt();
      SetTunings(kp, ki, kd);
      SetSampleTime(SampleTime);
      SetOutputLimits(outMin, outMax);
      SetMode(1);
      SetControllerDirection(1);
      Compute();
      AcciondeControl();
    } else if (ModeOperation == 1) {
      //Modo Manual
      char ingreso = frameReceived.charAt(1);
      char salida = frameReceived.charAt(2);
      int stateBombaIN = ingreso - '0';
      int stateBombaOUT = salida - '0';

      if (stateBombaIN == 1) {
        analogWrite(bombaIn, 255);
      } else {
        analogWrite(bombaIn, 0);
      }
      if (stateBombaOUT == 1) {
        analogWrite(bombaOut, 255);
      } else {
        analogWrite(bombaOut, 0);
      }
      Serial.println(Input);
      delay(100);
    }
  }
}
int ReadSerialPort() {
  if (Serial.available()) {
    frameReceived = Serial.read();
    char modePos = frameReceived.charAt(0);
    if (modePos == '1' || modePos == '2') {
      return modePos - '0';
    }
  }
  return -1;  // Retornar un valor inválido si no se recibió un modo válido
}

float filter(float input) {
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

  return y;
}

void sendInfo(int bomba1, int bomba2) {
  String frameToSend = String(bomba1) + "," + String(bomba2) + "," + String(Input);
  Serial.println(frameToSend);
}

void AcciondeControl() {
  if (error <= 0) {
    if (Output == 255) {
      analogWrite(bombaOut, 255);
      analogWrite(bombaIn, 50);
      sendInfo(50, 255);
    } else {
      analogWrite(bombaOut, Output);
      analogWrite(bombaIn, 50);
      sendInfo(50, Output);
    }
  } else {
    if (Output == 255) {
      analogWrite(bombaIn, 255);
      analogWrite(bombaOut, 50);
      sendInfo(255, 50);
    } else {
      analogWrite(bombaIn, Output);
      analogWrite(bombaOut, 50);
      sendInfo(Output, 50);
    }
  }
}

void Compute() {
  if (!inAuto) return;
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if (timeChange >= SampleTime) {
    // Calculamos todos los errores.
    error = Setpoint - Input;
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

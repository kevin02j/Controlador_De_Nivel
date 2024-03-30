const int echoPin = 5;
const int trigPin = 18;
const float soundSpeed = 0.034;  // Speed of sound in cm/microsecond
const int bombaIn = 19;
const int bombaOut = 21;

const int dutyCycle = 120;

String frameReceived;

// Kalman Filter Parameters
const double R = 40;    // Measurement noise
const double H = 1.00;  // Measurement function
double Q = 10;          // Process noise
double P = 0;           // Estimate error covariance
double U_hat = 0;       // A priori estimate
double K = 0;           // Kalman gain

// Variables de trabajo del PID
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp = 32.2, ki = 10.0, kd = 2.5;
int SampleTime = 500;  // Tiempo de muestreo 1 segundo.
double outMin = 1, outMax = 255;
bool inAuto = false;
#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(bombaIn, OUTPUT);
  pinMode(bombaOut, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  double distance = calculateDistance();
  Input = kalmanFilter(distance);
  int ModeOperation = ReadSerialPort();
  if (ModeOperation == 0) {
    //Modo Automatico
    Serial.println(20);
  } else if (ModeOperation == 1) {
    //Modo Manual
    String stateBombaIN = frameReceived.substring(1, 2);
    String stateBombaOUT = frameReceived.substring(2, 3);
    int A1 = stateBombaIN.toInt();
    int A2 = stateBombaOUT.toInt();

    if (A1 == 1) {
      analogWrite(bombaIn, 255);
    } else {
      analogWrite(bombaIn, 0);
    }
    if (A2 == 1) {
      analogWrite(bombaOut, 255);
    } else {
      analogWrite(bombaOut, 0);
    }
    Serial.println(Input);
  }
  delay(100);
}

int ReadSerialPort() {
  if (Serial.available()) {
    frameReceived = Serial.readString();
    if (frameReceived.startsWith("1")) {
      return 1;
    } else {
      return 0;
    }
  }
}

double Read_SetPoint() {
  while (Serial.available()) {
    int Data_Int = Serial.parseInt();
    return Data_Int;
  }
}

void sendPulse() {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

double kalmanFilter(double U) {
  K = P * H / (H * P * H + R);
  U_hat += +K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}

double calculateDistance() {
  sendPulse();
  unsigned long duration = pulseIn(echoPin, HIGH);
  double distance = (duration * soundSpeed) / 2.0;
  return distance;
}

// void AcciondeControl(){
//     if (Output==80){
//       analogWrite(PIN_VENTILADOR, 255);
//       alfa=80;
//     }
//     else {
//       alfa=Output;
//       analogWrite(PIN_VENTILADOR, 255);
//     }
// }
// void Compute() {
//   if(!inAuto) return;
//   unsigned long now = millis();
//   int timeChange = (now - lastTime);
//   if(timeChange>=SampleTime)   {
//     // Calculamos todos los errores.
//     double error = Setpoint - Input;
//     ITerm+= (ki * error);
//     if(ITerm > outMax) ITerm = outMax;
//     else if(ITerm < outMin) ITerm = outMin;
//     double dInput = (Input - lastInput);
//     // Calculamos la función de salida del PID.
//     Output = kp * error + ITerm- kd * dInput;
//     if(Output > outMax) Output = outMax;
//     else if(Output < outMin) Output = outMin;
//     //Guardamos el valor de algunas variables para el próximo recálculo
//     lastInput = Input;
//     lastTime = now;
//   }
// }

// void SetTunings(double Kp, double Ki, double Kd) {
//   if (Kp<0 || Ki<0|| Kd<0) return;
//   double SampleTimeInSec = ((double)SampleTime)/1000;
//   kp = Kp;
//   ki = Ki * SampleTimeInSec;
//   kd = Kd / SampleTimeInSec;
//   if(controllerDirection == REVERSE)   {
//     kp = (0 - kp);
//     ki = (0 - ki);
//     kd = (0 - kd);
//   }
// }

// void SetSampleTime(int NewSampleTime) {
//   if (NewSampleTime > 0)   {
//     double ratio  = (double)NewSampleTime / (double)SampleTime;
//     ki *= ratio;
//     kd /= ratio;
//     SampleTime = (unsigned long)NewSampleTime;
//   }
// }

// void SetOutputLimits(double Min, double Max) {
//   if(Min > Max) return;
//   outMin = Min;
//   outMax = Max;
//   if(Output > outMax) Output = outMax;
//   else if(Output < outMin) Output = outMin;
//   if(ITerm> outMax) ITerm= outMax;
//   else if(ITerm< outMin) ITerm= outMin;
// }

// void SetMode(int Mode) {
//   bool newAuto = (Mode == AUTOMATIC);
//     if(newAuto && !inAuto) {
//       // Para cambiar de manual a automático, inicializamos algunos parámetros.
//       Initialize();
//     }
//   inAuto = newAuto;
// }

// void Initialize() {
//   lastInput = Input;
//   ITerm = Output;
//   if(ITerm> outMax) ITerm= outMax;
//   else if(ITerm< outMin) ITerm= outMin;
// }

// void SetControllerDirection(int Direction) {
//   controllerDirection = Direction;
// }

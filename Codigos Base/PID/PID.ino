#include <TimerOne.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

#define PIN_SENSOR 4
#define PIN_VENTILADOR 5
#define TRIAC 3

LiquidCrystal_I2C lcd(0x27, 16, 2);

OneWire oneWire(PIN_SENSOR);
DallasTemperature sensors(&oneWire);

// Variables de trabajo del PID
unsigned long lastTime; 
double Input, Output, Setpoint; 
double ITerm, lastInput; 
double kp=32.2, ki=10.0, kd=2.5; 
int SampleTime = 500; // Tiempo de muestreo 1 segundo. 
double outMin=10, outMax=80; 
bool inAuto = false; 
#define MANUAL 0 
#define AUTOMATIC 1 
#define DIRECT 0 
#define REVERSE 1 
int controllerDirection = DIRECT; 
volatile int i = 0;  // Controla el tiempo de disparo del TRIAC
volatile boolean cruce_cero = false;
unsigned long previousTime = 0;
int T_int = 100;  // Tiempo de ejecución del Timer1 en uS
int alfa;

//FUNCIONES PARA ACTIVACION DEL TRIAC
void deteccion_Cruce_cero() {
  cruce_cero = true;
  i = 0;
  digitalWrite(TRIAC, LOW);
}

void Dimer() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - previousTime;

  if (cruce_cero) {
    if (i >= alfa) {
      digitalWrite(TRIAC, HIGH);
      i = 0;
      cruce_cero = false;
    } else {
      i++;
    }
  }
  previousTime = currentTime;
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(TRIAC, OUTPUT);
  pinMode(PIN_VENTILADOR, OUTPUT);
  attachInterrupt(0, deteccion_Cruce_cero, RISING);
  Timer1.initialize(T_int);
  Timer1.attachInterrupt(Dimer);
  sensors.begin();
}

//BUCLE PRINCIPAL
void loop(){
  String imprimir = String(Setpoint) + ";" + String(Input);
  Serial.println(imprimir);

  Input=Lectura_Sensor();
  Setpoint=Lectura_SetPoint();
  mostrarLCD();
  SetTunings(kp, ki, kd);
  SetSampleTime(SampleTime);
  SetOutputLimits(outMin, outMax);
  SetMode(1);
  SetControllerDirection(1);
  Compute();
  AcciondeControl();
}

float Lectura_Sensor() {
  sensors.requestTemperatures();
  float t = sensors.getTempCByIndex(0);
  return t;
}

float Lectura_SetPoint() {
  int Potenciometro = analogRead(A0);
  int Set = map(Potenciometro, 0, 1023, 20, 60);
  return Set;
}

void mostrarLCD() {
  lcd.clear();  // Limpiar la pantalla LCD
  lcd.setCursor(0, 0);
  lcd.print("Setpoint: ");
  lcd.setCursor(11, 0);
  lcd.print(Setpoint);
  lcd.setCursor(0, 1);
  lcd.print("Temp:");
  lcd.setCursor(7, 1);
  lcd.print(Input);
}

void AcciondeControl(){
    if (Output==80){
      analogWrite(PIN_VENTILADOR, 255);
      alfa=80;
    }
    else {
      alfa=Output;
      analogWrite(PIN_VENTILADOR, 255);
    }
}
void Compute() {
  if(!inAuto) return;   
  unsigned long now = millis();   
  int timeChange = (now - lastTime);   
  if(timeChange>=SampleTime)   { 
    // Calculamos todos los errores.      
    double error = Setpoint - Input;      
    ITerm+= (ki * error);      
    if(ITerm > outMax) ITerm = outMax;      
    else if(ITerm < outMin) ITerm = outMin;      
    double dInput = (Input - lastInput);      
    // Calculamos la función de salida del PID.      
    Output = kp * error + ITerm- kd * dInput;   
    if(Output > outMax) Output = outMax;      
    else if(Output < outMin) Output = outMin;
    //Guardamos el valor de algunas variables para el próximo recálculo
    lastInput = Input;
    lastTime = now;   
  } 
}

void SetTunings(double Kp, double Ki, double Kd) {   
  if (Kp<0 || Ki<0|| Kd<0) return;  
  double SampleTimeInSec = ((double)SampleTime)/1000;   
  kp = Kp;   
  ki = Ki * SampleTimeInSec;   
  kd = Kd / SampleTimeInSec;  
  if(controllerDirection == REVERSE)   {      
    kp = (0 - kp);      
    ki = (0 - ki);         
    kd = (0 - kd);   
  } 
}

void SetSampleTime(int NewSampleTime) {   
  if (NewSampleTime > 0)   {      
    double ratio  = (double)NewSampleTime / (double)SampleTime;      
    ki *= ratio;      
    kd /= ratio;      
    SampleTime = (unsigned long)NewSampleTime;   
  }
}

void SetOutputLimits(double Min, double Max) {
  if(Min > Max) return;   
  outMin = Min;   
  outMax = Max;   
  if(Output > outMax) Output = outMax;   
  else if(Output < outMin) Output = outMin;   
  if(ITerm> outMax) ITerm= outMax;   
  else if(ITerm< outMin) ITerm= outMin; 
}

void SetMode(int Mode) {    
  bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto) {  
      // Para cambiar de manual a automático, inicializamos algunos parámetros.        
      Initialize();    
    }    
  inAuto = newAuto; 
}

void Initialize() {   
  lastInput = Input;   
  ITerm = Output;   
  if(ITerm> outMax) ITerm= outMax;   
  else if(ITerm< outMin) ITerm= outMin; 
}

void SetControllerDirection(int Direction) {   
  controllerDirection = Direction; 
}

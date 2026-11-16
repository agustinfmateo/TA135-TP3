// control observador

//-------------------------Librerias---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "driver/mcpwm.h"

//-------------------------Variables---------------------------------
//Pines
const int servoPin = 17;
const int potPin = 35;

//Variables Pote
int potMin = 4095;
int potMax = 0;

//Variables Servo
const int minPulseUs = 500;   // 0.5 ms
const int maxPulseUs = 2450;  // 2.45 ms

const int servo_freq = 50; //El servo MG996R usa pwm de 50Hz
const int angleMin = 60;
const int angleMax = 120;

//Variables IMU
Adafruit_MPU6050 mpu;
float anguloRotAnt = 0;
float alpha = 0.02;

const int periodoMuestreo = 20;   //cada cuanto tiempo se toman muestras en milisegundos
unsigned long tiempoMuestra = 0;

const float angServoEq = 90;

float theta0 = 0;
float gyroX_offset = 0;

//Variables Control
float theta = 0  ;          //theta actual
float u = 0;                //u actual
float u_1=0;                //u-1 anterior
float u_print=0;            //para enviar a imprimir
float theta_e  = 0;         //theta estimada

const int zona_muerta = 4.2;

const unsigned long periodoMuestreo_us = 20000;
unsigned long tiempoMuestra_us = 2;

//Observador
float xe[4] = {0  , 0 , 0  , 0};      // vector de estados estimados Xe
float xe_1[4] = {0  , 0 , 0  , 0};    // vector de estados estimados Xe-1 anterior

float A_fila1[4] = {-1.394192024, -1.538948837, -26.62570789, 5.371708153};
float A_fila2[4] = {0.4787932776, 0.4599361115, 9.286705209, -1.361683934};
float A_fila3[4] = {0.05836927409, -0.004225844645, 1.817082701, 0.8339982901};
float A_fila4[4] = {-0.02566502655, 0.001858107998, -0.4068710149, 1.045291113};

float B_fila1[2] = {0, -126.7376687};
float B_fila2[2] = {0, 32.12695893};
float B_fila3[2] = {0, 3.916569758};
float B_fila4[2] = {0.125, -1.722119529};

float Cd[4] = {-0.03013174712, -0.09990423655, 0.04238446398, 0};
float K[4]  = {-2.086974155e-09, -0.001195349636, 0.02202631879, 1.468740554};

float const const_u = 1;

//--------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  delay(500);

  //--------------------Configuracion del Pote-------------------------------------
  pinMode(potPin, INPUT);
  analogReadResolution(12);

  //--------------------Configuracion del Servo-------------------------------------
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, servoPin);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = servo_freq;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  
  //------------------------Inicializacion de la IMU--------------------------------
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(100);
    }
  }
  Wire.setClock(400000);
  Serial.println("MPU6050 Found!");
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 5-10-21-44-94-184-260 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(50);

  //-------------------------Calibracion---------------------------------
  calibrar();
  delay(1000);
  //tiempoMuestra = millis();
  tiempoMuestra_us = micros();
}

void loop() {
  sensors_event_t a, g, temp;
  unsigned long tiempoActual_us = micros();
  //unsigned long tiempoActual = millis();

  //if((tiempoActual - tiempoMuestra >= periodoMuestreo))
  if (tiempoActual_us - tiempoMuestra_us >= periodoMuestreo_us)
  {
    float dt = (tiempoActual_us - tiempoMuestra_us) / 1000000.0f;
    tiempoMuestra_us += periodoMuestreo_us;
    
    //Mido el pote
    unsigned long a0 = micros();
    int potValor = analogRead(potPin);
    float phi = map(potValor, potMin, potMax, angleMin, angleMax);

    //Mido IMU
    mpu.getEvent(&a, &g, &temp);
    
    float gyrox = g.gyro.x - gyroX_offset;
    //float dt = (tiempoActual - tiempoMuestra) / 1000.0f;
    float anguloRotG = anguloRotAnt + gyrox*dt;
    float anguloRotA = atan2(a.acceleration.y,a.acceleration.z);
    float anguloRot = (1-alpha)*anguloRotG + alpha*anguloRotA;

    anguloRotAnt = anguloRot;
    theta = (anguloRot*RAD_TO_DEG-theta0);  // theta0 el error sistematico de la IMU

    //Control
    control_moderno(theta, &theta_e, &u_1,&u_print);

    //Imprimir datos
    Serial.print(theta, 3);
    Serial.print("\t");
    Serial.print(theta_e, 3);
    Serial.print("\t");
    Serial.println(u_print, 3);

    //tiempoMuestra = tiempoActual;
    //tiempoMuestra_us = tiempoActual_us;
  }
}

//-------------------Funciones Control-------------------------------

void control_moderno(float theta,  float* theta_e, float* u_1, float* u_print)
{
  xe[0]= (A_fila1[0]*xe_1[0]+ A_fila1[1]*xe_1[1]+A_fila1[2]*xe_1[2]+A_fila1[3]*xe_1[3]) + B_fila1[0]* (*u_1) + B_fila1[1]*theta;
  xe[1]= (A_fila2[0]*xe_1[0]+ A_fila2[1]*xe_1[1]+A_fila2[2]*xe_1[2]+A_fila2[3]*xe_1[3]) + B_fila2[0]* (*u_1) + B_fila2[1]*theta;
  xe[2]= (A_fila3[0]*xe_1[0]+ A_fila3[1]*xe_1[1]+A_fila3[2]*xe_1[2]+A_fila3[3]*xe_1[3]) + B_fila3[0]* (*u_1) + B_fila3[1]*theta;
  xe[3]= (A_fila4[0]*xe_1[0]+ A_fila4[1]*xe_1[1]+A_fila4[2]*xe_1[2]+A_fila4[3]*xe_1[3]) + B_fila4[0]* (*u_1) + B_fila4[1]*theta;
    
  float u = -const_u*(K[0]*xe[0]+K[1]*xe[1]+K[2]*xe[2]+K[3]*xe[3]);
  float theta_est = Cd[0]*xe[0]+Cd[1]*xe[1]+Cd[2]*xe[2]+Cd[3]*xe[3];

  xe_1[0]= xe[0];
  xe_1[1]= xe[1];
  xe_1[2]= xe[2];
  xe_1[3]= xe[3];

  // Resto la zona muerta cuando se sale
  float u_zm;
  if (u > zona_muerta) {
    u_zm = u - zona_muerta;
  }
  else if (u < -zona_muerta){ 
    u_zm = u + zona_muerta;
  }
  else{
    u_zm = 0;
  }

  float u_sat = constrain(u_zm, angleMin - angServoEq, angleMax - angServoEq);

  *u_1 = u_sat;
  *u_print = u_sat;
  *theta_e = theta_est;

  moverServo(u_sat + angServoEq);
}



//-------------------Funcion Calibracion-----------------------------
void calibrarIMU() {
  Serial.println("Calibrando IMU...");
  delay(500);

  float sumatoriaAngulos = 0;
  float sumatoriaGyro = 0;
  int muestras = 500;
  sensors_event_t a, g, temp;

  for (int i = 0; i < muestras; i++) {
    mpu.getEvent(&a, &g, &temp);
    //Calibrar angulo en equilibrio
    float anguloRotA = atan2(a.acceleration.y, a.acceleration.z); //Solo el acelerometro pq ta quieto
    sumatoriaAngulos += (anguloRotA * 180.0 / PI);
    //Calibrar offset del giroscopio
    sumatoriaGyro += g.gyro.x;
    delay(20);
  }

  theta0 = sumatoriaAngulos / muestras; //promedio
  gyroX_offset = sumatoriaGyro / muestras;

  anguloRotAnt = theta0 * PI / 180.0; 
  
  Serial.print("Calibracion exitosa, Nuevo theta0: ");
  Serial.print(theta0);
  Serial.print("\t Offset Gyro: ");
  Serial.println(gyroX_offset);
}

void calibrar() {
  calibrarIMU();

  moverServo(angServoEq);
  delay(3000);

  Serial.println("Valores antes de Calibrar");
  Serial.println(potMin);
  Serial.println(potMax);

  long suma = 0;
  int muestras = 500;

  moverServo(angleMin);
  delay(1000);
  
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(potPin);
    delay(1);
  }
  int potValor = suma/muestras;

  potMax = max(potMax, potValor);
  potMin = min(potMin, potValor);

  moverServo(angleMax);
  delay(1000);

  suma = 0;
  for (int i = 0; i < muestras; i++) {
    suma += analogRead(potPin);
    delay(1);
  }
  potValor = suma/muestras;

  potMax = max(potMax, potValor);
  potMin = min(potMin, potValor);

  Serial.println("Valores despues de Calibrar");
  Serial.println(potMin);
  Serial.println(potMax);
  
  moverServo(angServoEq);
  delay(2000);
}

//----------------Funciones para movimiento del Servo----------------
void moverServo(float theta) {
  theta = constrain(theta, angleMin, angleMax);
  float ang = theta + 6.0f;
  uint32_t pulso_us = (uint32_t)(minPulseUs + (ang / 180.0f) * (maxPulseUs - minPulseUs));
  
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulso_us);
}

//-----------------------Envio de datos a MATLAB-------------------------
void matlab_send(float dato1, float dato2, float dato3){
  //Serial.write("abcd");
  Serial.write(97);
  Serial.write(98);
  Serial.write(99);
  Serial.write(100);
  
  byte * b0 = (byte *) &dato1;
  Serial.write(b0,4);
  
  byte * b1 = (byte *) &dato2;
  Serial.write(b1,4);

  byte * b2 = (byte *) &dato3;
  Serial.write(b2,4);
}
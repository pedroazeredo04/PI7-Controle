#define DEBUG // comentar se não debug

#include <xc.h>
#include "always.h"
#include "serial.h"
#include "stdio.h"

#include "pid.h"
#include "motor.h"
#include "position_sensor.h"

static _EXCITATION excitation;

static _PID_MATH currentPosition;
static _PID_MATH setPoint;
static _PID_MATH integralErrorScaled;
static _PID_MATH integralError;
static _PID_MATH previousError;
static _PID_MATH kProportional;
static _PID_MATH kIntegral;
static _PID_MATH kDerivative;

_EXCITATION pid_scaleExcitation ( _PID_MATH activation) {
   
   excitation = ( _EXCITATION ) activation;
   if (excitation > 0)
     excitation = 150 + excitation;
   else if (excitation < 0)
     excitation = -150 + excitation;
   if (excitation > MAX_EXCITATION)
     excitation = MAX_EXCITATION;
   if (excitation < MIN_EXCITATION)
     excitation = MIN_EXCITATION;
   return excitation;   
} // scaleExcitation

static   _PID_MATH activation;
static   _PID_MATH error;
static   _EXCITATION excitation;

static _PID_MATH  half_PID_PERIOD = 3; //queremos 3ms mas vamos trabalhar em inteiro primeiro entao x1000
static _PID_MATH  PID_PERIOD = 6;
static   _PID_MATH previousPosition;

void pid_pid(void) {
//testes sem protocolo
 
#ifdef DEBUG
  RA4 = 1; // usado para saber quanto tempo o controle demora, ver pino 6 do PIC no oscilosc?pio
#endif
  
//  setPoint = (_PID_MATH) 10.0; //para teste sem protocolo
  currentPosition = pos_getCurrentPosition();
  error = -setPoint - (currentPosition/5); // 1852 pulsos por volta / 360 graus ~= 5
  
  integralError += (kIntegral * (half_PID_PERIOD)) * (error + previousError); // termo integral discreto
  integralErrorScaled = integralError >> 10; //divindo por 1024 para desfazer a multiplica??o por 1000 aplicada em T
  //bloco abaixo utilizado para anti-windup
  if (integralErrorScaled > 1024) {
      integralErrorScaled = 1024;
  }
  
  else if (integralErrorScaled < -1024){
      integralErrorScaled = -1024; 
  }
  activation = (kProportional * error) + (kDerivative *1000 * ((- currentPosition + previousPosition) / PID_PERIOD )) + integralErrorScaled;
  excitation = pid_scaleExcitation(activation);
  di(); 
  mot_setExcitation(excitation);
  ei(); 
  
//  activation = kProportional * error;
//  printReal(convertToReal(error));
//  putch('\n');

  previousError = error;
  previousPosition = currentPosition;
#ifdef DEBUG
  RA4 = 0;
#endif
} // pid_pid

void pid_setPoint(_PID_MATH position) {
   setPoint = position;
} // pid_setPoint

void pid_setDerivativeGain(_PID_MATH gain) {
  kDerivative = gain;
} // pid_setDerivativeGain

void pid_setIntegralGain(_PID_MATH gain) {
  kIntegral = gain;
} // pid_setIntegralGain

void pid_setProportionalGain(_PID_MATH gain) {
  kProportional = gain;
} // pid_setProportionalGain

void pid_clearError(void) {
  integralError = 0.0f;
  previousError = 0.0f;
} // pid_clearError

void pid_init(void) {
   integralError = 0.0f;
   previousError = 0.0f;
   currentPosition = 0.0f;
   kProportional = 20.0f;
   kDerivative = 0.0f;
   kIntegral = 5.0f;
} // pid_init

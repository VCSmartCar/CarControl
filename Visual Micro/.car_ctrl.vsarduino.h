//Board = Arduino Mega 2560 or Mega ADK
#define ARDUINO 103
#define __AVR_ATmega2560__
#define F_CPU 16000000L
#define __AVR__
#define __cplusplus
#define __attribute__(x)
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__
#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define prog_void
#define PGM_VOID_P int
#define NOINLINE __attribute__((noinline))

typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {}

//already defined in arduno.h
//already defined in arduno.h
void normalCtrl();
void cameraCtrl(int cameraAngle);
void getAccer_Gyro();
float accerDeleteZero(int16_t avalue,char axis);
float gyroDeleteZero(int16_t gvalue,char axis);
void balanceFilter(float angle_m,float gyro_m);
void AngleControl(float fangle);
void pwmOut(float angleOutput,float turnOutput);
void phoneRemote(void);
bool isStood();
void selfBalance();

#include "E:\College\各科作业\大学课件&作业\大四1\毕业设计\Arduino学习\Arduino\arduino-1.0.3\hardware\arduino\variants\mega\pins_arduino.h" 
#include "E:\College\各科作业\大学课件&作业\大四1\毕业设计\Arduino学习\Arduino\arduino-1.0.3\hardware\arduino\cores\arduino\arduino.h"
#include "E:\Project\Arduino\car_ctrl\car_center_ctrl.ino"
#include "E:\Project\Arduino\car_ctrl\motor_ctrl.h"

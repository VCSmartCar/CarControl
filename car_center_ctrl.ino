// include the library code:
#include <LiquidCrystal.h>
#include "motor_ctrl.h"
#include <Servo.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"
#define cameraAngleCtrlPin  10

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

int16_t a;
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(25, 23, 28, 26, 24, 22);
Servo myservo;  // create servo object to control a servo 
MPU6050 accelgyro;

unsigned long lastTime;
float interval;

/***************获取陀螺仪和加速度计*******************/

float accer_d[3];
float gyro_d[3];

float accer_Angle;
float gyro_Angle;

#define G_sens 131.0
#define A_sens 16384.0
#define Gxyz_zero 20.0
#define Axy_zero 0.05
#define Az_zero 0.0

/*********************互补滤波参数********************/

static long preTime;
static float f_angle;

/******************平衡控制**********************/

float output;
float lastErr;

/******************手机控制管脚**********************/

char data[10];
float forwardSpeed=0.0,turnSpeed=0.0;
int num=0;

/*********************************************/
// the setup routine runs only once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);  
  for(int i = 2; i<=9; i++)//控制电机的管脚设置为输出
	{
		pinMode(i, OUTPUT);
	}

  Serial2.begin(9600);
  Serial2.setTimeout(100);
 
   // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, GDUFS");
  myservo.attach(cameraAngleCtrlPin);
  myservo.write(90);

  Wire.begin();
  Serial.begin(38400);
  Serial.setTimeout(10);
  accelgyro.initialize();

  stop();
  lcd.setCursor(0, 0);
    // print the number of seconds since reset:
  lcd.print("starting...3s");
  delay(3000);//对待路由器启动
}

int i=0;
boolean isFirstTimeEnterSelfBalance = true;
boolean isFirstTimeEnterNormalCtrl = true;
// the loop routine runs over and over again forever:
void loop() {
	
		if(isStood())
	{
		if(isFirstTimeEnterSelfBalance)
		{
			stop();
			isFirstTimeEnterSelfBalance = false;
			isFirstTimeEnterNormalCtrl = true;
		}
		selfBalance();
	}
	else
	{
		if(isFirstTimeEnterNormalCtrl)
		{
			stop();
			isFirstTimeEnterNormalCtrl = false;
			isFirstTimeEnterSelfBalance = true;
		}
		normalCtrl();
	}
    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, 1);
    // print the number of seconds since reset:
    lcd.print(millis()/1000);
}

void normalCtrl()
{
	if (Serial2.available()) 
		{
				char ctrMessage[11] = "123\0";
				char Speed[7] = "123\0";
				char Angle[4] = "";
				Serial2.readBytes(ctrMessage,9);
				if(strcmp(ctrMessage,"camera789")==0)
				{
					Serial2.readBytes(Angle,3);
					int cameraAngle = atoi(Angle);
					lcd.clear();
					lcd.setCursor(4,1);
					lcd.print(cameraAngle);
					myservo.write(cameraAngle);
				}else
				{
					Serial2.readBytes(Speed,6);

					if(strcmp(ctrMessage,"forward89")==0)
					{
						forward(Speed);
					}
					if(strcmp(ctrMessage,"backward9")==0)
					{
						backward(Speed);
					}
					if(strcmp(ctrMessage,"turnleft9")==0)
					{
						turnLeft(Speed);
					}
					if(strcmp(ctrMessage,"turnright")==0)
					{
						turnRight(Speed);
					}
					if(strcmp(ctrMessage,"stop56789")==0)
					{
						stop();
					}
					lcd.clear();
					lcd.setCursor(0,0);
					lcd.print(strcat(ctrMessage,Speed));
					lcd.setCursor(11,1);
					lcd.print(++i);
				}			
		}
}

void cameraCtrl(int cameraAngle)
{
	analogWrite(cameraAngleCtrlPin, (int)(cameraAngle/180.0 * 255));//注意要转换成浮点型
	lcd.setCursor(11,0);
	lcd.print("OK");
}


void getAccer_Gyro()
{
	int16_t accer[3], gyro[3];
	unsigned long recentTime=millis();

	accelgyro.getMotion6(&accer[0], &accer[1], &accer[2], &gyro[0], &gyro[1], &gyro[2]);
	interval=(recentTime-lastTime)/1000.0;

	accer_d[1]=accerDeleteZero(accer[1],'y');
	accer_d[2]=accerDeleteZero(accer[2],'z');
	gyro_d[0]=gyroDeleteZero(gyro[0],'x');

	accer_Angle=atan2(accer_d[2],accer_d[1])*180/3.1415926;
	gyro_Angle-=gyro_d[0]*interval;

	lastTime=recentTime;

	/*Serial.print(accer_d[1]);
	Serial.print(",");
	Serial.print(accer_d[2]);
	Serial.print(",");
	Serial.println(gyro_d[0]);*/

	/*Serial.print(accer_Angle);
	Serial.print(",");
	Serial.println(-gyro_d[0]);*/
}

float accerDeleteZero(int16_t avalue,char axis)   //去加速度计零值
{
	float value=avalue/A_sens;
	if(axis=='x')
		value=value-0.09;
	else if(axis=='y')
		value=value+0.01;
	return value;
}

float gyroDeleteZero(int16_t gvalue,char axis)     //去陀螺仪零值
{
	float value=gvalue/G_sens;
	if(axis=='x')
		value=value-42.85;
	return value;
}

void balanceFilter(float angle_m,float gyro_m)
{
	unsigned long now = millis();                           // 当前时间(ms)
    float dt = (now - preTime) / 1000.0;                    // 微分时间(ms)
    preTime = now;  
    float K = 1.5;
    float A = K / (K + dt);                    
    f_angle = A * (f_angle + gyro_m * dt) + (1-A) * angle_m;  // 互补滤波算法
}

void AngleControl(float fangle)
{
	float err,dErr;
	float kp=55,kd=0.4;
	float setPoint=1.6;
	setPoint+=forwardSpeed;
	err=fangle-setPoint;
	dErr=(err-lastErr)/interval;

	output=kp*err-kd*gyro_d[0];

	lastErr=err;

	/*Serial.print(err);
	Serial.print(",");
	Serial.print(fangle);
	Serial.print(",");
	Serial.println(output);*/

	/*Serial.print(forwardSpeed);
	Serial.print(",");
	Serial.println(turnSpeed);*/

}

void pwmOut(float angleOutput,float turnOutput)
{
	float LOutput,ROutput;

	LOutput=angleOutput+turnOutput;//左电机
	ROutput=angleOutput-turnOutput;//右电机

	if(LOutput>0)//左电机向前
	{
		analogWrite(left_two_fw,constrain(abs(LOutput)+20,0,255));
		digitalWrite(left_two_bw,LOW);
	}
	else if(LOutput<0)//左电机向后
	{
		analogWrite(left_two_bw,constrain(abs(LOutput)+40,0,255));
		digitalWrite(left_two_fw,LOW);
	}
	else  //刹车
	{
		digitalWrite(left_two_fw, LOW);
		digitalWrite(left_two_bw, LOW);
	}
	if(ROutput>0)//右电机向前
	{
		analogWrite(right_two_fw,constrain(abs(ROutput)+20,0,255));
		digitalWrite(right_two_bw,LOW);
	}
	else if(ROutput<0)//右电机向后
	{
		analogWrite(right_two_bw,constrain(abs(ROutput)+40,0,255));
		digitalWrite(right_two_fw,LOW);
	}
	else//刹车
	{
		digitalWrite(right_two_fw, LOW);
		digitalWrite(right_two_bw, LOW);
	}
}

void phoneRemote(void)	 
{
	char i[7];

	if(Serial2.available())
	{
		Serial2.readBytes(data,9);
		Serial2.readBytes(i,6);
		if (strcmp(data,"stop56789")==0)
		{
			forwardSpeed=0; //停止
			turnSpeed=0;  
		}
		else if (strcmp(data,"forward89")==0)
		{
			forwardSpeed=2.0; //前进
		}  
		else if (strcmp(data,"backward9")==0)
		{
			forwardSpeed=-2.0; //后退
		}
		else if (strcmp(data,"turnleft9")==0)
		{
			turnSpeed=70; //左
		} 
		else if (strcmp(data,"turnright")==0)
		{
			turnSpeed=-70; //右
		} 
		else 
		{
			forwardSpeed=0; //停止
			turnSpeed=0;
		}
		lcd.setCursor(5,1);
		lcd.print(data);
	}
}

bool isStood()
{
    getAccer_Gyro();
    balanceFilter(accer_Angle,-gyro_d[0]);
    if(-15<f_angle&&f_angle<15)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void selfBalance()
{
    phoneRemote();
    AngleControl(f_angle);
    pwmOut(output,turnSpeed);
}

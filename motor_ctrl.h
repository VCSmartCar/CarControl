#include <arduino.h>
int left_one_fw = 2;//use PWM to ctrl motors
int left_one_bw = 3;
int right_one_fw = 4;
int right_one_bw = 5;
int left_two_fw = 6;
int left_two_bw = 7;
int right_two_fw = 8;
int right_two_bw = 9;

int speedLevel(char *level)
{
	if(strcmp(level,"level1")==0)
	{
		return 28*3;
	}
	if(strcmp(level,"level2")==0)
	{
		return 28*4;
	}
	if(strcmp(level,"level3")==0)
	{
		return (int)28*4.5;
	}
	if(strcmp(level,"level4")==0)
	{
		return 28*5;
	}
	if(strcmp(level,"level5")==0)
	{
		return (int)28*5.5;
	}
	if(strcmp(level,"level6")==0)
	{
		return 28*6;
	}
	if(strcmp(level,"level7")==0)
	{
		return (int)28*6.5;
	}
	if(strcmp(level,"level8")==0)
	{
		return 28*8;
	}
	if(strcmp(level,"level9")==0)
	{
		return 255;
	}
}

void forward(char *level)
{
	int speed = speedLevel(level);
	float percent = 0.915;
	analogWrite(left_one_fw, int (speed * percent));
	digitalWrite(left_one_bw, LOW);
	analogWrite(right_one_fw, speed);
	digitalWrite(right_one_bw, LOW);
	analogWrite(left_two_fw, int (speed * percent));
	digitalWrite(left_two_bw, LOW);
	analogWrite(right_two_fw, speed);
	digitalWrite(right_two_bw, LOW);
}

void backward(char *level)
{
	int speed = speedLevel(level);

	digitalWrite(left_one_fw, LOW);
	analogWrite(left_one_bw, speed);
	digitalWrite(right_one_fw, LOW);
	analogWrite(right_one_bw, speed);
	digitalWrite(left_two_fw, LOW);
	analogWrite(left_two_bw, speed);
	digitalWrite(right_two_fw, LOW);
	analogWrite(right_two_bw, speed);
}

void turnRight(char *level)
{
	int speed  = speedLevel(level);

	digitalWrite(left_one_fw, LOW);
	analogWrite(left_one_bw, speed);
	analogWrite(right_one_fw, speed);
	digitalWrite(right_one_bw, LOW);
	digitalWrite(left_two_fw, LOW);
	analogWrite(left_two_bw, speed);
	analogWrite(right_two_fw, speed);
	digitalWrite(right_two_bw, LOW);
}

void turnLeft(char *level)
{
	int speed = speedLevel(level);

	analogWrite(left_one_fw, speed);
	digitalWrite(left_one_bw, LOW);
	digitalWrite(right_one_fw, LOW);
	analogWrite(right_one_bw, speed);
	analogWrite(left_two_fw, speed);
	digitalWrite(left_two_bw, LOW);
	digitalWrite(right_two_fw, LOW);
	analogWrite(right_two_bw, speed);	
}

void stop()
{
	digitalWrite(left_one_fw, LOW);
	digitalWrite(left_one_bw, LOW);
	digitalWrite(right_one_fw, LOW);
	digitalWrite(right_one_bw, LOW);
	digitalWrite(left_two_fw, LOW);
	digitalWrite(left_two_bw, LOW);
	digitalWrite(right_two_fw, LOW);
	digitalWrite(right_two_bw, LOW);
}
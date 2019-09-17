/*
 * PIDCONtrol.c
 *
 * Created: 2019-07-04 오후 2:20:34
 *  Author: kw898
 */ 
                        
#include <stdio.h>
#include <math.h>
#include "PIDControl.h"
#include "Blarenxing_robot.h"

/*센서 Data 평균 사용시 
float ReadSensor()
{	
	float acc_f[3], gyro_f[3];
	char i;
	static float sum_accx,sum_accy,sum_accz,sum_gyrox,sum_gyroy,sum_gyroz;
	static float accx,accy,accz,gyrox,gyroy,gyroz,angle;
	for(i=0;i<=Sensor_Average; i++)
	{
		MPU6050I2CReadIMU_f(acc_f, gyro_f);
		sum_accx+=acc_f[0]; // x 축에 대한 가속도
		sum_accy+=acc_f[1]; // x 축에 대한 가속도
		sum_accz+=acc_f[2]; // z 축 가속도
		sum_gyrox+=gyro_f[0]; // x 축에 대한 각속도 roll
		sum_gyroy+=gyro_f[1]; // y 축에 대한 각속도 pitch
		sum_gyroz+=gyro_f[2]; // y 축에 대한 각속도 pitch
	}
	accx=sum_accx/Sensor_Average;
	accy=sum_accy/Sensor_Average;
	accz=sum_accz/Sensor_Average;
	gyrox=sum_gyrox/Sensor_Average;
	gyroy=sum_gyroy/Sensor_Average;
	gyroz=sum_gyroz/Sensor_Average;
	sum_accx=0;
	sum_accy=0;
	sum_accz=0;
	sum_gyrox=0;
	sum_gyroy=0;
	sum_gyroz=0;
	i=0;
	angle=1;//Readangle(accx,accy,accz,gyrox,gyroy,gyroz);
	return angle;
}
float Readangle(float accx,float accy,float accz,float gyrox,float gyroy,float gyroz)
{
	float accx_angle=0,gyrox_angle=0,accy_angle=0,gyroy_angle=0,accz_angle=0,gyroz_angle=0;
	float SANGBO_angle=0;
	
	accy_angle=atan(-accx/sqrt(pow(accy,2)+pow(accz,2)))*e_PI; // Pitch 가속도 각도
	gyroy_angle+=gyroy;
	
	SANGBO_angle=alpha*(SANGBO_angle+(gyroy_angle*intergral_time/1000.0))+((1-alpha)*accy_angle);
	
	return accy_angle;
}
*/

int Readangle()
{
	int16_t acc[3],gyro[3];
	MPU6050I2CReadIMU(acc,gyro);
	float  accy_angle=0, gyroy_angle=0;
	static float  SANGBO_angle;
	
	accy_angle=atan2(acc[0],acc[2])*e_PI;///sqrt(pow(acc[1],2)+pow(acc[2],2)))*e_PI;
	//accy_angle=atan(acc[0]/sqrt(pow(acc[1],2)+pow(acc[2],2)))*e_PI;
	gyroy_angle=((gyro[1])/131.);
	
	SANGBO_angle=(alpha*(SANGBO_angle+(gyroy_angle/1000000)))+((1-alpha)*accy_angle);
	
	return SANGBO_angle;
}

int PIDControl(int first_angle, float angle)
{
	static int Sum_error=0,Old_Angle_Error=0,New_Angle_Error=0;
	static int PID_Val,P_out=0,I_out=0,D_out=0;
	
	Old_Angle_Error=first_angle-angle; // 목표 각도-현재각도 차
	Sum_error+=(New_Angle_Error-Old_Angle_Error); // 차이값 누적(적분을 위하여) 
	New_Angle_Error=Old_Angle_Error; // 현재 차이값 = 과거 차이값
	P_out=Kp*Old_Angle_Error;          // 차이 값의 게인값 곱 = P제어
	I_out+=Ki*Sum_error*dT;  // 게인 곱 I제어 편차값이 일정 수치 이상 되면 I제어 실행
	D_out=Kd*(Old_Angle_Error-New_Angle_Error)/dT;  // 현재 차이 - 과거 차이 적분*게인 D제어
	PID_Val=P_out+I_out+D_out; // PID 제어 값 출력
	Old_Angle_Error=0;
	
	return PID_Val;
}
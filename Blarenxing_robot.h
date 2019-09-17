/*
 * Blarenxing_robot.h
 *
 * Created: 2019-06-23 오후 8:27:33
 *  Author: kw898
 */ 

#ifndef BLARENXING_ROBOT_H_
#define BLARENXING_ROBOT_H_

#define Sevor_Motor_MAX 2300
#define Sevor_Motor_MIX 700
#define Sevor_Motor_MID (Sevor_Motor_MAX+Sevor_Motor_MIX)/2

#define ON_Signal  1
#define OFF_Signal 0
#define Voltage_ON 200
#define Voltage_Off 0


void MOTor_SevO_Micowave_set_up(unsigned int DC_Sevo_Motor_time);
void Ultrasonic_control();
void Ultrasonic_comparison_Moter(unsigned char Initialization, unsigned char limit, int PID_Val);

#endif /* BLARENXING_ROBOT_H_ */
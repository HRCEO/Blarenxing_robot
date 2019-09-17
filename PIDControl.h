/*
 * PIDControl.h
 *
 * Created: 2019-07-04 오후 2:21:16
 *  Author: kw898
 */ 


#ifndef PIDCONTROL_H_
#define PIDCONTROL_H_

#define dT 0.000020
#define Sensor_Average 0
#define Kp 100
#define Ki 1
#define Kd 1
#define e_PI 180/M_PI
#define alpha 0.98

int Readangle();
int PIDControl(int first_angle, float angle);

#endif /* PIDCONTROL_H_ */
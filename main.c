/*
 * Blarenxing_robot.c
 *
 * Created: 2019-06-23 오후 7:59:34
 * Author : kw898
*/
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "Blarenxing_robot.h"
#include "PIDControl.h"
#include "mpu6050_i2c.h"
#include "oled.h"
#include "USART.h"

#define DC_Sevo_Motor_Pulse_Period 2000 // *0.5us 10ms
#define Initialization 15
#define limit 30

void setting_Init()
{
	UART_Init();
	oled_init();
	MPU6050I2CInit(400);//400kHz
	MOTor_SevO_Micowave_set_up(DC_Sevo_Motor_Pulse_Period);
}
int main(void)
{
	int PID_Val=0; 
	setting_Init();
	Start_Draw();
	int first_angle=Readangle();
	while(1)
	{
		float angle=Readangle();
		PID_Val=PIDControl(first_angle,angle);
		//Ultrasonic_control();
		Ultrasonic_comparison_Moter(Initialization,limit,PID_Val);
		
 		char anglee[200],anglef[200];
 		sprintf(anglee,"angle=%.0f first_angle=%d PID_Val=%d\n",angle,first_angle,PID_Val);
 		UART1_Puts(anglee);
		
// 		sprintf(anglef,"%f,%f,%f\n",(float)angle,(float)first_angle,(float)PID_Val);
// 		UART0_Puts(anglef);
	}
}
/*
 // 모터 / 초음파 / 서보모터 / OLED / 블루투스 / 자이로 / 스위치
 // 모터 헤더파일 / 서	보모터 헤더 파일 / 초음파 입력캡처 (3번)
 // 제어 목표 순서
 // 1. 초음파,모터,서보 모터 구동
 // 2. 초음파 센서 전방 감지 물체 있을시 정지후 없어지면 재 가동
 // 3. 자이로 센서 값 (3개 배열) 이용 PID 제어를 이용 제자리 서기 (초기 시작시 설정값 5번 받아서 평균값 저장후 계속 유지)
 // 4. 서보모터 좌우 회전 기능 추가후 초음파 센서 3개 받은후 가장 먼곳으로 이동 (3개다 설정값 이하일 경우 설정값의 3배 후진)
 // 5. 블루투스 이용 전후좌우 이동
 // 6. 블루투스 이용 센서 설정값 설정
 // 7. 자율주행모드 / 조종 모드 (조종모드에서 초음파는 전방만 감지-> 검출시 정지후 핸드폰에 알림)

 // x축 가속도, y자이로 값 (acc 0 : y 회전, 1 : X 회전 2: z)
 // x축 가속도, y자이로 값 (gyro 2 : +y 1 :+x 0 : z)

07/01
1. 센서 값 평균 산출 OK -> 0 출력
2. 자이로 변화에 따른 값 미 도출 문제 발생 -> ??? 각도 계산 이전 수정 실시 !

7/03
1. 실수의 자리수가 8bit를 넘어서 표현 이 불가. ! -> 가 아니었음.
- > 자리수 조정 필요 (2자리 까지) 어떻게 ? -> .2 좌측 정렬 2자리 까지 표현 완료,
2. 값을 읽지 못함. 원인이 무었일까...?
3. 초기 각도가 먹통 ... 이상하게 -4~0~4 은 나옴... 각으로 사용가능 할정도... OK! 값이 그 위치에 따라 변화감 (절대값)
4. 초기 각도 인식  ! (목표값으로 인식) 유후

1. 각도 값과 PID 제어를 공부 하는 도중 자이로 센서의 rang라는 함수를 제외 하여도 값이 잘 나오는것을 알게됨
   분명 필요 이유가 있을겄이다..

2. 현재 각 부품 레지스트 셋팅 완료
3. 전 후 쓰러짐 양에 따른 모터 속도 제어 완료 (고속회전후 원위치시 모터 정지 현상 발생)
4. OLED 셋팅 ok 자이로 값 표현 준비 !
5. 각도를 정확하게 표현하고 있는것인가 ?? 현재는 0.4~0.2 등 소수점으로 표현됨
6. PID 제어가 들어 갔는가 ?

7. 모터 정역 OK , 속도 조절 OK -> 서잇지 못함 (조금의 가능성도 보이지 않는다.)
    -> 각도 즉 상보필터부터 다시 한번 CHECK

센서값 TEST

! 중요 이론 : 가속도 , 각속도(자이로)

1. gyro[0] -> 좌우 회전시 반응 함 -> x 자이로(각속도) 값 반응 roll
가속도 3방향 조금씩 이나 전부 반응함

gyro[1] -> 전후 회전시 반응함 -> y 자이로(각속도) 값 반응 pich
가속도 3방향 조금씩 전부 반응

gyro[2] -> 360도 회전 방향 반응 -> z 자이로(각속도) 값 반응 yaw
가속도 3방향 반응 무 !


* 상당한 노이즈 발생, 및 진동에 엄청 많은 값을 출력함 !

acc[0] -> 3방향의 가속도를 모두 출력하나 X방향의 가속도가 가장 크게 나타난것으로 보아 X 가속도
acc[1] -> y!
acc[2] -> z 가속도 !!


! 초기화 목표는 각도 구하는것과 PID제어 그리고 상보필터를 제대로 이해하고 넘어가자 !
  프로젝트 실패는 상관없다. !!

  1. 자이로 센서의 각 각의 배열에 따른 값 정확히 파악하기 
  2. 각도즉 상보필터 공식 다시 한번 이해하고 넘어가기 
  3. 각도 구하기
  4. 각도에 따른 PID제어 재 검토
     -> ID가 없는 P 제어  , PI제어 ,PD제어 ,PID 제어 TEST 하기
  5. PID제어 별 GAIN값 수정
  6. 다시 TEST
  
  
  * 주요 느낌점 ! 
   상보필터는 말그대로 필터 ! 
   즉 각속도를 이용한 각도 와 가속도를 이용한 각도를 상보 필터를 거처 1개의 각도 를 얻는것.
  
  * 현제 가속도 값 구하기 완료 , 자이로(각속도 각도 구하기 진행중)
  -> 중간 프로그램 정리
  -> 회전각 을 구하기 위한 Millis 함수 제작 필요  
  -> 각속도= 회전한 각도  / 시간
  -> 회전한각도 = 각속도 * 시간

*/
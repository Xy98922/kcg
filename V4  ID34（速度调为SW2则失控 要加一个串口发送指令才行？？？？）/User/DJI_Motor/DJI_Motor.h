#ifndef __DJI_MOTOR_H
#define	__DJI_MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"




/*CAN���ͻ��ǽ��յ�ID*/
typedef enum
{

	CAN_TxPY12V_ID 	= 0x200,		//��̨12V����ID
	CAN_TxPY24V_ID	= 0x1FF,		//��̨12V����ID
//	CAN_Pitch_ID 	= 0x201,			//��̨Pitch
//	CAN_Yaw_ID   	= 0x203,			//��̨Yaw
	CAN_YAW_FEEDBACK_ID   = 0x205,		//��̨Yaw24v
	CAN_PIT_FEEDBACK_ID  = 0x206,			//��̨Yaw24v
	CAN_POKE_FEEDBACK_ID  = 0x207,
	CAN_ZGYRO_RST_ID 			= 0x404,
	CAN_ZGYRO_FEEDBACK_MSG_ID = 0x401,
	CAN_MotorLF_ID 	= 0x041,    //��ǰ
	CAN_MotorRF_ID 	= 0x042,		//��ǰ
	CAN_MotorLB_ID 	= 0x043,    //���
	CAN_MotorRB_ID 	= 0x044,		//�Һ�

	CAN_EC60_four_ID	= 0x200,	//EC60���ճ���
	CAN_backLeft_EC60_ID = 0x203, //ec60
	CAN_frontLeft_EC60_ID = 0x201, //ec60
	CAN_backRight_EC60_ID = 0x202, //ec60
	CAN_frontRight_EC60_ID = 0x204, //ec60
	
	//add by langgo
	CAN_3510Moto_ALL_ID = 0x200,
	CAN_3510Moto1_ID = 0x201,
	CAN_3510Moto2_ID = 0x202,
	CAN_3510Moto3_ID = 0x203,
	CAN_3510Moto4_ID = 0x204,
	CAN_DriverPower_ID = 0x80,

	CAN_HeartBeat_ID = 0x156,
	
}CAN_Message_ID;

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
	
    POSITION_PID,
    DELTA_PID,
};

#define FILTER_BUF_LEN		5
/*���յ�����̨����Ĳ����ṹ��*/
typedef struct{
		uint16_t 	angle;				//abs angle range:[0,8191] ���ת�Ǿ���ֵ
		uint16_t 	last_angle;	  //abs angle range:[0,8191]
	
		int16_t	 	speed_rpm;       //ת��
		int16_t  	real_current;    //ת��
	
		int16_t  	given_current;   //ʵ�ʵ�ת�ص���
		uint8_t  	Temp;           //�¶�

		uint16_t	offset_angle;   //�������ʱ�����ƫ�Ƕ�
		int32_t		round_cnt;     //���ת��Ȧ��
		int32_t		total_angle;    //���ת�����ܽǶ�
	
		uint8_t		buf_idx;
		uint16_t	angle_buf[FILTER_BUF_LEN];
		uint16_t	fited_angle;
		uint32_t	msg_cnt;
}moto_measure_t;


typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[3];				//Ŀ��ֵ,����NOW�� LAST�� LLAST���ϴ�
    float get[3];				//����ֵ
    float err[3];				//���
	    
    float pout;							//p���
    float iout;							//i���
    float dout;							//d���
    
    float pos_out;						//����λ��ʽ���
    float last_pos_out;				//�ϴ����
    float delta_u;						//��������ֵ
    float delta_out;					//��������ʽ��� = last_delta_out + delta_u
    float last_delta_out;
    
	  float max_err;
	  float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//����޷�
    uint32_t IntegralLimit;		//�����޷�
    
//    void (*f_param_init)(struct __pid_t *pid,  //PID������ʼ��
//                    uint32_t pid_mode,
//                    uint32_t maxOutput,
//                    uint32_t integralLimit,
//                    float p,
//                    float i,
//                    float d);
//    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid���������޸�

}pid_t;

typedef struct __command_t //������λ����������Ľṹ������
{
	unsigned char cmd; //������
	
	int tag_rpm1;
	int tag_rpm2;
    int rev_rpm1;
	int rev_rpm2;
	unsigned char rpm_available; //ָʾת�������Ƿ����
	
	float tag_speed_x;
//	float tag_speed_y;
//	float tag_speed_z;
	float tag_speed_w;
	unsigned char speed_available;
	
	unsigned char flag;  //��־λ 0x01��ʾ���յ�һ֡���� ������
	
	unsigned char loraRev_flag;
}command_t;

void DJI_Motor_Init(void);
void DJI_Motor_Control(void);	
//void Mecanum_Wheel_Rpm_Model(int16_t v1,int16_t v2,int16_t v3,int16_t v4); 
//void DiffX4_Wheel_Speed_Model(float speed_x,float speed_w);
//void Mecanum_Wheel_Speed_Model(int16_t speed_x,int16_t speed_y,int16_t speed_w);
void DiffX2_Wheel_Speed_Model(float speed_x,float speed_w);
void DiffX2_Wheel_Rpm_Model(float v1,float v2);


void DJI_Motor_Show_Message(void);
void DJI_Motor_Upload_Message(void);
void DJI_Motor_WriteData_In_Buff(uint8_t *DataBuff,uint16_t datalength);
void DJI_Motor_Clear_Odom(void);


void CAN_DJI_C620_DataSend( int16_t iq1, int16_t iq2);
void CAN_RxCpltCallback(CanRxMsg* RxMessage);
void get_moto_measure(moto_measure_t *ptr,CanRxMsg* RxMessage);
void get_moto_offset(moto_measure_t *ptr,CanRxMsg* RxMessage);
void get_total_angle(moto_measure_t *p);
void EXTIX_Init(void);
void KEY_Init(void);
void PID_struct_init(pid_t* pid,uint32_t mode,uint32_t maxout,
    uint32_t intergral_limit, float 	kp, float 	ki, float 	kd);
float pid_calc(pid_t* pid, float get, float set);
float pid_sp_calc(pid_t* pid, float get, float set, float gyro);
void servoinit(int16_t angle1);
void Watch_PID(void);
		
void abs_limit(float *a, float ABS_MAX); //��Χ����
static void pid_reset(pid_t	*pid, float kp, float ki, float kd); //����PID����


//extern pid_t pid_rol;
//extern pid_t pid_pit;
//extern pid_t pid_yaw;
//extern pid_t pid_pit_omg;
//extern pid_t pid_yaw_omg;	
extern pid_t pid_spd[2];
//extern pid_t pid_yaw_alfa;
//extern pid_t pid_chassis_angle;
//extern pid_t pid_poke;
//extern pid_t pid_poke_omg;
//extern pid_t pid_imu_tmp;		//imu_temperature
//extern pid_t pid_cali_bby;	//big buff yaw
//extern pid_t pid_cali_bbp;
//extern pid_t pid_omg;
//extern pid_t pid_pos;

#ifdef __cplusplus
}
#endif

#endif

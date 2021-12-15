#include "td_2d.h"
#include "math.h"

#define ABS(x)		((x>0)? (x): (-x)) 

Fhan_Data RCx;
Fhan_Data RCw;
Fhan_Data Encode[2];

//���ź���
int16_t Sign_TD(float Input)
{
    int16_t output=0;
    if(Input>1E-6) output=1;
    else if(Input<-1E-6) output=-1;
    else output=0;
    return output;
}

//�ֶκ�����x>d 
int16_t Fsg_TD(float x,float d)
{
    int16_t output=0;
    output=(Sign_TD(x+d)-Sign_TD(x-d))/2;
    return output;
}

//����΢������ʼ�� r=1000000,h=0.005,N0=5
void ADRC_Init(Fhan_Data *fhan_Input)
{
	  fhan_Input->r=200;
    fhan_Input->h=0.014;
    fhan_Input->N0=5;
}

void ADRC1_Init(Fhan_Data *fhan_Input)
{
	  fhan_Input->r=1000;
    fhan_Input->h=0.001;
    fhan_Input->N0=5;
}

//ADRC���ٸ���΢����TD���Ľ����㷨fhan
void Fhan_TD(Fhan_Data *fhan_Input,float expect_rcDat) //����ADRC���ȹ���
{
    float d=0,a0=0,y=0,a1=0,a2=0,a=0;
    float x1_delta=0;//RC״̬���������
    x1_delta=fhan_Input->x1	-expect_rcDat;//��x1-v(k)���x1�õ���ɢ���¹�ʽ
    fhan_Input->h0=fhan_Input->N0*fhan_Input->h;//��h0���h��������ٸ���΢�����ٶȳ�������
    d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
    a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
    y=x1_delta+a0;//y=x1+a0
    a1=sqrt(d*(d+8*ABS(y)));//a1=sqrt(d*(d+8*ABS(y))])
    a2=a0+Sign_TD(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
    a=(a0+y)*Fsg_TD(y,d)+a2*(1-Fsg_TD(y,d));
    fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_TD(a,d)
    -fhan_Input->r*Sign_TD(a)*(1-Fsg_TD(a,d));//�õ�����΢�ּ��ٶȸ�����
    fhan_Input->x1+=fhan_Input->h*fhan_Input->x2;//�������ٸ���״̬��x1��ң������Ҫ���ź�
    fhan_Input->x2+=fhan_Input->h*fhan_Input->fh;//�������ٸ���״̬��΢��x2
}
//ADRC���ٸ���΢����TD���Ľ����㷨fhan

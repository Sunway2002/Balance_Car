#include "DMP.h"
//�����˲���
// a = tau / (tau + dt)  
// acc = ���ٶȴ��������� 
// gyro = ���������� 
// dt = ��������         
float angle;
float a;

short x_nAcc,y_nAcc,z_nAcc;//���ٶ�x�ᡢy�ᡢz������
short x_nGyro,y_nGyro,z_nGyro;//������x�ᡢy�ᡢz������
float x_fAcc,y_fAcc,z_fAcc;

float g_fAccAngle;//���ٶȴ���������atan2()����õ��ĽǶ�
float g_fGyroAngleSpeed;//�����ǽ��ٶ�
float g_fCarAngle;//С�����
float dt = 0.005;//�����˲�����������

unsigned char g_ucMainEventCount;//���¼��������������ж���
#include "math.h"
#include "MPU.h"

float ComplementaryFilter(float acc, float gyro, float dt) 
{
    a = 0.98;  
    angle = a * (angle + gyro * dt) + (1 - a) * (acc);  
    return angle;  
}
void GetMpuData(void)//��ȡMPU-6050���ݺ���
{
    MPU_Get_Accelerometer(&x_nAcc,&y_nAcc,&z_nAcc);//��ȡMPU-6050���ٶ�����
    MPU_Get_Gyroscope(&x_nGyro,&y_nGyro,&z_nGyro); //��ȡMPU-6050����������
}
void AngleCalculate(void)//�Ƕȼ��㺯��
{
    //-------���ٶ����ݴ���--------------------------
    //����Ϊ��2gʱ�������ȣ�16384 LSB/g
    x_fAcc = x_nAcc / 16384.0;
    y_fAcc = y_nAcc / 16384.0;
    z_fAcc = z_nAcc / 16384.0;

    g_fAccAngle = atan2(y_fAcc,z_fAcc) / 3.14 * 180.0;

    //-------���������ݴ���-------------------------
    //��ΧΪ2000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
    g_fGyroAngleSpeed = x_nGyro / 16.4;  //������ٶ�ֵ                   

    //-------�����˲�---------------
    g_fCarAngle = ComplementaryFilter(g_fAccAngle, g_fGyroAngleSpeed, dt);
}

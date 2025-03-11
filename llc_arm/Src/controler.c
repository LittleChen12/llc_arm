#include "controler.h"

//�Զ������
//DH��
float dh_matrix[6][4] = {
		{0.0f,     0.0f,   0.212f,    0.0f},       // Joint 1
		{-PI/2.0f, 0.04f,  0.062f,    -PI/2.0f},   // Joint 2
		{0.0f,     0.18f,  -0.05f,    0.0f},       // Joint 3
		{-PI/2.0f, 0.055f, 0.205027f, 0.0f},       // Joint 4
		{PI/2.0f,  0.0f,   0.0f,      PI/2.0f},    // Joint 5
		{-PI/2.0f, 0.0f,   0.0f,   		0.0f}        // Joint 6
};

//�仯����
arm_matrix_instance_f32 T_trans;
float t_trans[16];

//�����۳˵���ʱ����									 
arm_matrix_instance_f32	 T_temp;
float t_temp[16] = {0};

//�����������
arm_matrix_instance_f32 T_result;
float t_result[16] = {1.0f, 0.0f, 0.0f, 0.0f,
												0.0f, 1.0f, 0.0f, 0.0f,
												0.0f, 0.0f, 1.0f, 0.0f,
												0.0f, 0.0f, 0.0f, 1.0f};

//ĩ�������6���λ�˾���
arm_matrix_instance_f32 T_xyz_6;	
float t_xyz_6[16] = {1.0f, 0.0f, 0.0f, 0.0f,
									 0.0f, 1.0f, 0.0f, 0.0f,
									 0.0f, 0.0f, 1.0f, 0.0865f,
									 0.0f, 0.0f, 0.0f, 1.0f};											

//ĩ�������0���λ�þ���
arm_matrix_instance_f32 T_xyz_0;	
float t_xyz_0[16] = {1.0f, 0.0f, 0.0f, 0.0f,
									 0.0f, 1.0f, 0.0f, 0.0f,
									 0.0f, 0.0f, 1.0f, 0.0f,
									 0.0f, 0.0f, 0.0f, 1.0f};			

//����Ƕ�
float motor_theta[6]={0.513709f,0.514567f,0.515424f,0.516280f,0.517137f,0.517992f};//������ٱ�50,50,50,4,4,30


void matrix_init(void)
{
	arm_mat_init_f32(&T_trans, 4, 4, t_trans);   // ��ʼ���仯����	
	arm_mat_init_f32(&T_result, 4, 4, t_result); // ��ʼ���������
	arm_mat_init_f32(&T_xyz_6, 4, 4, t_xyz_6);   // ��ʼ��ĩ����̬����
	arm_mat_init_f32(&T_xyz_0, 4, 4, t_xyz_0);   // ��ʼ��ĩ����̬����
	arm_mat_init_f32(&T_temp, 4, 4, t_temp);     //��ʼ����ʱ�������
}

//��������
void dh_test(void)
{
	matrix_init();
	forward_kinematics(dh_matrix,6);
}

// ����Ľ��� DH ������Ӧ�ı任����
// ���� modified_dh_matrix ��ÿһ�д�ţ���, a, d,��(����)
void compute_modified_dh_matrix(float modified_dh_matrix[6][4], int i) {
    float alpha = modified_dh_matrix[i][0];
    float a     = modified_dh_matrix[i][1];
    float d     = modified_dh_matrix[i][2];
    float theta = modified_dh_matrix[i][3] + motor_theta[i];

    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);
    float cos_alpha = arm_cos_f32(alpha);
    float sin_alpha = arm_sin_f32(alpha);

    // ���ڸĽ��� DH ��������α任����
    // ע�⣺���� sin_alpha*d ǰ������ţ�cos_alpha*d ǰ���޷���
    float t_data[16] = {
        // ��һ��
        cos_theta,            -sin_theta,                0.0f,              a,
        // �ڶ���
        sin_theta * cos_alpha, cos_theta * cos_alpha,    -sin_alpha,        -sin_alpha * d,
        // ������
        sin_theta * sin_alpha, cos_theta * sin_alpha,     cos_alpha,         cos_alpha * d,
        // ������
        0.0f,                 0.0f,                     0.0f,              1.0f
    };
		memcpy(t_trans,t_data,sizeof(t_data));
}

// �����˶�ѧ
void forward_kinematics(float matrix[6][4], int num_joints) {
    for (int i = 0; i < num_joints; i++) {
        compute_modified_dh_matrix(matrix, i);//����仯����
        arm_mat_mult_f32(&T_result, &T_trans, &T_temp);//������˱仯����
			  // ���˷�������ƻ� t_result ������ T_result
        memcpy(t_result, t_temp, sizeof(t_temp));
    }
		arm_mat_mult_f32(&T_result,&T_xyz_6,&T_xyz_0);
}





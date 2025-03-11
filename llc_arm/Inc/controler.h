#ifndef __CONTROLER_H
#define __CONTROLER_H

//ͷ�ļ�����
#include "arm_math.h"
#include "math.h"
#include "string.h"
#include "stdio.h"


//�Զ��庯��
// д����������Ƕȵ��Ľ��� DH ������
void Write_DH(float (*modified_dh_matrix)[6][4], float* theta);

// ����Ľ��� DH ������Ӧ�ı任����
// ���� modified_dh_matrix ��ÿһ�д�ţ� ��, d, a, ��
void compute_modified_dh_matrix(float modified_dh_matrix[6][4], int i);

// �����˶�ѧ
void forward_kinematics(float dh_matrix[6][4], int num_joints);

void dh_test(void);

#endif

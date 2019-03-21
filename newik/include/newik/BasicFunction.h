#ifndef BASICFUNCTION_H
#define BASICFUNCTION_H

#include <math.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <limits>


#define Pi                  3.14159265358979323846
#define Inf					0x7fffffff
#define Thresholds			0.000001

#define RArmla1				0.04920
#define	RArmla2				0.22452
#define RArmla3				0.21923
#define LArmla1			   -0.04920
#define	LArmla2				0.22452
#define LArmla3				0.21923

#define Rq1_lower_limit	   -2.26 + Pi / 2
#define Rq1_upper_limit		0.7854 + Pi / 2
#define Rq2_lower_limit	   -1.3090 + Pi / 2	+ Deg2Rad(20.0)		//avoid singularity
#define Rq2_upper_limit		0 + Pi / 2 + Deg2Rad(20.0)
#define Rq3_lower_limit    -Pi / 4 + Pi / 2
#define Rq3_upper_limit		Pi / 2 + Pi / 2
#define Rq4_lower_limit	   -1.5700								//physical restriction
#define Rq4_upper_limit	   -0.0175								//avoid singularity
#define Rq5_lower_limit    -Pi / 2
#define Rq5_upper_limit     Pi / 2
#define Rq6_lower_limit	   -0.6981 - Pi / 2
#define Rq6_upper_limit	    0.6981 - Pi / 2
#define Rq7_lower_limit	   -0.5235
#define Rq7_upper_limit	    0.5235

#define Lq1_lower_limit	   -2.26 + Pi / 2
#define Lq1_upper_limit		0.7854 + Pi / 2
#define Lq2_lower_limit	    0 + Pi / 2 - Deg2Rad(20.0)			//avoid singularity
#define Lq2_upper_limit		1.3090 + Pi / 2 - Deg2Rad(20.0)	
#define Lq3_lower_limit    -Pi / 2 + Pi / 2
#define Lq3_upper_limit	    Pi / 4 + Pi / 2
#define Lq4_lower_limit	   -1.5700								//physical restriction
#define Lq4_upper_limit	   -0.0175								//avoid singularity
#define Lq5_lower_limit    -Pi / 2
#define Lq5_upper_limit     Pi / 2
#define Lq6_lower_limit	   -0.6981 - Pi / 2
#define Lq6_upper_limit	    0.6981 - Pi / 2
#define Lq7_lower_limit	   -0.5235
#define Lq7_upper_limit	    0.5235


struct Matrix
{
	double *data;
	int *size;
	int allocatedSize;
	int numDimensions;
	bool canFreeData;
};


//��ʼ������
void InitMatrix(Matrix **pEmxArray, int numDimensions);

//�ı�����?��
void EnsureMatrix(Matrix *emxArray, int oldNumel);

//�ͷž����ڴ�
void FreeMatrix(Matrix **pEmxArray);

//�Ƕ�תΪ����
double Deg2Rad(double Deg);

//�������?    C[m][n] = A[m][p] * B[p][n]
void MultMatrix(int m, int n, int p, double *A, double *B, double *C);

//�������?	C= cross(A,B), A,B,C:1*3, ע�⣺C����Ԫ�ر�����3
void CrossVector(double* A, double* B, double* C);

//�������?	C= A*B, A,B,C:1*n
void DotVector(int n, double* A, double* B, double& C);

//������λ��	B = unit(A), A,B:1*n 
void NormVector(int n, double* A, double* B);

//����ת��	B = A', A:m*n	B:n*m 
void TransMatrix(int m, int n, double* A, double* B);

//����α任������� B = (A)^-1
void InverseTransMatrix(double *A, double *B);

//һά�����С��������?
void Sort(double *A, int size);

//һά�����ֵ�˲�?
void Smooth(double* A, int span, int n, double* B);

//һά������
void Diff(double *x, int n, double *B);

//�����Ǻ���������
bool CheckOverFlow(double* value);

//��Ԫ���任Ϊ��ת����4X4
void Quat2T(double quat[4], double *T);

//��ת����4X4�任Ϊ��Ԫ��
void T2Quat(double *T, double quat[4]);

//A(4x4), rotx -> B(4x4)
void RotX(double rad, double *A, double *B);

//A(4x4), roty -> B(4x4)
void RotY(double rad, double *A, double *B);

//A(4x4), rotz -> B(4x4)
void RotZ(double rad, double *A, double *B);

#endif
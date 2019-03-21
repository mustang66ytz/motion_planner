#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include "XR1ArmKinematics.h"
#include "XR1ArmMotionPlanning.h"
#include "XR1ArmAngle.h"

int main()
{
	int i, j;
	int num;
	double rtn;
	//num of way points
	int cnt1 = 40;
	double data1[40][7] = {0.0};
	//this is fix, shoulder_x in back_y
	double tempLTransT[16]={0, -0.939692620785908,  0.342020143325669,  0.24286, 
							0, -0.342020143325669, -0.939692620785908, -0.15779,
							1,  0,					0,					0,
							0,	0,					0,					1};
	//this is fix, shoulder_x in back_y
	double tempRTransT[16]={0, -0.939692620785908, -0.342020143325669,  0.24286, 
							0,  0.342020143325669, -0.939692620785908,  0.15779,
							1,  0,					0,					0,
							0,	0,					0,					1};
	double LTransT[16]={0.0};
	double RTransT[16]={0.0};
	//this is fix, back_y in shoulder_x
	InverseTransMatrix(tempLTransT, LTransT);
	//this is fix, back_y in shoulder_x
	InverseTransMatrix(tempRTransT, RTransT);
	//InitTheta is not fix, it is one of the input
	double LInitTheta[7]={0.12059172666541951,  0.032427442409019125, 0.21557455513227075,
						 -0.16147293197242885, -0.16147293197242885, 0.045181704745496765,
						 -0.0014133836888979743};
	double RInitTheta[7]={0.0};

	Matrix *LT;
	Matrix *RT;
	Matrix *LTheta;
	Matrix *RTheta;

	XR1ArmAngle ArmAngle;
	XR1ArmKinematics XR1ArmK;
	XR1ArmMotionPlanning XR1ArmMP;

	//matrix struct necessary beginning process
	InitMatrix(&LT, 3);
	num = LT->size[0] * LT->size[1] * LT->size[2];
	LT->size[0] = 4;
	LT->size[1] = 4;
	LT->size[2] = cnt1;
	EnsureMatrix(LT, num);

	InitMatrix(&RT, 3);
	num = RT->size[0] * RT->size[1] * RT->size[2];
	RT->size[0] = 4;
	RT->size[1] = 4;
	RT->size[2] = cnt1;
	EnsureMatrix(RT, num);

	//collect data from file
	FILE *fp;
	fp = fopen("angle.txt", "r");
	for (i = 0; i < cnt1; i++)
	{
		for (j = 0; j < 7; j++)
		{
			fscanf(fp, "%lf", &data1[i][j]);
		}
	}
	fclose(fp);

	//copy data to matrix struct
	for(i=0; i<cnt1; i++)
	{
		//orientation trans to rotm
		Quat2T(&data1[i][0], &LT->data[i*16]);
		//position trans to rotm
		LT->data[i*16+3] = data1[i][4];
		LT->data[i*16+7] = data1[i][5];
		LT->data[i*16+11] = data1[i][6];
		//this is fix
		LT->data[i*16+15] = 1.0;
	}

	//MotionPlanning with LArm_IK(Arbitrary way points)
	rtn = XR1ArmMP.LArmArbitraryMP(LInitTheta, LTransT, LT, LTheta);
	




}




#ifndef XR1ARMANGLE_H
#define XR1ARMANGLE_H

#include "BasicFunction.h"


class XR1ArmAngle
{
public:
	
	double OptimizeRange(double la1, double la2, double la3, double InitThete[7], Matrix *T70, Matrix *OptmzRange);
	
	bool CalculateMultiAngleRange(double la1, double la2, double la3, double *T70, Matrix *RArmAngleRange);

private:

	void GetQ1LimitPos(double la1, double la2, double la3, double Parm[4], double q1, double Q1LimitPos[9]);

	void GetQ2LimitPos(double la1, double la2, double la3, double Parm[4], double q2, double Q2LimitPos[9]);

	void GetQ6LimitPos(double la1, double la2, double la3, double Parm[4], double T70[16], double q6, double Q6LimitPos[9]);

	void GetQ7LimitPos(double la1, double la2, double la3, double Parm[4], double T70[16], double q7, double Q7LimitPos[9]);

	bool CheckAngleRange1(double la1, double la2, double la3, double LimitPhi1[9], double Pc[3], double R, double u_uint[3], double v_uint[3], int m, Matrix *PhiRange1);

	bool CheckAngleRange2(double la1, double la2, double la3, double LimitPhi2[9], double T70[16], double Pc[3], double R, double u_uint[3], double v_uint[3], int m, Matrix *PhiRange2);

	void Pos2Phi(double LimitPos[9], double u_uint[3], double n_uint[3], double Pc[3], double LimitPhi[2]);

	void ChooseRange(Matrix *Range, double *Compare, double OptmzRange[2]);

};

#endif
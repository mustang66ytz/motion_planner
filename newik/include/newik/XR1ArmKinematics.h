#ifndef XR1ARMKINEMATICS_H
#define XR1ARMKINEMATICS_H


class XR1ArmKinematics
{
public:

	bool RArmFKSolver(double* Q, double*T70);

	bool LArmFKSolver(double* Q, double*T70);

	bool RArmIKSolver(double Arm_angle, double* T70, double* Q);

	bool LArmIKSolver(double Arm_angle, double* T70, double* Q);

	bool CalculateArmAngle(double la1, double la2, double la3, double *Q, double *ArmAngle);

	bool CalculateQ1(double la1, double la2, double la3, double q2, double *cir_fun, double *q1);


private:

	bool IKInitialize(double la1, double la2, double la3, double Arm_angle, double *T70, double *Pw, double *cir_fun);

	bool FKSolver(double la1, double la2, double la3, double *Q, double *T70);

	bool IKSolver(double la1, double la2, double la3, double Arm_angle, double *T70, double* Q);
	
	bool CalculateQ3(double la1, double la2, double la3, double q1, double q2, double q4, double Arm_angle, double *Pw, double *q3);

	bool CalculateQ4(double la1, double la2, double la3, double q1, double q2, double *Pw, double *cir_fun, double *q4);

	bool CalculateQ6(double la1, double la2, double la3, double q1, double q2, double q3, double q4, double *T70, double *q5, double *T74);


};

#endif
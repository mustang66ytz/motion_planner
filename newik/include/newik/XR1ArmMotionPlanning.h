#ifndef XR1ARMMOTIONPLANNING_H
#define XR1ARMMOTIONPLANNING_H

#include <vector>
#include "BasicFunction.h"

using namespace std;


class XR1ArmMotionPlanning
{
public:

	double RArmArbitraryMP(double *InitTheta, double *TransT, Matrix *T70, Matrix *Theta);

	double LArmArbitraryMP(double *InitTheta, double *TransT, Matrix *T70, Matrix *Theta);

	double RArmArbitraryMP(double *InitTheta, double *TransT, Matrix *T70, Matrix *Theta, vector<int> &index);

	double LArmArbitraryMP(double *InitTheta, double *TransT, Matrix *T70, Matrix *Theta, vector<int> &index);

	double RArmLinearMP(double *InitTheta, double *TransT, double *EndT, int t, Matrix *Theta);

	double LArmLinearMP(double *InitTheta, double *TransT, double *EndT, int t, Matrix *Theta);

	double RArmLinearMP(double *InitTheta, double *TransT, double *EndT, int t, Matrix *Theta, vector<int> &index);

	double LArmLinearMP(double *InitTheta, double *TransT, double *EndT, int t, Matrix *Theta, vector<int> &index);

	bool RArmTargetPoint(double *InitTheta, double *TransT, double *EndT, double *TargetTheta, double &error);

	bool LArmTargetPoint(double *InitTheta, double *TransT, double *EndT, double *TargetTheta, double &error);

	
private:

	void Ctraj(double T0[16], double T1[16], int t, Matrix *T);

	bool PlanArmAngele(double InitArmAngle, Matrix *OptmzRange, double *ArmAngle);


};

#endif
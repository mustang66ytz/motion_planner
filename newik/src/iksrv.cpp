#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <stdlib.h>
#include <stddef.h>
#include "newik/XR1ArmKinematics.h"
#include "newik/XR1ArmMotionPlanning.h"
#include "newik/XR1ArmAngle.h"
#include "newik/trajectoryIK.h"
#include "newik/trajectoryIKRequest.h"
#include "newik/trajectoryIKResponse.h"


bool ik(newik::trajectoryIK::Request &req, newik::trajectoryIK::Response &res)
{
std::cout << "11" << std::endl;
    //definition
    int i, j;
    int num;
    int mode;
    int arm_select;
    int matrix_size;
    vector<int> index;
    double error = 0;   
    double rtn = 0;
    double InitTheta[7] = {0.0};
    double *inputWayPoint;
    Matrix *T;
    Matrix *Theta;
    //this is fix, shoulder_x in back_y
	double tempLTransT[16]={0, -0.939692620785908,  0.342020143325669,  0.2581, 
							0, -0.342020143325669, -0.939692620785908, -0.1634,
							1,  0,					0,				   -0.0170,
							0,	0,					0,					1};
	//this is fix, shoulder_x in back_y
	double tempRTransT[16]={0, -0.939692620785908, -0.342020143325669,  0.2581, 
							0,  0.342020143325669, -0.939692620785908,  0.1634,
							1,  0,					0,				   -0.0170,
							0,	0,					0,					1};
    double LTransT[16]={0.0};
    double RTransT[16]={0.0};
    InverseTransMatrix(tempLTransT, LTransT);
    InverseTransMatrix(tempRTransT, RTransT);

    XR1ArmMotionPlanning XR1ArmMP;


    // input print
    std::cout << "--------------------------------------" << std::endl;
    std::cout << "input parm: " << std::endl;
    num = req.num_waypoint;
    std::cout << "num is:" << num << std::endl;

    arm_select = req.arm_select;
    std::cout << "arm_select is:" << arm_select << std::endl;

    mode = req.mode_select;
    std::cout << "mode is:" << mode << std::endl;

    std::cout << "initial angle is: ";
    for (i=0; i<req.init_theta.size(); i++) {
        InitTheta[i] = req.init_theta[i];
        std::cout << InitTheta[i] << " ";
    }
    std::cout << std::endl;

    inputWayPoint = (double *)malloc(req.waypointPos.size()*sizeof(double));
    std::cout << "waypoints are: " << endl;
    for(i=0; i<req.waypointPos.size(); i++)
    {
        inputWayPoint[i] = req.waypointPos.at(i);
        std::cout << inputWayPoint[i] << " ";
    }
    std::cout << endl;
    std::cout << "--------------------------------------" << std::endl;

    //input T
    InitMatrix(&T, 3);
    matrix_size = T->size[0] * T->size[1] * T->size[2];
    T->size[0] = 4;
    T->size[1] = 4;
    T->size[2] = req.waypointPos.size()/7;
    EnsureMatrix(T, matrix_size);
    for (i=0; i<req.waypointPos.size()/7; i++)
    {
        //orientation trans to rotm
        Quat2T(&inputWayPoint[i*7], &T->data[i*16]);
        //position trans to rotm
        T->data[i*16+3] = inputWayPoint[i*7+4];
        T->data[i*16+7] = inputWayPoint[i*7+5];
        T->data[i*16+11] = inputWayPoint[i*7+6];
        //this is fix
        T->data[i*16+15] = 1.0;
    }
    for(i=0; i<16*req.waypointPos.size()/7; i++)
    {
        cout << T->data[0] <<" ";
    }
    cout << endl;

    //output theta
    InitMatrix(&Theta, 2);
    matrix_size = Theta->size[0] * Theta->size[1];
    Theta->size[0] = num;
    Theta->size[1] = 7;
    EnsureMatrix(Theta, matrix_size);

    // arm selection
    if(0 == arm_select)
    {
        // mode selection
        if (1 == mode)
        {
            rtn = XR1ArmMP.LArmArbitraryMP(InitTheta, LTransT, T, Theta, index);
            if (rtn==1)
            {
                for (i=0; i<7*num-7*index.size(); i++)
                {
                    res.jointAngles.push_back(Theta->data[i]);
                }
            }
        }
        else if(2 == mode)
        {
            rtn = XR1ArmMP.LArmLinearMP(InitTheta, LTransT, T->data, num, Theta, index);
            if (rtn==1)
            {
                for (i=0; i<7*num-7*index.size(); i++)
                {
                    res.jointAngles.push_back(Theta->data[i]);
                }
            }
        }
        else if(3 == mode)
        {
            rtn = XR1ArmMP.LArmTargetPoint(InitTheta, LTransT, T->data, Theta->data, error);
            if (rtn==1)
            {
                for (i=0; i<7; i++)
                {
                    res.jointAngles.push_back(Theta->data[i]);
                }
            }
        }
        else
        {
            std::cout << "error mode!!!" << endl;
        }
    }
    else if(1 == arm_select)
    {
        if (1 == mode)
        {
            rtn = XR1ArmMP.RArmArbitraryMP(InitTheta, RTransT, T, Theta, index);
            if (rtn==1)
            {
                for (i=0; i<7*num-7*index.size(); i++)
                {
                    res.jointAngles.push_back(Theta->data[i]);
                }
            }
        }
        else if(2 == mode)
        {
            rtn = XR1ArmMP.RArmLinearMP(InitTheta, RTransT, T->data, num, Theta, index);
            if (rtn==1)
            {
                for (i=0; i<7*num-7*index.size(); i++)
                {
                    res.jointAngles.push_back(Theta->data[i]);
                }
            }
        }
        else if(3 == mode)
        {
            rtn = XR1ArmMP.RArmTargetPoint(InitTheta, RTransT, T->data, Theta->data, error);
            if (rtn==1)
            {
                for (i=0; i<7; i++)
                {
                    res.jointAngles.push_back(Theta->data[i]);
                }
            }
        }
        else
        {
            std::cout << "error mode!!!" << endl;
        }
    }

    // output print
    std::cout << "--------------------------------------" << std::endl;
    std::cout << "output parm: " << std::endl;
    std::cout << "error is: " << error << std::endl;
    res.success = rtn;
    std::cout << "Sucess rate is: " << res.success << std::endl;

    std::cout << "The failed range is: " << std::endl;
    for(i=0; i<index.size(); i++)
    {
        res.failNum.push_back(index.at(i));
        std::cout << res.failNum.at(i) << " ";
    }
    std::cout << std::endl;

    std::cout << "joint angle are: ";
    for (i=0; i<res.jointAngles.size(); i++){
        std::cout << res.jointAngles.at(i) << " ";
    }
    std::cout << std::endl;

    res.error = error;
    std::cout << "Error is: " << res.error << std::endl;
    std::cout << "--------------------------------------" << std::endl;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_ik_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("provide_ik_service", ik);
    ROS_INFO("Ready to provide ik service: ");
    ros::spin();

    return 0;
}

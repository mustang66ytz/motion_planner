#include <cstdlib>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_dmp/DMPAction.h>
#include <action_dmp/DMPResult.h>
#include <dmp/SetActiveDMP.h>
#include <dmp/LearnDMPFromDemo.h>
#include <dmp/DMPPoint.h>
#include <dmp/DMPTraj.h>
#include <dmp/DMPData.h>
#include <dmp/GetDMPPlanResponse.h>
#include <dmp/GetDMPPlan.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <std_msgs/Float64MultiArray.h>
// for rviz visualization
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define GetCurrentDir getcwd

//std::vector<dmp::DMPData> makeLFDRequest(ros::NodeHandle &nh, int dims, std::vector<std::vector<double>> traj, int dt, int K, int D, int num_bases){
dmp::LearnDMPFromDemo::Response makeLFDRequest(ros::NodeHandle &nh, int dims, std::vector<std::vector<double>> traj, double dt, int K, int D, int num_bases){
    //create message
    dmp::DMPTraj demotraj;

    for (int i=0; i<traj.size(); i++){
        dmp::DMPPoint pt;
        pt.positions = traj[i];
        demotraj.points.push_back(pt);
        demotraj.times.push_back(dt*i);
    }

    std::vector<double> k_gains(dims, K);
    std::vector<double> d_gains(dims, D);

    std::cout<<"Starting Learn from Demo"<<std::endl;
    // instantiate a service client
    ros::ServiceClient learn_dmp_client = nh.serviceClient<dmp::LearnDMPFromDemo>("learn_dmp_from_demo");
    // define the service request
    dmp::LearnDMPFromDemo srv1;
    srv1.request.demo = demotraj;
    srv1.request.k_gains = k_gains;
    srv1.request.d_gains = d_gains;
    srv1.request.num_bases = num_bases;
    // call the service
    std::vector<dmp::DMPData> res_dmp_list;
    if(learn_dmp_client.call(srv1)){
        res_dmp_list = srv1.response.dmp_list;
    }
    return srv1.response;

}

void makeSetActiveRequest(ros::NodeHandle &nh, std::vector<dmp::DMPData> res_dmp_list){
    // instantiate a service client
    ros::ServiceClient set_active_dmp_client = nh.serviceClient<dmp::SetActiveDMP>("set_active_dmp");
    // define the service request
    dmp::SetActiveDMP srv2;
    srv2.request.dmp_list = res_dmp_list;
    // call the service
    bool res = false;

    if(set_active_dmp_client.call(srv2)){
        res = srv2.response.success;
        if(res) std::cout<<"set the current dmp as active successful!"<<std::endl;
        else if(!res) std::cout<<"set the current dmp as active failed!"<<std::endl;
    }
    std::cout<<res<<std::endl;
}

dmp::GetDMPPlan::Response makePlanRequest(ros::NodeHandle &nh, std::vector<double> x_0, std::vector<double> x_dot_0, int t_0, std::vector<double> goal, std::vector<double>goal_thresh, int seg_length, double tau, double dt, int integrate_iter){
    std::cout<<"starting dmp planning!"<<std::endl;
    // instantiate a service client
    ros::ServiceClient make_dmp_plan = nh.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
    // define the service request
    dmp::GetDMPPlan srv3;
    srv3.request.x_0 = x_0;
    srv3.request.x_dot_0 = x_dot_0;
    srv3.request.t_0 = t_0;
    srv3.request.goal = goal;
    srv3.request.goal_thresh = goal_thresh;
    srv3.request.seg_length = seg_length;
    srv3.request.tau = tau;
    srv3.request.dt = dt;
    srv3.request.integrate_iter = integrate_iter;
    // call the service
    if(make_dmp_plan.call(srv3)){
        std::cout<<"DMP planning done!"<<std::endl;
    }
    return srv3.response;

}

void visualizer(std::vector<double> &start, std::vector<double> &target){
    // render the initial pose and the target pose in rviz
    static tf2_ros::StaticTransformBroadcaster static_broadcaster_initial_pose;
    static tf2_ros::StaticTransformBroadcaster static_broadcaster_target_pose;
    
    geometry_msgs::TransformStamped static_transform_initial_pose;
    geometry_msgs::TransformStamped static_transform_target_pose;

    static_transform_initial_pose.header.stamp = ros::Time::now();
    static_transform_initial_pose.header.frame_id = "/Back_Y";
    static_transform_initial_pose.child_frame_id = "DMP_initial";
    static_transform_initial_pose.transform.translation.x = start[4];
    static_transform_initial_pose.transform.translation.y = start[5];
    static_transform_initial_pose.transform.translation.z = start[6];
    static_transform_initial_pose.transform.rotation.x = start[0];
    static_transform_initial_pose.transform.rotation.y = start[1];
    static_transform_initial_pose.transform.rotation.z = start[2];
    static_transform_initial_pose.transform.rotation.w = start[3];
    static_broadcaster_initial_pose.sendTransform(static_transform_initial_pose);

    std::cout<<"broadcasted the initial pose"<<std::endl;

    static_transform_target_pose.header.stamp = ros::Time::now();
    static_transform_target_pose.header.frame_id = "/Back_Y";
    static_transform_target_pose.child_frame_id = "DMP_target";
    static_transform_target_pose.transform.translation.x = target[4];
    static_transform_target_pose.transform.translation.y = target[5];
    static_transform_target_pose.transform.translation.z = target[6];
    static_transform_target_pose.transform.rotation.x = target[0];
    static_transform_target_pose.transform.rotation.y = target[1];
    static_transform_target_pose.transform.rotation.z = target[2];
    static_transform_target_pose.transform.rotation.w = target[3];
    static_broadcaster_target_pose.sendTransform(static_transform_target_pose);

}

std::vector<dmp::DMPPoint> dmp_ee(ros::NodeHandle &nh, std::string &filename, std::vector<double> &start, std::vector<double> &target){
    int dims = 7;
    double dt = 1.0;
    int K = 50;
    int D = 2.0*sqrt(K);
    int num_bases = 10;
    std::vector<std::vector<double>> inputStates;
    
    //reading trainning data from file
    std::string line;
    //get current directory
    char the_path[256];
    getcwd(the_path, 255);

    std::ifstream f(filename);
    bool file_opened = f.is_open();
    if(!f.is_open()) std::cout<<"error while opening file"<<std::endl;
    while(std::getline(f, line)){
        std::stringstream ss;
        // remove the brackets of the line
        line.erase(0, 1);
        line.erase(line.size()-1 );
        // insert the line into the stringstream
        ss<<line;
        //std::cout<<ss.str()<<std::endl;
        double curr;
        std::vector<double> curr_line;
        while(ss>>curr){
            curr_line.push_back(curr);
            if(ss.peek() == ','){
                ss.ignore();
            }
        } 
        inputStates.push_back(curr_line);
    }
    f.close();
    std::cout<<"finish reading demo data"<<std::endl;

    // learn from the dmp demo data
    dmp::LearnDMPFromDemo::Response resp1 = makeLFDRequest(nh, dims, inputStates, dt, K, D, num_bases);
    // set it as the active dmp
    makeSetActiveRequest(nh, resp1.dmp_list);
    // generate a plan
    std::vector<double> x_dot_0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int t_0 = 0;
    std::vector<double> goal_thresh = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    int seg_length = -1;
    double tau = resp1.tau;
    dt = 1.0;
    int integrate_iter = 5;
    // render the dmp start and goal poses
    visualizer(start, target);

    dmp::GetDMPPlan::Response resp2 = makePlanRequest(nh, start, x_dot_0, t_0, target, goal_thresh, seg_length, tau, dt, integrate_iter);
    std::vector<dmp::DMPPoint> plan_generated = resp2.plan.points;
    return plan_generated;
}

class DMPAction{

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<action_dmp::DMPAction> as; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name;
    // create messages that are used to publish feedback/result
    action_dmp::DMPFeedback feedback;
    action_dmp::DMPResult result;

public:

    DMPAction(std::string name):
        as(nh, name, boost::bind(&DMPAction::executeCB, this, _1), false),
        action_name(name)
    {
        as.start();
    }

    ~DMPAction(void)
    {
    }

    void executeCB(const action_dmp::DMPGoalConstPtr &goal){
        // dmp for ginger starts here
        std::string file_name = goal->mp_file;
        std::vector<double> start = goal->initial_pose;
        std::vector<double> target = goal->target_pose;
        std::vector<dmp::DMPPoint> plan_generated = dmp_ee(nh, file_name, start, target);
        // print the generated trajectory:
        /*
        for (int i=0; i<plan_generated.size(); i++){
            std::vector<double> curr_pt = plan_generated[i].positions;
            for (int j=0; j<curr_pt.size(); j++){
                std::cout<<curr_pt[j]<<" ";
            }
            std::cout<<" "<<std::endl;
        }*/
        // generate the action result
        std::vector<std::vector<double>> res;
        std::vector<double> res_1d;
        for (int i=0; i<plan_generated.size(); i++){
            for(int j=0; j<plan_generated[i].positions.size(); j++){
                res_1d.push_back(plan_generated[i].positions[j]);
            }
        }
        
        std_msgs::Float64MultiArray vec;
        vec.layout.dim.push_back(std_msgs::MultiArrayDimension());
        vec.layout.dim.push_back(std_msgs::MultiArrayDimension());
        vec.layout.dim[0].label = "row";
        vec.layout.dim[1].label = "col";
        vec.layout.dim[0].size = plan_generated.size();
        vec.layout.dim[1].size = plan_generated[0].positions.size();
        vec.data = res_1d;
        result.traj_dmp = vec;
        as.setSucceeded(result);
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "dmp_test_server");
    ROS_INFO("DMP action server is ready!");
    
    DMPAction dmp_ginger("dmp_action_server");
    ros::spin();

    return 0;
}
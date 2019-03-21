#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <action_dmp/DMPAction.h>
#include <action_dmp/DMPGoal.h>

std::vector<std::vector<double>> result_processor(action_dmp::DMPResult &raw_res){
    int row_num = raw_res.traj_dmp.layout.dim[0].size;
    int col_num = raw_res.traj_dmp.layout.dim[1].size;
    int count = 0;
    std::vector<std::vector<double>> res;
    for(int i=0; i<row_num; i++){
        std::vector<double> curr_pt;
        for(int j=0; j<col_num; j++){
            curr_pt.push_back(raw_res.traj_dmp.data[count]);
            count++;
        }
        res.push_back(curr_pt);
    }
    return res;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "dmp_test_client");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<action_dmp::DMPAction> ac("dmp_action_server", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // get the dmp folder name from user input
    std::string folder;
    std::cout<<"Enter the dmp learning primitive folder: "<<std::endl;
    std::cin>>folder;
    // send a goal to the action
    action_dmp::DMPGoal goal;
    std::string file_name = "./src/action_dmp/src/dmp_learning_primitives/"+folder+"/R_0";
    std::vector<double> start = {0.08742966306435822, 0.23510704690286763, -0.27011733036133695, 0.9295791428121432, -0.1, -0.2, 0.09};
    std::vector<double> target = {-0.5571638192069343, 0.575242166261965, -0.21679645766259464, 0.5582689768911218, 0.22445238346047316, -0.20288282629715598, 0.3291512751673038};
    goal.initial_pose = start;
    goal.target_pose = target;
    goal.mp_file = file_name;

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    action_dmp::DMPResult result;
    result = *ac.getResult();

    // reformat data conversion
    std::vector<std::vector<double>> res;
    res = result_processor(result);
    
    // print the result
    std::cout<<"The result from dmp action server is:"<<std::endl;
    for(int i=0; i<res.size(); i++){
        for(int j=0; j<res[i].size(); j++){
            std::cout<<res[i][j]<<" ";
        }
        std::cout<<" "<<std::endl;
    }
    //exit
    return 0;
}

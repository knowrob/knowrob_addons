#include "knowrob_kautham/knowrob_kautham.h"

#include <sensor_msgs/JointState.h>

typedef struct
{
    std::vector<float> conf;
    bool armType;
    bool response;
}tIKanswer;

void ham_product(double ax, double ay, double az, double aw, double bx, double by, double bz, double bw, double &rx, double &ry, double &rz, double &rw)
{
    rx = bw*ax + bx*aw - by*az + bz*ay;
    ry = bw*ay + bx*az + by*aw - bz*ax;
    rz = bw*az - bx*ay + by*ax + bz*aw;
    rw = bw*aw - bx*ax - by*ay - bz*az;
}

void get_grasp_pose_in_map(double otx, double oty, double otz, double orx, double ory, double orz, double orw, double gtx, double gty, double gtz, double grx, double gry, double grz, double grw,
                           double &rtx, double &rty, double &rtz, double &rrx, double &rry, double &rrz, double &rrw)
{
    double dummy;
    ham_product(orx, ory, orz, orw, grx, gry, grz, grw, rrx, rry, rrz, rrw);
    ham_product(orx, ory, orz, orw, gtx, gty, gtz, 0, rtx, rty, rtz, dummy);
    ham_product(rtx, rty, rtz, dummy, -orx, -ory, -orz, orw, rtx, rty, rtz, dummy);
    rtx += otx;
    rty += oty;
    rtz += otz;
}

void getCurrentRobotJointState(std::vector<float> &conf)
{
    boost::shared_ptr<const sensor_msgs::JointState> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    unsigned int maxK = msg->position.size();
    conf.clear();
    conf.resize(maxK);
    for(unsigned int k = 0; k < maxK; k++)
        conf[k] = (float)msg->position[k];
}

bool setQuery(std::vector <float> initControl,  std::vector <float> goalControl)
{
    kautham::SetQuery set_query_srv;

    set_query_srv.request.init.resize(initControl.size());
    set_query_srv.request.goal.resize(goalControl.size());

    // Set the initial and goal vectors (kautham controls)
    set_query_srv.request.init = initControl;
    set_query_srv.request.goal = goalControl;

    //call kautham service to set the query
    ros::service::call("/kautham_node/SetQuery", set_query_srv);

    // Evaluate the set_query service and perform rest of process
    if (set_query_srv.response.response == false)
    {
        std::cout << "SetQuery service has not been performed. " << std::endl;
        for (unsigned int i = 0; i < initControl.size(); ++i)
            std::cout<< initControl[i] <<" ";
        std::cout<<std::endl;
        for (unsigned int i = 0; i < goalControl.size(); ++i)
            std::cout<< goalControl[i] <<" ";
        std::cout<<std::endl;
        return false;
    }

    else
    {
        //        ROS_INFO( "SetQuery service has been performed. " );
        return true;
    }
}

std::vector <std::vector<float> > collisionFreePath(void)
{
    std::vector <std::vector<float> > path;
    path.clear();

    // activate the getpath service

    kautham::GetPath get_path_srv;
    ros::service::call("/kautham_node/GetPath", get_path_srv);

    if(get_path_srv.response.response.size()==0)
    {
        std::cout<<"Kautham returned No path from\n:";
        std::cout<<std::endl;
        ROS_WARN("There is no path for the corresponding action !");
        path.resize(0);
        return path;
    }

    // Return the size of path vector inside kautham
    // (ompl path returns a void for the last configuration, thus -1)
    else
    {
        int sizev = get_path_srv.response.response.size() - 1;

        std::cout<<"\n Kautham Query: (";

        std::cout<<")\n Kautham returned a path of "<<sizev<<" elements"<<std::endl;

        path.resize(sizev);

        for(int i=0; i<sizev; i++)
        {
            path[i].resize(get_path_srv.response.response[i].v.size());
            for(int j=0; j
                <get_path_srv.response.response[i].v.size(); j++)
            {
                path[i][j] = get_path_srv.response.response[i].v[j];
            }
        }
        return path;
    }

}

bool attachObjToRob(int64_t rob, int64_t tcp, int64_t index)
{
    kautham::AttachObstacle2RobotLink attach_object_srv;

    attach_object_srv.request.robot = rob;
    attach_object_srv.request.link = tcp;
    attach_object_srv.request.obs = index; //index of the obstacle

    ros::service::call("/kautham_node/AttachObstacle2RobotLink", attach_object_srv);
    //std::cout << "Attaching object  " << index<<std::endl;
    if (!attach_object_srv.response.response)
    {

        ROS_ERROR("The object has not been attached to the robot");
        return false;
    }

    else
    {
        //        std::cout << "The object number " << index << " has been attached to the robot." << std::endl;
        return true;
    }
}

bool detachObj(int64_t index)
{
    kautham::DetachObstacle detach_object_srv;

    detach_object_srv.request.obs = index;

    ros::service::call("/kautham_node/DetachObstacle", detach_object_srv);
    //std::cout << "Dettaching object  " << index<<std::endl;
    if (!detach_object_srv.response.response)
    {
        ROS_ERROR("The object has not been detached to the robot");
        return false;
    }

    else
    {
        //        std::cout << "The object number " << index << " has been detached to the robot" << std::endl;
        return true;
    }
}

bool pub_rob_motion(std::vector<std::vector<float> > path){
    std::vector<std::vector<float> > conf = path;

    if(conf.size()<1)
    {
        ROS_WARN("Path is empty, nothing to publish");
        return false;
    }
    else
    {
        //Sending configurations to RVIZ
        ros::NodeHandle nh;
        ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);   //moving publisher
        ros::Rate loop_rate(40);
        sensor_msgs::JointState joint_states;
        //defining joint names
        joint_states.name.resize(18);
        joint_states.name[0]="yumi_joint_1_r";
        joint_states.name[1]="yumi_joint_2_r";
        joint_states.name[2]="yumi_joint_7_r";
        joint_states.name[3]="yumi_joint_3_r";
        joint_states.name[4]="yumi_joint_4_r";
        joint_states.name[5]="yumi_joint_5_r";
        joint_states.name[6]="yumi_joint_6_r";

        joint_states.name[7]="yumi_joint_1_l";
        joint_states.name[8]="yumi_joint_2_l";
        joint_states.name[9]="yumi_joint_7_l";
        joint_states.name[10]="yumi_joint_3_l";
        joint_states.name[11]="yumi_joint_4_l";
        joint_states.name[12]="yumi_joint_5_l";
        joint_states.name[13]="yumi_joint_6_l";

        joint_states.name[14]="gripper_r_joint";
        joint_states.name[15]="gripper_r_joint_m";
        joint_states.name[16]="gripper_l_joint";
        joint_states.name[17]="gripper_l_joint_m";


        for (int i = 0; i < conf.size(); i++ )
        {

            joint_states.header.stamp = ros::Time::now();
            joint_states.position.resize(18);
            joint_states.position[0]=conf[i][0];
            joint_states.position[1]=conf[i][1];
            joint_states.position[2]=conf[i][2];
            joint_states.position[3]=conf[i][3];
            joint_states.position[4]=conf[i][4];
            joint_states.position[5]=conf[i][5];
            joint_states.position[6]=conf[i][6];

            joint_states.position[7]=conf[i][8];
            joint_states.position[8]=conf[i][9];
            joint_states.position[9]=conf[i][10];
            joint_states.position[10]=conf[i][11];
            joint_states.position[11]=conf[i][12];
            joint_states.position[12]=conf[i][13];
            joint_states.position[13]=conf[i][14];

            joint_states.position[14]=0.0;
            joint_states.position[15]=0.0;
            joint_states.position[16]=0.0;
            joint_states.position[17]=0.0;

            joint_pub.publish(joint_states); //publishing the configuration message to the robot
            loop_rate.sleep();
            //       ros::Duration(0.1).sleep();
        }
        return true;
    }
}

tIKanswer get_config_from_pose(double otx, double oty, double otz, double orx, double ory, double orz, double orw, double gtx, double gty, double gtz, double grx, double gry, double grz, double grw)
{
    double rtx;
    double rty;
    double rtz;
    double rrx;
    double rry;
    double rrz;
    double rrw;
    get_grasp_pose_in_map(otx, oty, otz, orx, ory, orz, orw, gtx, gty, gtz, grx, gry, grz, grw, rtx, rty, rtz, rrx, rry, rrz, rrw);

    kautham::FindIK Yumi_srv;

    Yumi_srv.request.maintSameWrist = true;
    Yumi_srv.request.armType = true;

    Yumi_srv.request.pos.resize(8);

    Yumi_srv.request.pos.at(0) = rtx;
    Yumi_srv.request.pos.at(1) = rty;
    Yumi_srv.request.pos.at(2) = rtz;

    Yumi_srv.request.pos.at(3) = rrx;
    Yumi_srv.request.pos.at(4) = rry;
    Yumi_srv.request.pos.at(5) = rrz;
    Yumi_srv.request.pos.at(6) = rrw;
    Yumi_srv.request.pos.at(7) = 0.0;

    ros::service::call("/kautham_node/FindIK", Yumi_srv);

    tIKanswer retq;
    retq.response = Yumi_srv.response.response;
    retq.conf = Yumi_srv.response.conf;
    retq.armType = true;
    
}

PREDICATE(kautham_init_planning_scene_internal, 2)
{
    std::string modelFolder((char*)PL_A1);
    std::string sceneMap((char*)PL_A2);
    kautham::OpenProblem kthopenproblem_srv;
    kthopenproblem_srv.request.problem = sceneMap;
    kthopenproblem_srv.request.dir.resize(1);
    kthopenproblem_srv.request.dir[0] = modelFolder;
    ros::service::call("kautham_node/OpenManipProblem", kthopenproblem_srv);
    if (kthopenproblem_srv.response.response == true)
    {
        ROS_INFO( "Kautham Problem opened correctly" );
    }
    else
    {
        ROS_ERROR( "ERROR Opening Kautham Problem" );
        ROS_ERROR( "models folder: %s", kthopenproblem_srv.request.dir[0].c_str() );
        ROS_ERROR( "scene map file: %s", kthopenproblem_srv.request.problem.c_str() );
    }
}

PREDICATE(kautham_grab_part_internal, 3)
{
    int objectIndex = (int)PL_A3;
    int rob = 0;
    int tcp = 6;
    PlTail objpose_list(PL_A1), grasppose_list(PL_A2);
    PlTerm value;
    // object pose in map
    objpose_list.next(value); double otx = value;
    objpose_list.next(value); double oty = value;
    objpose_list.next(value); double otz = value;
    objpose_list.next(value); double orx = value;
    objpose_list.next(value); double ory = value;
    objpose_list.next(value); double orz = value;
    objpose_list.next(value); double orw = value;
    // grasp transform in object frame
    grasppose_list.next(value); double gtx = value;
    grasppose_list.next(value); double gty = value;
    grasppose_list.next(value); double gtz = value;
    grasppose_list.next(value); double grx = value;
    grasppose_list.next(value); double gry = value;
    grasppose_list.next(value); double grz = value;
    grasppose_list.next(value); double grw = value;

    tIKanswer ikConf = get_config_from_pose(otx, oty, otz, orx, ory, orz, orw, gtx, gty, gtz, grx, gry, grz, grw);

    if(ikConf.response)
    {
        std::vector<float> init;
        getCurrentRobotJointState(init);
        if(!setQuery(init, ikConf.conf))
            return FALSE;
        std::vector<std::vector<float> > path = collisionFreePath();
        if(!path.size())
            return FALSE;
        pub_rob_motion(path);
        if(!attachObjToRob(rob, tcp, objectIndex))
            return FALSE;
        return TRUE;
    }

    return FALSE;
}

PREDICATE(kautham_put_part_internal, 3)
{
    int objectIndex = (int)PL_A3;
    PlTail objpose_list(PL_A1), grasppose_list(PL_A2);
    PlTerm value;
    // object pose in map
    objpose_list.next(value); double otx = value;
    objpose_list.next(value); double oty = value;
    objpose_list.next(value); double otz = value;
    objpose_list.next(value); double orx = value;
    objpose_list.next(value); double ory = value;
    objpose_list.next(value); double orz = value;
    objpose_list.next(value); double orw = value;
    // grasp transform in object frame
    grasppose_list.next(value); double gtx = value;
    grasppose_list.next(value); double gty = value;
    grasppose_list.next(value); double gtz = value;
    grasppose_list.next(value); double grx = value;
    grasppose_list.next(value); double gry = value;
    grasppose_list.next(value); double grz = value;
    grasppose_list.next(value); double grw = value;

    tIKanswer ikConf = get_config_from_pose(otx, oty, otz, orx, ory, orz, orw, gtx, gty, gtz, grx, gry, grz, grw);

    if(ikConf.response)
    {
        std::vector<float> init;
        getCurrentRobotJointState(init);
        if(!setQuery(init, ikConf.conf))
            return FALSE;
        std::vector<std::vector<float> > path = collisionFreePath();
        if(!path.size())
            return FALSE;
        pub_rob_motion(path);
        if(!detachObj(objectIndex))
            return FALSE;
        return TRUE;
    }

    return FALSE;
}

PREDICATE(kautham_blocking_objects, 3)
{
    PlTail colliders(PL_A3);
    PlTail objpose_list(PL_A1), grasppose_list(PL_A2);
    PlTerm value;
    // object pose in map
    objpose_list.next(value); double otx = value;
    objpose_list.next(value); double oty = value;
    objpose_list.next(value); double otz = value;
    objpose_list.next(value); double orx = value;
    objpose_list.next(value); double ory = value;
    objpose_list.next(value); double orz = value;
    objpose_list.next(value); double orw = value;
    // grasp transform in object frame
    grasppose_list.next(value); double gtx = value;
    grasppose_list.next(value); double gty = value;
    grasppose_list.next(value); double gtz = value;
    grasppose_list.next(value); double grx = value;
    grasppose_list.next(value); double gry = value;
    grasppose_list.next(value); double grz = value;
    grasppose_list.next(value); double grw = value;

    tIKanswer ikConf = get_config_from_pose(otx, oty, otz, orx, ory, orz, orw, gtx, gty, gtz, grx, gry, grz, grw);

    if (ikConf.response)
    {
        kautham::CheckCollision check_collision_obstacles_srv;
        check_collision_obstacles_srv.request.config = ikConf.conf;
        ros::service::call("/kautham_node/CheckCollisionRob", check_collision_obstacles_srv);

        if(!check_collision_obstacles_srv.response.response)
        {
            unsigned int maxK = check_collision_obstacles_srv.response.collObjs.size();
            for(unsigned int k = 0; k < maxK; k++)
                colliders.append((long int)check_collision_obstacles_srv.response.collObjs[k]);
        }
        colliders.close();
    }
    else
    {
        colliders.close();
    }
    
    return TRUE;
}

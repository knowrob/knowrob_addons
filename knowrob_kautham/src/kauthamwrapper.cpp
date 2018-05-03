#include "knowrob_kautham/knowrob_kautham.h"

#include <sensor_msgs/JointState.h>

#include <ros/package.h>

#include <stdlib.h>
#include <time.h>
#include <math.h>

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

int getNameConfIndex(const std::string& name)
{
    if("yumi_joint_1_l" == name)
        return 0;
    if("yumi_joint_2_l" == name)
        return 1;
    if("yumi_joint_7_l" == name)
        return 2;
    if("yumi_joint_3_l" == name)
        return 3;
    if("yumi_joint_4_l" == name)
        return 4;
    if("yumi_joint_5_l" == name)
        return 5;
    if("yumi_joint_6_l" == name)
        return 6;
    if("gripper_l_joint" == name)
        return 7;
    if("yumi_joint_1_r" == name)
        return 8;
    if("yumi_joint_2_r" == name)
        return 9;
    if("yumi_joint_7_r" == name)
        return 10;
    if("yumi_joint_3_r" == name)
        return 11;
    if("yumi_joint_4_r" == name)
        return 12;
    if("yumi_joint_5_r" == name)
        return 13;
    if("yumi_joint_6_r" == name)
        return 14;
    if("gripper_r_joint" == name)
        return 15;
    return -1;
}

std::vector<float> normalizeJointVals(std::vector<float> const& inp)
{
    std::vector<float> shifty;
    shifty.resize(16);

    shifty[0] = (inp[0] + 2.941)/(2.941 + 2.941);
    shifty[1] = (inp[1] + 2.505)/(0.759 + 2.505);
    shifty[2] = (inp[2] + 2.941)/(2.941 + 2.941);
    shifty[3] = (inp[3] + 2.155)/(1.396 + 2.155);
    shifty[4] = (inp[4] + 5.061)/(5.061 + 5.061);
    shifty[5] = (inp[5] + 1.536)/(2.409 + 1.536);
    shifty[6] = (inp[6] + 3.997)/(3.997 + 3.997);
    shifty[7] = 1.0;
    shifty[8] = (inp[8] + 2.941)/(2.941 + 2.941);
    shifty[9] = (inp[9] + 2.505)/(0.759 + 2.505);
    shifty[10] = (inp[10] + 2.941)/(2.941 + 2.941);
    shifty[11] = (inp[11] + 2.155)/(1.396 + 2.155);
    shifty[12] = (inp[12] + 5.061)/(5.061 + 5.061);
    shifty[13] = (inp[13] + 1.536)/(2.409 + 1.536);
    shifty[14] = (inp[14] + 3.997)/(3.997 + 3.997);
    shifty[15] = 1.0;
    return shifty;
}

void getCurrentRobotJointState(std::vector<float> &conf)
{
    boost::shared_ptr<const sensor_msgs::JointState> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    unsigned int maxK = msg->position.size();
    std::vector<int> confIndex;
    confIndex.resize(maxK);
    for(int k = 0; k < maxK; k++)
        confIndex[k] = getNameConfIndex(msg->name[k]);
    conf.clear();
    conf.resize(16);
    for(unsigned int k = 0; k < maxK; k++)
    {
        int kc = confIndex[k];
        if(0 <= kc)
            conf[kc] = (float)msg->position[k];
    }
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

        for(int i = 0; i < sizev; i++)
        {
            path[i] = get_path_srv.response.response[i].v;
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
        joint_states.name[0]="yumi_joint_1_l";
        joint_states.name[1]="yumi_joint_2_l";
        joint_states.name[2]="yumi_joint_7_l";
        joint_states.name[3]="yumi_joint_3_l";
        joint_states.name[4]="yumi_joint_4_l";
        joint_states.name[5]="yumi_joint_5_l";
        joint_states.name[6]="yumi_joint_6_l";

        joint_states.name[8]="yumi_joint_1_r";
        joint_states.name[9]="yumi_joint_2_r";
        joint_states.name[10]="yumi_joint_7_r";
        joint_states.name[11]="yumi_joint_3_r";
        joint_states.name[12]="yumi_joint_4_r";
        joint_states.name[13]="yumi_joint_5_r";
        joint_states.name[14]="yumi_joint_6_r";

        joint_states.name[7]="gripper_l_joint";
        joint_states.name[15]="gripper_r_joint";
        joint_states.name[16]="gripper_l_joint_m";
        joint_states.name[17]="gripper_r_joint_m";


        for (int i = 0; i < conf.size(); i++ )
        {

            joint_states.header.stamp = ros::Time::now();
            joint_states.position.resize(18);
            joint_states.position[0]=conf[i][9];
            joint_states.position[1]=conf[i][10];
            joint_states.position[2]=conf[i][11];
            joint_states.position[3]=conf[i][12];
            joint_states.position[4]=conf[i][13];
            joint_states.position[5]=conf[i][14];
            joint_states.position[6]=conf[i][15];

            joint_states.position[8]=conf[i][0];
            joint_states.position[9]=conf[i][1];
            joint_states.position[10]=conf[i][2];
            joint_states.position[11]=conf[i][3];
            joint_states.position[12]=conf[i][4];
            joint_states.position[13]=conf[i][5];
            joint_states.position[14]=conf[i][6];

            joint_states.position[7]=0.03;
            joint_states.position[15]=0.03;
            joint_states.position[16]=0.03;
            joint_states.position[17]=0.03;

            joint_pub.publish(joint_states); //publishing the configuration message to the robot
            loop_rate.sleep();
            //       ros::Duration(0.1).sleep();
        }
        return true;
    }
}

tIKanswer call_Yumi_IK(bool leftArm, double rtx, double rty, double rtz, double rrx, double rry, double rrz, double rrw)
{
    yumik::YumIK Yumi_srv;

    Yumi_srv.request.theta3 = 0.0;

    Yumi_srv.request.pose.position.x = rtx;
    Yumi_srv.request.pose.position.y = rty;
    Yumi_srv.request.pose.position.z = rtz;

    Yumi_srv.request.pose.orientation.x = rrx;
    Yumi_srv.request.pose.orientation.y = rry;
    Yumi_srv.request.pose.orientation.z = rrz;
    Yumi_srv.request.pose.orientation.w = rrw;

    if(leftArm)
        ros::service::call("/Yumi/IKSolverLeft", Yumi_srv);
    else
        ros::service::call("/Yumi/IKSolverRight", Yumi_srv);

    tIKanswer retq;
    retq.response = Yumi_srv.response.status;
    unsigned int maxK = Yumi_srv.response.ik_solution.size();
    retq.conf.resize(maxK);
    for(unsigned int k = 0; k < maxK; k++)
        retq.conf[k] = (float)Yumi_srv.response.ik_solution[k];
    retq.armType = true;

    return retq;
}

PREDICATE(kautham_call_yumi_ik, 3)
{
    std::string armName((char*)PL_A1);
    bool leftArm = true;
    if(armName == "right")
        leftArm = false;
    PlTail pose(PL_A2);
    PlTail answer(PL_A3);
    PlTerm value;
    // object pose in map
    pose.next(value); double rtx = value;
    pose.next(value); double rty = value;
    pose.next(value); double rtz = value;
    pose.next(value); double rrx = value;
    pose.next(value); double rry = value;
    pose.next(value); double rrz = value;
    pose.next(value); double rrw = value;

    tIKanswer retq = call_Yumi_IK(leftArm, rtx, rty, rtz, rrx, rry, rrz, rrw);

    //answer.append((long int)retq.response);
    //answer.append((long int)retq.armType);
    for(int k = 0; k < retq.conf.size(); k++)
        answer.append((double)retq.conf[k]);
    answer.close();
    return TRUE;
}

tIKanswer get_config_from_pose(bool leftArm, double otx, double oty, double otz, double orx, double ory, double orz, double orw, double gtx, double gty, double gtz, double grx, double gry, double grz, double grw)
{
    double rtx;
    double rty;
    double rtz;
    double rrx;
    double rry;
    double rrz;
    double rrw;
    get_grasp_pose_in_map(otx, oty, otz, orx, ory, orz, orw, gtx, gty, gtz, grx, gry, grz, grw, rtx, rty, rtz, rrx, rry, rrz, rrw);

    return call_Yumi_IK(leftArm, rtx + 0.08, rty, rtz - 0.1, rrx, rry, rrz, rrw);
}

std::string resolvePackageURL(std::string const& meshURL)
{
    unsigned int headerLength = std::string("package://").length();
    if(meshURL.length() < headerLength)
        return meshURL;
    std::string head, tail;
    head = meshURL.substr(0, headerLength);
    tail = meshURL.substr(headerLength);
    if("package://" == head)
    {
        unsigned int slash = tail.find('/');
        std::string package = tail.substr(0, slash);
        tail = tail.substr(slash);
        head = ros::package::getPath(package);
        return head + tail;
    }
    else
        return meshURL;
}

PREDICATE(kautham_add_obstacles_internal, 2)
{
    PlTail partDataList(PL_A1);
    PlTail partIndexList(PL_A2);
    PlTerm partData;
    int index = 0;
    while(partDataList.next(partData))
    {
        PlTerm part, mesh, pose, partIndex;
        PlTail data(partData), response(partIndex);
        data.next(part);
        data.next(mesh);
        data.next(pose);

        std::string meshStr((char*)mesh);
        meshStr = resolvePackageURL(meshStr);

        PlTail poseTail(pose);
        PlTerm poseComponent;

        std::vector<float> conf;
        conf.clear(); conf.reserve(7);
        while(poseTail.next(poseComponent))
            conf.push_back((float)((double)poseComponent));

        kautham::AddObstacle addobstacle_srv;
        addobstacle_srv.request.obstacle = meshStr;
        addobstacle_srv.request.scale = 1.0;
        addobstacle_srv.request.home = conf;

        ros::service::call("/kautham_node/AddObstacle", addobstacle_srv);
        index = addobstacle_srv.response.response;
        if(-1 != index)
        {
            response.append(part);
            response.append((long int)index);
            response.close();
            partIndexList.append(partIndex);
        }
    }
    partIndexList.close();
    return TRUE;
}

PREDICATE(kautham_init_planning_scene_internal, 2)
{
    srand(time(NULL));
    std::string modelFolder((char*)PL_A1);
    std::string sceneMap((char*)PL_A2);
    kautham::OpenProblem kthopenproblem_srv;
    kthopenproblem_srv.request.problem = resolvePackageURL(sceneMap);
    kthopenproblem_srv.request.dir.resize(1);
    kthopenproblem_srv.request.dir[0] = resolvePackageURL(modelFolder);
    ros::service::call("kautham_node/OpenProblem", kthopenproblem_srv);
    if (kthopenproblem_srv.response.response == true)
    {
        ROS_INFO( "Kautham Problem opened correctly" );
        return TRUE;
    }
    else
    {
        ROS_ERROR( "ERROR Opening Kautham Problem" );
        ROS_ERROR( "models folder: %s", kthopenproblem_srv.request.dir[0].c_str() );
        ROS_ERROR( "scene map file: %s", kthopenproblem_srv.request.problem.c_str() );
        return FALSE;
    }
}

PREDICATE(kautham_grab_part_internal, 5)
{
    std::string armName((char*)PL_A4);
    bool leftArm = true;
    if(armName == "right")
        leftArm = false;
    int objectIndex = (int)PL_A3;
    int rob = 0;
    int tcp = 6;
    if(leftArm)
        rob = 1;
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

    tIKanswer ikConf = get_config_from_pose(leftArm, otx, oty, otz, orx, ory, orz, orw, gtx, gty, gtz, grx, gry, grz, grw);

    if(ikConf.response)
    {
        std::vector<float> init;
        getCurrentRobotJointState(init);
        std::vector<float> goal = init;
        int b = 0;
        if(!leftArm)
            b = 8;
        for(int k = 0; k < 7; k++)
            goal[k + b] = ikConf.conf[k];
        init = normalizeJointVals(init);
        goal = normalizeJointVals(goal);
        if(!setQuery(init, goal))
        {
            PL_A5 = "fail: query";
            return TRUE;
        }
        std::vector<std::vector<float> > path = collisionFreePath();
        if(!path.size())
        {
            PL_A5 = "fail: path";
            return TRUE;
        }
        pub_rob_motion(path);
        if(!attachObjToRob(rob, tcp, objectIndex))
        {
            PL_A5 = "fail: attach";
            return TRUE;
        }
        PL_A5 = "ok";
        return TRUE;
    }

    PL_A5 = "fail: IK";
    return TRUE;
}

PREDICATE(kautham_put_part_internal, 5)
{
    std::string armName((char*)PL_A4);
    bool leftArm = true;
    if(armName == "right")
        leftArm = false;
    int objectIndex = (int)PL_A3;
    int rob = 0;
    int tcp = 6;
    if(leftArm)
        rob = 1;
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

    tIKanswer ikConf = get_config_from_pose(leftArm, otx, oty, otz, orx, ory, orz, orw, gtx, gty, gtz, grx, gry, grz, grw);

    if(ikConf.response)
    {
        std::vector<float> init;
        getCurrentRobotJointState(init);
        std::vector<float> goal = init;
        int b = 0;
        if(!leftArm)
            b = 8;
        for(int k = 0; k < 7; k++)
            goal[k + b] = ikConf.conf[k];
        init = normalizeJointVals(init);
        goal = normalizeJointVals(goal);
        if(!setQuery(init, goal))
        {
            PL_A5 = "fail: query";
            return TRUE;
        }
        std::vector<std::vector<float> > path = collisionFreePath();
        if(!path.size())
        {
            PL_A5 = "fail: path";
            return TRUE;
        }
        pub_rob_motion(path);
        if(!detachObj(objectIndex))
        {
            PL_A5 = "fail: detachObj";
            return TRUE;
        }
        PL_A5 = "ok";
        return TRUE;
    }

    PL_A5 = "fail: IK";
    return TRUE;
}

PREDICATE(kautham_blocking_objects, 4)
{
    std::string armName((char*)PL_A4);
    bool leftArm = true;
    if(armName == "right")
        leftArm = false;
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

    tIKanswer ikConf = get_config_from_pose(leftArm, otx, oty, otz, orx, ory, orz, orw, gtx, gty, gtz, grx, gry, grz, grw);

    if (ikConf.response)
    {
        kautham::CheckCollision check_collision_obstacles_srv;
        std::vector<float> init;
        getCurrentRobotJointState(init);
        std::vector<float> goal = init;
        int b = 0;
        if(!leftArm)
            b = 8;
        for(int k = 0; k < 7; k++)
            goal[k + b] = ikConf.conf[k];
        std::vector<float> shifty;

        shifty = normalizeJointVals(goal);
        check_collision_obstacles_srv.request.config = shifty;
        ros::service::call("/kautham_node/CheckCollision", check_collision_obstacles_srv);

        //colliders.append((long int)check_collision_obstacles_srv.response.response);
        if(check_collision_obstacles_srv.response.response && (!check_collision_obstacles_srv.response.collisionFree))
        {
            //unsigned int maxK = check_collision_obstacles_srv.response.collObjs.size();
            //for(unsigned int k = 0; k < maxK; k++)
            //    colliders.append((long int)check_collision_obstacles_srv.response.collObjs[k]);
            colliders.append((long int)check_collision_obstacles_srv.response.collObj);
        }
        colliders.close();
    }
    else
    {
        colliders.close();
    }
    
    return TRUE;
}

//ArmName, TableLimits, PartHeight, [GTx, GTy, GTz, GRx, GRy, GRz, GRw], [PRx, PRy, PRz, PRw], PartPosesAndDists, PartGlobalTargetPose, FindResult
PREDICATE(kautham_find_away_pose, 8)
{
    PlTerm value;
    std::string armName((char*)PL_A1);
    bool leftArm = true;
    if(armName == "right")
        leftArm = false;
    PlTail limits(PL_A2);
    double xMin, xMax, yMin, yMax, height;
    limits.next(value); xMin = value;    
    limits.next(value); xMax = value;    
    limits.next(value); yMin = value;    
    limits.next(value); yMax = value;
    limits.next(value); height = value;
    double partHeight = (double)PL_A3;
    PlTail grasppose_list(PL_A4);
    grasppose_list.next(value); double gtx = value;
    grasppose_list.next(value); double gty = value;
    grasppose_list.next(value); double gtz = value;
    grasppose_list.next(value); double grx = value;
    grasppose_list.next(value); double gry = value;
    grasppose_list.next(value); double grz = value;
    grasppose_list.next(value); double grw = value;
    PlTail quat_list(PL_A5);
    quat_list.next(value); double prx = value;
    quat_list.next(value); double pry = value;
    quat_list.next(value); double prz = value;
    quat_list.next(value); double prw = value;
    PlTail posesAndDists_list(PL_A6);
    std::vector<std::vector<double> > posesAndDists;
    posesAndDists.clear();
    while(posesAndDists_list.next(value))
    {
        PlTail poseAndDist_list(value);
        PlTerm component;
        std::vector<double> aux;
        while(poseAndDist_list.next(component))
        {
            aux.push_back((double) component);
        }
        posesAndDists.push_back(aux);
    }

    int maxK = 100;
    int k = 0;
    bool found = false;
    std::vector<double> partPos;
    partPos.resize(7);
    std::vector<std::vector<double> > candidates;
    while((!found) && (k < maxK))
    {
        double ptx = xMin + (((double)rand())/((double)RAND_MAX))*(xMax - xMin);
        double pty = yMin + (((double)rand())/((double)RAND_MAX))*(yMax - yMin);
        double ptz = height + partHeight;
        double rtx, rty, rtz, rrx, rry, rrz, rrw;
        get_grasp_pose_in_map(ptx, pty, ptz, prx, pry, prz, prw, gtx, gty, gtz, grx, gry, grz, grw, rtx, rty, rtz, rrx, rry, rrz, rrw);
        tIKanswer ikConf = call_Yumi_IK(leftArm, rtx + 0.08, rty, rtz - 0.1, rrx, rry, rrz, rrw);
        if(ikConf.response)
        {
            std::vector<double> aux;
            aux.resize(7);
            aux[0] = ptx;
            aux[1] = pty;
            aux[2] = ptz;
            aux[3] = prx;
            aux[4] = pry;
            aux[5] = prz;
            aux[6] = prw;
            double cost = 0.0;
            int maxJ = posesAndDists.size();
            for(int j = 0; j < maxJ; j++)
            {
                double dx = ptx - posesAndDists[j][0];
                double dy = pty - posesAndDists[j][1];
                double d = sqrt(dx*dx + dy*dy);
                d = posesAndDists[j][7] - d;
                if(0 < d)
                    cost += d;
            }
            if(0.001 > cost)
            {
                found = true;
                partPos = aux;
            }
            else
            {
                aux.push_back(cost);
                candidates.push_back(aux);
            }
        }
        k++;
    }

    if((!found) && (candidates.size()))
    {
        found = true;
        int minIdx = 0;
        double cost = 1000.0;
        for(int k = 1; k < candidates.size(); k++)
            if(cost > candidates[k][7])
            {
                cost = candidates[k][7];
                minIdx = k;
            }
        for(int k = 0; k < 7; k++)
            partPos[k] = candidates[minIdx][k];
    }

    PlTail objpose_list(PL_A7);

    if(found)    
    {
        PL_A8 = "ok";
        for(int k = 0; k < 7; k++)
            objpose_list.append(partPos[k]);
        objpose_list.close();
    }
    else
    {
        PL_A8 = "fail: clutter";
    }
    return TRUE;
}


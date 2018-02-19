#include "kautham_wrapper.h"

void ham_product(double ax, double ay, double az, double aw, double bx, double by, double bz, double bw, double &rx, double &ry, double &rz, double &rw)
{
    rx = bw*ax + bx*aw - by*az + bz*ay;
    ry = bw*ay + bx*az + by*aw - bz*ax;
    rz = bw*az - bx*ay + by*ax + bz*aw;
    rw = bw*aw - bx*ax - by*ay - bz*az;
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
    // grasp pose in map
    double dummy;
    double rtx;
    double rty;
    double rtz;
    double rrx;
    double rry;
    double rrz;
    double rrw;
    ham_product(orx, ory, orz, orw, grx, gry, grz, grw, rrx, rry, rrz, rrw);
    ham_product(orx, ory, orz, orw, gtx, gty, gtz, 0, rtx, rty, rtz, dummy);
    ham_product(rtx, rty, rtz, dummy, -orx, -ory, -orz, orw, rtx, rty, rtz, dummy);
    rtx += otx;
    rty += oty;
    rtz += otz;

    ros::NodeHandle n;
    ros::service::waitForService("kautham_node/FindIK");
    ros::ServiceClient Yumi_client = n.serviceClient<kautham::FindIK>("kautham_node/FindIK");
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

    Yumi_client.call(Yumi_srv);

    if (Yumi_srv.response.response)
    {
        ikSol = Yumi_srv.response.conf;
        ros::service::waitForService("/kautham_node/CheckCollisionRob");
        ros::ServiceClient check_collision_obstacles_client = node.serviceClient<kautham::CheckCollision>("/kautham_node/CheckCollisionRob");

        kautham::CheckCollision check_collision_obstacles_srv;
        check_collision_obstacles_srv.request.config = ikSol;
        check_collision_obstacles_client.call(check_collision_obstacles_srv);

        if(!check_collision_obstacles_srv.response.response)
        {
            unsigned int maxK = check_collision_obstacles_srv.response.collObjs.size();
            for(unsigned int k = 0; k < maxK; k++)
                colliders.append(check_collision_obstacles_srv.response.collObjs[k]);
        }
        colliders.close();
    }
    else
    {
        colliders.close();
    }
    
    return TRUE;
}

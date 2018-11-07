#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>
#include<mavros_msgs/PositionTarget.h>
#include<mavros_msgs/CommandBool.h>
#include<mavros_msgs/CommandTOL.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/State.h>
#include<tf/tf.h>
#include <tf/transform_datatypes.h>

#include <math.h>
#include <unistd.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include <iostream>

using namespace std;


//定点模式
#define POS_HOLD_MODE mavros_msgs::PositionTarget::IGNORE_VX|mavros_msgs::PositionTarget::IGNORE_VY|mavros_msgs::PositionTarget::IGNORE_VZ|\
                      mavros_msgs::PositionTarget::IGNORE_AFX|mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|\
                      mavros_msgs::PositionTarget::IGNORE_YAW|mavros_msgs::PositionTarget::IGNORE_YAW_RATE

#define distance (uavCurrentLocalPose.pose.position.x-uav_pos_tar.position.x)*(uavCurrentLocalPose.pose.position.x-uav_pos_tar.position.x)+(uavCurrentLocalPose.pose.position.y-uav_pos_tar.position.y)*(uavCurrentLocalPose.pose.position.y-uav_pos_tar.position.y)

mavros_msgs::State current_state;//飞控状态
geometry_msgs::PoseStamped uavCurrentLocalPose,uavHoldPose;//当前位置,定点位置
mavros_msgs::PositionTarget uav_pos_tar;//local_raw目标点消息
mavros_msgs::CommandTOL land;//自动降落服务
ros::Time last_cmd_time;//上一次接受到命令的时间
double uavRollENU, uavPitchENU, uavYawENU;//飞控ENU姿态

double land_height,landoff_height=1.5;//地面高度、期望的离地高度
double cir_radius=1.5,wa=0.1;//绕圈半径、角速度

int ch;
bool cir_flag=false,mat_flag=false; //自主标志
double t;


void state_cb(const mavros_msgs::State::ConstPtr& msg){//飞控状态消息回调
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//当前位姿消息回调
{
    uavCurrentLocalPose.pose = msg->pose;//更新当前位姿
    //Using ROS tf to get RPY angle from Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(uavCurrentLocalPose.pose.orientation, quat);//tf转换得到姿态
    tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);
    //ROS_INFO("z=%f yaw=%f",msg->pose.position.z,uavYawENU*180/M_PI);
}

char getch()//键盘控制函数
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
    {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
    {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0)
    {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
    {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_ctrl");
    ros::NodeHandle nh;

    //订阅飞机状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //订阅local pose
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    //setpoint_raw话题,根据消息设置不同,可提供全局坐标系下的控速控点,及机体坐标系下的控速
    ros::Publisher raw_tar_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    //rosservice land降落服务
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // 等待飞控连接
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //解锁前高度设置为地面高度
    if(!current_state.armed){
        land_height = uavCurrentLocalPose.pose.position.z;
    }

    uav_pos_tar.header.stamp=ros::Time::now();
    uav_pos_tar.header.frame_id="map";
    uav_pos_tar.coordinate_frame=mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//设置为全局坐标系
    uav_pos_tar.type_mask=POS_HOLD_MODE;


    t=ros::Time::now().toSec();
    last_cmd_time=ros::Time::now();

    
    while(ros::ok())
    {
        ros::spinOnce();
        if(current_state.mode != "OFFBOARD")
        {
            uav_pos_tar.header.stamp=ros::Time::now();
            uav_pos_tar.position.x=uavCurrentLocalPose.pose.position.x;
            uav_pos_tar.position.y=uavCurrentLocalPose.pose.position.y;
            //uav_pos_tar.position.z=uavCurrentLocalPose.pose.position.z;
        }
        else
        {
            uav_pos_tar.position.z=land_height+landoff_height;
        }

        ch = getch();
        if (ch != EOF)
        {
            switch (ch)
            {
                case 'w':     //key w
                {
                    uav_pos_tar.position.y += 1;
                    ROS_INFO("w:y+1");
                 }break;
                case 's':     //key s
                {
                    uav_pos_tar.position.y -= 1;
                    ROS_INFO("s:y-1");
                 }break;
                case 'd':     //key d
                {
                    uav_pos_tar.position.x += 1;
                    ROS_INFO("d:x+1");
                 }break;
                case 'a':      //key a
                {
                    uav_pos_tar.position.x -= 1;
                    ROS_INFO("a:x-1");
                 }break;
                case 'c':      //key c 绕圈
                {
                    //巡点起点
                    uavHoldPose.pose.position.x=uavCurrentLocalPose.pose.position.x;
                    uavHoldPose.pose.position.y=uavCurrentLocalPose.pose.position.y;
                    cir_flag = true;
                }break;
                case 'x':      //key x 矩形
                {
                    //巡点起点
                    uavHoldPose.pose.position.x=uavCurrentLocalPose.pose.position.x;
                    uavHoldPose.pose.position.y=uavCurrentLocalPose.pose.position.y;
                    //矩形规划目标点集
                    vector<geometry_msgs::Point> goal(8);
                    goal[0].x=uavHoldPose.pose.position.x+1;goal[0].y=uavHoldPose.pose.position.y;
                    goal[1].x=goal[0].x+1;  goal[1].y=goal[0].y;
                    goal[2].x=goal[1].x;    goal[2].y=goal[1].y+1;
                    goal[3].x=goal[2].x;    goal[3].y=goal[2].y+1;
                    goal[4].x=goal[3].x-1;  goal[4].y=goal[3].y;
                    goal[5].x=goal[4].x-1;  goal[5].y=goal[4].y;
                    goal[6].x=goal[5].x;    goal[6].y=goal[5].y-1;
                    goal[7].x=goal[6].x;    goal[7].y=goal[6].y-1;

                    for(int i=0;i<goal.size();i++)
                    {
                        uav_pos_tar.position.x = goal[i].x;
                        uav_pos_tar.position.y = goal[i].y;
                        ROS_INFO("%f %f",uav_pos_tar.position.x,uav_pos_tar.position.y);
                        while(distance > 0.09)
                        {
                            raw_tar_pos_pub.publish(uav_pos_tar);
                            ros::spinOnce();
                            rate.sleep();
                        }     
                        cout << "reach the target point" << endl;               
                    }
                }break;
                case 'l':      //key l land降落
                {
                    if(land_client.call(land) && land.response.success)
                    {
                        ROS_INFO("landing enabled");
                    }
                }break;
             }
        }
        if(cir_flag && cir_radius != 0)
        {
            double ct=ros::Time::now().toSec()-t;
            uav_pos_tar.position.x = uavHoldPose.pose.position.x+cir_radius*sin(wa*ct);
            uav_pos_tar.position.y = uavHoldPose.pose.position.y+cir_radius*cos(wa*ct);
            //ROS_INFO("%f",ros::Time::now().toSec()-t);
            ROS_INFO("%f %f",uavCurrentLocalPose.pose.position.x,uavCurrentLocalPose.pose.position.y);
        }
        raw_tar_pos_pub.publish(uav_pos_tar);
        rate.sleep();
    }
}

#include "ros/ros.h"
#include "server_communicate_pkg/AddInts.h"

bool doreq(server_communicate_pkg::AddInts::Request &req, server_communicate_pkg::AddInts::Response &resp)
{
    int num1 = req.num1;
    int num2 = req.num2;

    if (num1 < 0 || num2 < 0)
    {
        ROS_ERROR("请求值必须大于0");
        return false;
    }
    resp.sum = num1 + num2;
    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    // 初始化ros节点，固定的argc，argv，节点名（）必须唯一
    ros::init(argc, argv, "server");

    // 下面两步就是在注册消息回调
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService("AddInts", doreq);

    // 注册完回调就进循环等消息，spin是阻塞，spinonce是处理一条
    ros::spin();

    return 0;
}

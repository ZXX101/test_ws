#include <ros/ros.h>
#include "server_communicate_pkg/AddInts.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<server_communicate_pkg::AddInts>("AddInts");
    ros::service::waitForService("AddInts");

    server_communicate_pkg::AddInts ai;
    ai.request.num1 = atoi(argv[0]);
    ai.request.num2 = atoi(argv[1]);
    client.call(ai);

    return 0;
}
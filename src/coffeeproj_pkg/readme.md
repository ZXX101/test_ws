## 流程
main函数启动了CoffeeProj类的构造函数和run
* CoffeeProj类构造

__init__函数
mqtt 初始化
订阅4个相关主题
配置连接参数并连接

# 启动
roscore
make px4_sitl_default gazebo-classic
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" use_sim_time:=false

最后执行./start_coffeeproj.sh
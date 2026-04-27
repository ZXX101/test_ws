## 流程
main函数启动了CoffeeProj类的构造函数和run
* CoffeeProj类构造

__init__函数
mqtt 初始化
订阅4个相关主题
配置连接参数并连接

# 启动
roscore

roslaunch mission_master_pkg mavros_px4_udp.launch fcu_url:=udp://:14550@255.255.255.255:14550

最后执行./start_coffeeproj.sh
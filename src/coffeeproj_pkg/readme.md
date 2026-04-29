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




# 代码流程
## 一些实现问题
三个类有重复的ros节点订阅，是否有必要？
## 一些实现细节
使用参数，在初始化时传递类指针
## 概览
⬇️ 启动
__init__
CoffeeProjNode： 
    初始化ros，仅订阅反馈节点（设置回调）
    初始化三个个类
    CLASS MQTTClient
        初始化mqtt客户端。
        设置onConnect回调，onMessage回调，onDisconnect（设置重连标志位用于检测重连）
            onConnect
                订阅TASK和COMMAND节点用于接收消息
            onMessage
                处理接收消息，也就是订阅的那两个节点
                    匹配主题，处理
                        handle_task->
                        handle_command->

    CLASS FlightController飞行控制类：实现飞行控制接口
        获取launch参数
        订阅mavros话题设置回调
        阻塞等待mavros服务启动
    CLASS TaskStateMachine状态机：
        初始化状态
    启动TaskStateMachine状态处理线程，其中的状态处理函数是阻塞的。

    启动Timer用于发布遥测消息

## 详细描述
### MQTTClient
> connect(self)
调用mqtt库的connect连接mqttBroker
loob_start() 启动mqtt库的消息处理循环，内部会开启一个线程，通过disconnect来退出线程

> onConnect(self,client:)
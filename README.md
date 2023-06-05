# casia_humanoid_mujoco

功能说明：mujoco仿真环境及控制代码，以humanoid为例，完成一个仿真环境的最小实现，通过UDP实现仿真环境与控制器的通信，可以读取机器人状态并对机器人各关节进行控制。用户可根据自己的需求，修改仿真模型并完成运动平衡控制器。

运行环境：
Ubuntu22.04

mujoco210下载地址及使用详见
https://mujoco.readthedocs.io/en/latest/programming/index.html

使用方法：
将mujoco安装在~/.mujoco/mujoco210/目录下，分别启动humanoid_ctrl与humanoid_sim，机器人的关节输出控制器发送的力矩指令。

代码说明：
（1）humanoid_ctrl文件夹为控制器，main.cpp为主函数，使用UDP与仿真器进行通信，其通信数据由robot_state与robot_ctrl定义，robot_state为仿真器传入数据，robot_ctrl为控制器输出数据，用户可根据自身需要修改该数据格式。

UDP使用方式参考https://www.geeksforgeeks.org/udp-server-client-implementation-c

（2）humanoid_sim文件夹为仿真器，同样使用UDP与控制器进行通信，其通信数据由robot_state与robot_ctrl定义，robot_state为仿真器输出数据，robot_ctrl为控制器输入数据。

（3）humanoid_sim文件夹中，humanoid.xml为模型文件，其描述内容参考：
https://mujoco.readthedocs.io/en/latest/XMLreference.html 

（4）humanoid_sim文件夹中，humanoidmujoco.c为仿真器相关函数封装，其功能参考mujoco文档：
https://mujoco.readthedocs.io/en/latest/programming/index.html 

mujoco相关API使用参考文档：
https://mujoco.readthedocs.io/en/latest/APIreference/index.html 

（5）humanoid_sim文件夹中，main.c为仿真器主函数，用于控制仿真器的运行。

（6）humanoid_sim文件夹中，main.m文件为matlab文件，用于处理仿真过程中产生的数据，需使用matlab打开。

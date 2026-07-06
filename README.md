# -RK3588-
AI视觉识别、嵌入式、yolo v8、elf2、物联网

1. 连接与启动摄像头

串口连接：将STM32的PA9连接到Elf2的10号引脚，PA10连接到8号引脚，并接地。
启动摄像头：
点击Terminal图标进入终端。
输入 cd Desktop/ 进入桌面目录（或使用桌面右键“在终端中打开”快捷方式）。
激活虚拟环境：输入 source tieluchubing/bin/activate 。
进入脚本目录：输入 cd ai-tieluchubing/ 。
运行Python脚本：输入 sudo python3 x4.py 。

2. 串口调试命令

安装串口助手：输入 sudo apt-get install cutecom 。
打开串口助手：输入 sudo cutecom 。

3. 控制STM32舵机与继电器

控制舵机：通过发送JSON格式的数据来控制舵机角度或模式。例如：
四舵机归零： {"servo1": 90, "servo2": 0, "servo3": 0, "servo4": 0} 
测试舵机1角度： {"servo1": 0} （最小角度）或 {"servo1": 180} （最大角度）
单个舵机测试： {"servo3": 120} 
切换模式：启用自动或手动模式，分别发送 {"auto": 0} （自动模式）或 {"auto": 1} （手动模式）。
启动继电器：发送 {"switch": 1} 。

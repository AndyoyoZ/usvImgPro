无人船平台图像处理程序v0.1
主要节点：１）ros_cv　用于获取视频，并将图像信息发布到　camera/image　
　　　　　２）yolocv_kcf 执行yolo和kcf，实现目标识别与跟踪，
　　　　　　　若成功，则通过　cmd_vel_mux/input/teleop　发布速度信息，
　　　　　　　若失败，则通过发布　flag_cv_to_nav　交接控制权　
运行方式：　直接在终端运行　runImgPro.sh　脚本文件即可，该脚本文件会调用 yolocv_kcf/launch/runImgPro.launch
其他说明：　可以通过修改yolocv_kcf/launch/runImgPro.launch文件配置视频来源和yolo模型文件，yolo模型文件存放在yolocv_kcf/data目录下
　　　　　　


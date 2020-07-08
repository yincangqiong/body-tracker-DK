# body-tracker-DK
Azure_kinect_DK设备的人体姿态检测

1、官网下载Azure Kinect SDK 1.3.0与Azure Kinect Body Tracking SDK 1.0.1.msi,并安装(默认位置即可)

2、k4a.lib、k4abt.lib、opencv_world3410.lib、opencv_world3410d.lib库放到该工程目录下，并在IDE中添加库

3、在Azure Kinect SDK 1.3.0与Azure Kinect Body Tracking SDK 1.0.1目录中找到以下库函数，并放到该工程目录下：
cublas64_100.dll、cudart64_100.dll、cudnn64_7.dll、depthengine_2_0.dll、dnn_model_2_0.onnx、k4a.dll、k4abt.dll、onnxruntime.dll

4、安装Opencv并将以下库函数放到该工程目录下：
opencv_ffmpeg3410_64.dll、opencv_world3410.dll、opencv_world3410d.dll

5、将opencv2的文件夹放到该工程目录中

6、IDE使用VS 2019,X64-debug运行即可

7、action-detection.cpp检测了几种瑜伽动作

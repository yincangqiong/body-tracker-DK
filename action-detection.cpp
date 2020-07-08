
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "include/k4a.h"
#include "include/k4abt.h"
#include <math.h>

// OpenCV
#include "include/opencv.hpp"
// Kinect DK
#include "include/k4a.hpp"

#define VERIFY(result, error)                                                                        \
if(result != K4A_RESULT_SUCCEEDED)                                                                   \
{                                                                                                    \
    printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
    exit(1);                                                                                         \
}                                                                                                    
float get_angle(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3)
{
    float dis1 = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2) + pow((z1 - z2), 2));
    float dis2 = sqrt(pow((x2 - x3), 2) + pow((y2 - y3), 2) + pow((z2 - z3), 2));
    float dis3 = sqrt(pow((x1 - x3), 2) + pow((y1 - y3), 2) + pow((z1 - z3), 2));
    float angle = acos((dis1 * dis1 + dis2 * dis2 - dis3 * dis3) / (2 * dis1 * dis2)) * 180.0 / 3.1415926;
    return angle;
}

int main()
{
    //定义倾角
    

    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(1, &device), "Open K4A Device failed!");

    const uint32_t device_count = k4a_device_get_installed_count();
    if (1 == device_count)
    {
        std::cout << "Found " << device_count << " connected devices. " << std::endl;
    }
    else
    {
        std::cout << "Error: more than one K4A devices found. " << std::endl;
    }

    //打开设备
    k4a_device_open(1, &device);
    std::cout << "Done: open device. " << std::endl;

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    deviceConfig.synchronized_images_only = true;// ensures that depth and color images are both available in the capture

    //开始相机
    //k4a_device_start_cameras(device, &deviceConfig);
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");
    std::cout << "Done: start camera." << std::endl;

    //查询传感器校准
    k4a_calibration_t sensor_calibration;
    k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration);
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration),
        "Get depth camera calibration failed!");
    //创建人体跟踪器
    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker);
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

    cv::Mat cv_rgbImage_with_alpha;
    cv::Mat cv_rgbImage_no_alpha;

    int frame_count = 0;
    //定义计数器
    int action1 = 0;
    int action2 = 0;
    int action3 = 0;
    int action4 = 0;
    while(true)
    {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        //获取RGB和depth图像
        k4a_image_t rgbImage = k4a_capture_get_color_image(sensor_capture);

        //RGB
        cv_rgbImage_with_alpha = cv::Mat(k4a_image_get_height_pixels(rgbImage), k4a_image_get_width_pixels(rgbImage), CV_8UC4, k4a_image_get_buffer(rgbImage));
        cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);      

        //计算姿态
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            frame_count++;
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
            k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            }
            else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }

            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Successfully popped the body tracking result. Start your processing 
                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);

                for (size_t i = 0; i < num_bodies; i++)
                {
                    k4abt_skeleton_t skeleton;
                    k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                    //std::cout << typeid(skeleton.joints->position.v).name();

                    k4a_float2_t P_NOSE_2D;
                    k4a_float2_t P_EYE_RIGHT_2D;
                    k4a_float2_t P_EAR_RIGHT_2D;
                    k4a_float2_t P_EYE_LEFT_2D;
                    k4a_float2_t P_EAR_LEFT_2D;
                    k4a_float2_t P_SHOULDER_RIGHT_2D;
                    k4a_float2_t P_SHOULDER_LEFT_2D;
                    k4a_float2_t P_ELBOW_RIGHT_2D;
                    k4a_float2_t P_ELBOW_LEFT_2D;
                    k4a_float2_t P_WRIST_RIGHT_2D;
                    k4a_float2_t P_WRIST_LEFT_2D;
                    k4a_float2_t P_HAND_RIGHT_2D;
                    k4a_float2_t P_HAND_LEFT_2D;
                    k4a_float2_t P_THUMB_RIGHT_2D;
                    k4a_float2_t P_THUMB_LEFT_2D;
                    k4a_float2_t P_HANDTIP_RIGHT_2D;
                    k4a_float2_t P_HANDTIP_LEFT_2D;
                    k4a_float2_t P_SPINE_CHEST_2D;
                    k4a_float2_t P_HEAD_2D;
                    k4a_float2_t P_NECK_2D;
                    k4a_float2_t P_SPINE_NAVEL_2D;
                    k4a_float2_t P_PELVIS_2D;
                    k4a_float2_t P_CLAVICLE_RIGHT_2D;
                    k4a_float2_t P_CLAVICLE_LEFT_2D;
                    k4a_float2_t P_HIP_RIGHT_2D;
                    k4a_float2_t P_HIP_LEFT_2D;
                    k4a_float2_t P_KNEE_LEFT_2D;
                    k4a_float2_t P_KNEE_RIGHT_2D;
                    k4a_float2_t P_ANKLE_LEFT_2D;
                    k4a_float2_t P_ANKLE_RIGHT_2D;
                    k4a_float2_t P_FOOT_LEFT_2D;
                    k4a_float2_t P_FOOT_RIGHT_2D;
                    int result;

                    //头部
                    k4abt_joint_t  P_NOSE = skeleton.joints[K4ABT_JOINT_NOSE];
                    k4abt_joint_t  P_HEAD = skeleton.joints[K4ABT_JOINT_HEAD];
                    k4abt_joint_t  P_EYE_RIGHT = skeleton.joints[K4ABT_JOINT_EYE_RIGHT];
                    k4abt_joint_t  P_EAR_RIGHT = skeleton.joints[K4ABT_JOINT_EAR_RIGHT];
                    k4abt_joint_t  P_EYE_LEFT = skeleton.joints[K4ABT_JOINT_EYE_LEFT];
                    k4abt_joint_t  P_EAR_LEFT = skeleton.joints[K4ABT_JOINT_EAR_LEFT];
                    //3D转2D，并在color中画出
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HEAD.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HEAD_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_NOSE.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_NOSE_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_EYE_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_EYE_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_EAR_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_EAR_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_EYE_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_EYE_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_EAR_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_EAR_LEFT_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_NOSE_2D.xy.x, P_NOSE_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_EYE_RIGHT_2D.xy.x, P_EYE_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_EAR_RIGHT_2D.xy.x, P_EAR_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_EYE_LEFT_2D.xy.x, P_EYE_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_EAR_LEFT_2D.xy.x, P_EAR_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));

                    //颈部+胳膊
                    k4abt_joint_t  P_NECK = skeleton.joints[K4ABT_JOINT_NECK];
                    k4abt_joint_t  P_CLAVICLE_RIGHT = skeleton.joints[K4ABT_JOINT_CLAVICLE_RIGHT];
                    k4abt_joint_t  P_CLAVICLE_LEFT = skeleton.joints[K4ABT_JOINT_CLAVICLE_LEFT];
                    k4abt_joint_t  P_SHOULDER_RIGHT = skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT];
                    k4abt_joint_t  P_SHOULDER_LEFT = skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT];
                    k4abt_joint_t  P_ELBOW_RIGHT = skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT];
                    k4abt_joint_t  P_ELBOW_LEFT = skeleton.joints[K4ABT_JOINT_ELBOW_LEFT];
                    k4abt_joint_t  P_WRIST_RIGHT = skeleton.joints[K4ABT_JOINT_WRIST_RIGHT];
                    k4abt_joint_t  P_WRIST_LEFT = skeleton.joints[K4ABT_JOINT_WRIST_LEFT];
                    k4abt_joint_t  P_HAND_RIGHT = skeleton.joints[K4ABT_JOINT_HAND_RIGHT];
                    k4abt_joint_t  P_HAND_LEFT = skeleton.joints[K4ABT_JOINT_HAND_LEFT];
                    k4abt_joint_t  P_HANDTIP_RIGHT = skeleton.joints[K4ABT_JOINT_HANDTIP_RIGHT];
                    k4abt_joint_t  P_HANDTIP_LEFT = skeleton.joints[K4ABT_JOINT_HANDTIP_LEFT];
                    k4abt_joint_t  P_THUMB_RIGHT = skeleton.joints[K4ABT_JOINT_THUMB_RIGHT];
                    k4abt_joint_t  P_THUMB_LEFT = skeleton.joints[K4ABT_JOINT_THUMB_LEFT];
                    //3D转2D，并在color中画出
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_NECK.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_NECK_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_CLAVICLE_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_CLAVICLE_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_CLAVICLE_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_CLAVICLE_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_SHOULDER_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_SHOULDER_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_SHOULDER_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_SHOULDER_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_ELBOW_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_ELBOW_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_ELBOW_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_ELBOW_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_WRIST_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_WRIST_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_WRIST_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_WRIST_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HAND_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HAND_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HAND_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HAND_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HANDTIP_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HANDTIP_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HANDTIP_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HANDTIP_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_THUMB_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_THUMB_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_THUMB_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_THUMB_LEFT_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_NECK_2D.xy.x, P_NECK_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_CLAVICLE_RIGHT_2D.xy.x, P_CLAVICLE_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_CLAVICLE_LEFT_2D.xy.x, P_CLAVICLE_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_SHOULDER_RIGHT_2D.xy.x, P_SHOULDER_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_SHOULDER_LEFT_2D.xy.x, P_SHOULDER_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_ELBOW_RIGHT_2D.xy.x, P_ELBOW_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_ELBOW_LEFT_2D.xy.x, P_ELBOW_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_WRIST_RIGHT_2D.xy.x, P_WRIST_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_WRIST_LEFT_2D.xy.x, P_WRIST_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_HAND_RIGHT_2D.xy.x, P_HAND_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_HAND_LEFT_2D.xy.x, P_HAND_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_HANDTIP_RIGHT_2D.xy.x, P_HANDTIP_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_HANDTIP_LEFT_2D.xy.x, P_HANDTIP_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_THUMB_RIGHT_2D.xy.x, P_THUMB_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_THUMB_LEFT_2D.xy.x, P_THUMB_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));

                    //胸部
                    k4abt_joint_t  P_SPINE_CHEST = skeleton.joints[K4ABT_JOINT_SPINE_CHEST];
                    k4abt_joint_t  P_SPINE_NAVEL = skeleton.joints[K4ABT_JOINT_SPINE_NAVEL];
                    //3D转2D，并在color中画出
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_SPINE_CHEST.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_SPINE_CHEST_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_SPINE_NAVEL.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_SPINE_NAVEL_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_SPINE_CHEST_2D.xy.x, P_SPINE_CHEST_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_SPINE_NAVEL_2D.xy.x, P_SPINE_NAVEL_2D.xy.y), 3, cv::Scalar(0, 255, 255));


                    //髋部+下肢
                    k4abt_joint_t  P_PELVIS = skeleton.joints[K4ABT_JOINT_PELVIS];
                    k4abt_joint_t  P_HIP_RIGHT = skeleton.joints[K4ABT_JOINT_HIP_RIGHT];
                    k4abt_joint_t  P_KNEE_RIGHT = skeleton.joints[K4ABT_JOINT_KNEE_RIGHT];
                    k4abt_joint_t  P_HIP_LEFT = skeleton.joints[K4ABT_JOINT_HIP_LEFT];
                    k4abt_joint_t  P_KNEE_LEFT = skeleton.joints[K4ABT_JOINT_KNEE_LEFT];
                    k4abt_joint_t  P_ANKLE_RIGHT = skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT];
                    k4abt_joint_t  P_ANKLE_LEFT = skeleton.joints[K4ABT_JOINT_ANKLE_LEFT];
                    k4abt_joint_t  P_FOOT_LEFT = skeleton.joints[K4ABT_JOINT_FOOT_LEFT];
                    k4abt_joint_t  P_FOOT_RIGHT = skeleton.joints[K4ABT_JOINT_FOOT_RIGHT];

                    //3D转2D，并在color中画出
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_PELVIS.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_PELVIS_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HIP_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HIP_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_KNEE_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_KNEE_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HIP_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HIP_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_KNEE_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_KNEE_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_FOOT_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_FOOT_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_ANKLE_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_ANKLE_LEFT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_FOOT_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_FOOT_RIGHT_2D, &result);
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_ANKLE_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_ANKLE_RIGHT_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_KNEE_LEFT_2D.xy.x, P_KNEE_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_HIP_LEFT_2D.xy.x, P_HIP_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_KNEE_RIGHT_2D.xy.x, P_KNEE_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_HIP_RIGHT_2D.xy.x, P_HIP_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_PELVIS_2D.xy.x, P_PELVIS_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_ANKLE_RIGHT_2D.xy.x, P_ANKLE_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_ANKLE_LEFT_2D.xy.x, P_ANKLE_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_FOOT_RIGHT_2D.xy.x, P_FOOT_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_FOOT_LEFT_2D.xy.x, P_FOOT_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
 
                    //连接骨骼点
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_EYE_RIGHT_2D.xy.x, P_EYE_RIGHT_2D.xy.y), cv::Point(P_EAR_RIGHT_2D.xy.x, P_EAR_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_EYE_LEFT_2D.xy.x, P_EYE_LEFT_2D.xy.y), cv::Point(P_EAR_LEFT_2D.xy.x, P_EAR_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_EYE_RIGHT_2D.xy.x, P_EYE_RIGHT_2D.xy.y), cv::Point(P_NOSE_2D.xy.x, P_NOSE_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_EYE_LEFT_2D.xy.x, P_EYE_LEFT_2D.xy.y), cv::Point(P_NOSE_2D.xy.x, P_NOSE_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_NOSE_2D.xy.x, P_NOSE_2D.xy.y), cv::Point(P_HEAD_2D.xy.x, P_HEAD_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_HEAD_2D.xy.x, P_HEAD_2D.xy.y), cv::Point(P_NECK_2D.xy.x, P_NECK_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_NECK_2D.xy.x, P_NECK_2D.xy.y), cv::Point(P_SPINE_CHEST_2D.xy.x, P_SPINE_CHEST_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_SPINE_CHEST_2D.xy.x, P_SPINE_CHEST_2D.xy.y), cv::Point(P_SPINE_NAVEL_2D.xy.x, P_SPINE_NAVEL_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_SPINE_CHEST_2D.xy.x, P_SPINE_CHEST_2D.xy.y), cv::Point(P_CLAVICLE_RIGHT_2D.xy.x, P_CLAVICLE_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_SPINE_CHEST_2D.xy.x, P_SPINE_CHEST_2D.xy.y), cv::Point(P_CLAVICLE_LEFT_2D.xy.x, P_CLAVICLE_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_CLAVICLE_RIGHT_2D.xy.x, P_CLAVICLE_RIGHT_2D.xy.y), cv::Point(P_SHOULDER_RIGHT_2D.xy.x, P_SHOULDER_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_CLAVICLE_LEFT_2D.xy.x, P_CLAVICLE_LEFT_2D.xy.y), cv::Point(P_SHOULDER_LEFT_2D.xy.x, P_SHOULDER_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_SHOULDER_RIGHT_2D.xy.x, P_SHOULDER_RIGHT_2D.xy.y), cv::Point(P_ELBOW_RIGHT_2D.xy.x, P_ELBOW_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_SHOULDER_LEFT_2D.xy.x, P_SHOULDER_LEFT_2D.xy.y), cv::Point(P_ELBOW_LEFT_2D.xy.x, P_ELBOW_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_ELBOW_RIGHT_2D.xy.x, P_ELBOW_RIGHT_2D.xy.y), cv::Point(P_WRIST_RIGHT_2D.xy.x, P_WRIST_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_ELBOW_LEFT_2D.xy.x, P_ELBOW_LEFT_2D.xy.y), cv::Point(P_WRIST_LEFT_2D.xy.x, P_WRIST_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_WRIST_LEFT_2D.xy.x, P_WRIST_LEFT_2D.xy.y), cv::Point(P_HAND_LEFT_2D.xy.x, P_HAND_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_WRIST_RIGHT_2D.xy.x, P_WRIST_RIGHT_2D.xy.y), cv::Point(P_HAND_RIGHT_2D.xy.x, P_HAND_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_WRIST_RIGHT_2D.xy.x, P_WRIST_RIGHT_2D.xy.y), cv::Point(P_THUMB_RIGHT_2D.xy.x, P_THUMB_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_WRIST_LEFT_2D.xy.x, P_WRIST_LEFT_2D.xy.y), cv::Point(P_THUMB_LEFT_2D.xy.x, P_THUMB_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_HAND_RIGHT_2D.xy.x, P_HAND_RIGHT_2D.xy.y), cv::Point(P_HANDTIP_RIGHT_2D.xy.x, P_HANDTIP_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_HAND_LEFT_2D.xy.x, P_HAND_LEFT_2D.xy.y), cv::Point(P_HANDTIP_LEFT_2D.xy.x, P_HANDTIP_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_SPINE_NAVEL_2D.xy.x, P_SPINE_NAVEL_2D.xy.y), cv::Point(P_PELVIS_2D.xy.x, P_PELVIS_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_PELVIS_2D.xy.x, P_PELVIS_2D.xy.y), cv::Point(P_HIP_RIGHT_2D.xy.x, P_HIP_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_PELVIS_2D.xy.x, P_PELVIS_2D.xy.y), cv::Point(P_HIP_LEFT_2D.xy.x, P_HIP_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_HIP_RIGHT_2D.xy.x, P_HIP_RIGHT_2D.xy.y), cv::Point(P_KNEE_RIGHT_2D.xy.x, P_KNEE_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_HIP_LEFT_2D.xy.x, P_HIP_LEFT_2D.xy.y), cv::Point(P_KNEE_LEFT_2D.xy.x, P_KNEE_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_KNEE_RIGHT_2D.xy.x, P_KNEE_RIGHT_2D.xy.y), cv::Point(P_ANKLE_RIGHT_2D.xy.x, P_ANKLE_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_KNEE_LEFT_2D.xy.x, P_KNEE_LEFT_2D.xy.y), cv::Point(P_ANKLE_LEFT_2D.xy.x, P_ANKLE_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_ANKLE_RIGHT_2D.xy.x, P_ANKLE_RIGHT_2D.xy.y), cv::Point(P_FOOT_RIGHT_2D.xy.x, P_FOOT_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_ANKLE_LEFT_2D.xy.x, P_ANKLE_LEFT_2D.xy.y), cv::Point(P_FOOT_LEFT_2D.xy.x, P_FOOT_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);


                    //计算关节倾角角度
                    float leg_angle_right = get_angle(P_CLAVICLE_RIGHT.position.xyz.x, P_CLAVICLE_RIGHT.position.xyz.y, P_CLAVICLE_RIGHT.position.xyz.z, P_HIP_RIGHT.position.xyz.x, P_HIP_RIGHT.position.xyz.y, P_HIP_RIGHT.position.xyz.z, P_KNEE_RIGHT.position.xyz.x, P_KNEE_RIGHT.position.xyz.y, P_KNEE_RIGHT.position.xyz.z);
                    std::cout << "右腿抬起角度;" << leg_angle_right << std::endl;
                    float leg_angle_left = get_angle(P_CLAVICLE_LEFT.position.xyz.x, P_CLAVICLE_LEFT.position.xyz.y, P_CLAVICLE_LEFT.position.xyz.z, P_HIP_LEFT.position.xyz.x, P_HIP_LEFT.position.xyz.y, P_HIP_LEFT.position.xyz.z, P_KNEE_LEFT.position.xyz.x, P_KNEE_LEFT.position.xyz.y, P_KNEE_LEFT.position.xyz.z);
                    std::cout << "左腿抬起角度;" << leg_angle_left << std::endl;
                    float arm_angle_right = get_angle(P_SPINE_CHEST.position.xyz.x, P_SPINE_CHEST.position.xyz.y, P_SPINE_CHEST.position.xyz.z, P_CLAVICLE_RIGHT.position.xyz.x, P_CLAVICLE_RIGHT.position.xyz.y, P_CLAVICLE_RIGHT.position.xyz.z, P_SHOULDER_RIGHT.position.xyz.x, P_SHOULDER_RIGHT.position.xyz.y, P_SHOULDER_RIGHT.position.xyz.z);
                    std::cout << "右胳膊抬起角度;" << arm_angle_right << std::endl;
                    float arm_angle_left = get_angle(P_SPINE_CHEST.position.xyz.x, P_SPINE_CHEST.position.xyz.y, P_SPINE_CHEST.position.xyz.z, P_CLAVICLE_LEFT.position.xyz.x, P_CLAVICLE_LEFT.position.xyz.y, P_CLAVICLE_LEFT.position.xyz.z, P_SHOULDER_LEFT.position.xyz.x, P_SHOULDER_LEFT.position.xyz.y, P_SHOULDER_LEFT.position.xyz.z);
                    std::cout << "左胳膊抬起角度;" << arm_angle_left << std::endl;
                    float hip_angel_right = get_angle(P_PELVIS.position.xyz.x, P_PELVIS.position.xyz.y, P_PELVIS.position.xyz.z, P_HIP_RIGHT.position.xyz.x, P_HIP_RIGHT.position.xyz.y, P_HIP_RIGHT.position.xyz.z, P_KNEE_RIGHT.position.xyz.x, P_KNEE_RIGHT.position.xyz.y, P_KNEE_RIGHT.position.xyz.z);
                    std::cout << "右髋劈开角度;" << hip_angel_right << std::endl;
                    float hip_angel_left = get_angle(P_PELVIS.position.xyz.x, P_PELVIS.position.xyz.y, P_PELVIS.position.xyz.z, P_HIP_LEFT.position.xyz.x, P_HIP_LEFT.position.xyz.y, P_HIP_LEFT.position.xyz.z, P_KNEE_LEFT.position.xyz.x, P_KNEE_LEFT.position.xyz.y, P_KNEE_LEFT.position.xyz.z);
                    std::cout << "左髋劈开角度;" << hip_angel_left << std::endl;
                    float knee_angle_right = get_angle(P_HIP_RIGHT.position.xyz.x, P_HIP_RIGHT.position.xyz.y, P_HIP_RIGHT.position.xyz.z, P_KNEE_RIGHT.position.xyz.x, P_KNEE_RIGHT.position.xyz.y, P_KNEE_RIGHT.position.xyz.z, P_ANKLE_RIGHT.position.xyz.x,P_ANKLE_RIGHT.position.xyz.y, P_ANKLE_RIGHT.position.xyz.z);
                    std::cout << "右膝角度;" << knee_angle_right << std::endl;
                    float knee_angle_left = get_angle(P_HIP_LEFT.position.xyz.x, P_HIP_LEFT.position.xyz.y, P_HIP_LEFT.position.xyz.z, P_KNEE_LEFT.position.xyz.x, P_KNEE_LEFT.position.xyz.y, P_KNEE_LEFT.position.xyz.z, P_ANKLE_LEFT.position.xyz.x, P_ANKLE_LEFT.position.xyz.y, P_ANKLE_LEFT.position.xyz.z);
                    std::cout << "左膝角度;" << knee_angle_left << std::endl;
                    float elbow_angel_right = get_angle(P_SHOULDER_RIGHT.position.xyz.x, P_SHOULDER_RIGHT.position.xyz.y, P_ANKLE_RIGHT.position.xyz.z, P_ELBOW_RIGHT.position.xyz.x, P_ELBOW_RIGHT.position.xyz.y, P_ELBOW_RIGHT.position.xyz.z, P_WRIST_RIGHT.position.xyz.x, P_WRIST_RIGHT.position.xyz.y, P_WRIST_RIGHT.position.xyz.z);
                    std::cout << "右肘角度;" << elbow_angel_right << std::endl;
                    float elbow_angel_left = get_angle(P_SHOULDER_LEFT.position.xyz.x, P_SHOULDER_LEFT.position.xyz.y, P_ANKLE_LEFT.position.xyz.z, P_ELBOW_LEFT.position.xyz.x, P_ELBOW_LEFT.position.xyz.y, P_ELBOW_LEFT.position.xyz.z, P_WRIST_LEFT.position.xyz.x, P_WRIST_LEFT.position.xyz.y, P_WRIST_LEFT.position.xyz.z);
                    std::cout << "左肘角度;" << elbow_angel_left << std::endl;

                    //动作1
                    if (hip_angel_left > 140 && knee_angle_left < 120 && hip_angel_right > 120 && arm_angle_right > 90 && arm_angle_left > 90 && arm_angle_right > 90)
                    {
                        action1++;
                        if (action1>20)
                        {
                            for (int i = 0; i < 5000; i++)
                            {
                                cv::putText(cv_rgbImage_no_alpha, "GOOD JOB", cv::Point(500, 360), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0, 255, 0), 8, 8);
                            } 
                            action1 = 0;
                        }
                    }
                    //动作2
                    if (hip_angel_left > 160 && hip_angel_right > 160 && knee_angle_right < 100 && knee_angle_left < 100 && arm_angle_right > 90 && arm_angle_left > 90 && elbow_angel_right <100 && elbow_angel_left < 100)
                    {
                        action2++;
                        if (action2 > 20)
                        {
                            for (int i = 0; i < 5000; i++)
                            {
                                cv::putText(cv_rgbImage_no_alpha, "GOOD JOB", cv::Point(500, 360), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0, 255, 0), 8, 8);
                            }
                            action2 = 0;
                        }
                    }
                    //动作3
                    if (arm_angle_right > 100 && arm_angle_left > 120 && elbow_angel_right > 120 && elbow_angel_right > 120 && knee_angle_right < 75)
                    {
                        action3++;
                        if (action3 > 20)
                        {
                            for (int i = 0; i < 5000; i++)
                            {
                                cv::putText(cv_rgbImage_no_alpha, "GOOD JOB", cv::Point(500, 360), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0, 255, 0), 8, 8);
                            }
                            action3 = 0;
                        }
                    }
                    //动作4
                    if (hip_angel_left >120 && knee_angle_left < 100 && arm_angle_right > 110)
                    {
                        action4++;
                        if (action4 > 20)
                        {
                            for (int i = 0; i < 5000; i++)
                            {
                                cv::putText(cv_rgbImage_no_alpha, "GOOD JOB", cv::Point(500, 360), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0, 255, 0), 8, 8);
                            }
                            action4 = 0;
                        }
                    }
                    

                    uint32_t id = k4abt_frame_get_body_id(body_frame, i);

                }
                printf("%zu bodies are detected!\n", num_bodies);

                k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }
        }
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // It should never hit time out when K4A_WAIT_INFINITE is set.
            printf("Error! Get depth frame time out!\n");
            break;
        }
        else
        {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }
        
        // show image
        imshow("Action-detection", cv_rgbImage_no_alpha);
        cv::waitKey(0);
        k4a_image_release(rgbImage);
    }

    printf("Finished body tracking processing!\n");

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}
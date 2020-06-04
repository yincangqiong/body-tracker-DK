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

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

int main()
{
    //定义身高
    float UPPER_BODY; //上身长度
    float body_angel; //上身倾角

    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

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
    k4a_device_open(0, &device);
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
    cv::Mat cv_depth;
    cv::Mat cv_depth_8U;

    int frame_count = 0;
    while(true)
    {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        //获取RGB和depth图像
        k4a_image_t rgbImage = k4a_capture_get_color_image(sensor_capture);
        k4a_image_t depthImage = k4a_capture_get_depth_image(sensor_capture);
        //RGB
        cv_rgbImage_with_alpha = cv::Mat(k4a_image_get_height_pixels(rgbImage), k4a_image_get_width_pixels(rgbImage), CV_8UC4, k4a_image_get_buffer(rgbImage));
        cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);      
        //depth
        cv_depth = cv::Mat(k4a_image_get_height_pixels(depthImage), k4a_image_get_width_pixels(depthImage), CV_16U, k4a_image_get_buffer(depthImage), k4a_image_get_stride_bytes(depthImage));
        cv_depth.convertTo(cv_depth_8U, CV_8U, 1);


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

                    k4a_float2_t P_HEAD_2D;
                    k4a_float2_t P_NECK_2D;
                    k4a_float2_t P_CHEST_2D;
                    k4a_float2_t P_HIP_2D;
                    k4a_float2_t P_CLAVICLE_RIGHT_2D;
                    k4a_float2_t P_CLAVICLE_LEFT_2D;
                    k4a_float2_t P_HIP_RIGHT_2D;
                    k4a_float2_t P_HIP_LEFT_2D;
                    k4a_float2_t P_KNEE_LEFT_2D;
                    k4a_float2_t P_KNEE_RIGHT_2D;
                    int result;

                    //头部
                    k4abt_joint_t  P_HEAD = skeleton.joints[K4ABT_JOINT_NOSE];
                    //3D转2D，并在color中画出
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HEAD.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HEAD_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_HEAD_2D.xy.x, P_HEAD_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    
                    //颈部
                    k4abt_joint_t  P_NECK = skeleton.joints[K4ABT_JOINT_NECK];
                    //3D转2D，并在color中画出
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_NECK.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_CHEST_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_CHEST_2D.xy.x, P_CHEST_2D.xy.y), 3, cv::Scalar(0, 255, 255));

                    //胸部
                    k4abt_joint_t  P_CHEST = skeleton.joints[K4ABT_JOINT_SPINE_CHEST];
                    //3D转2D，并在color中画出
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_CHEST.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_NECK_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_NECK_2D.xy.x, P_NECK_2D.xy.y), 3, cv::Scalar(0, 255, 255));

                    //髋部
                    k4abt_joint_t  P_HIP = skeleton.joints[K4ABT_JOINT_PELVIS];
                    //3D转2D，并在color中画出
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HIP.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HIP_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_HIP_2D.xy.x, P_HIP_2D.xy.y), 3, cv::Scalar(0, 255, 255));

                    //右肩（计算上身倾角需要）
                    k4abt_joint_t  P_CLAVICLE_RIGHT = skeleton.joints[K4ABT_JOINT_CLAVICLE_RIGHT];
                    //3D转2D，并在color中画出
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_CLAVICLE_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_CLAVICLE_RIGHT_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_CLAVICLE_RIGHT_2D.xy.x, P_CLAVICLE_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));

                    //右髋（计算上身倾角需要）
                    k4abt_joint_t  P_HIP_RIGHT = skeleton.joints[K4ABT_JOINT_HIP_RIGHT];
                    //3D转2D，并在color中画出,并画出右肩到右髋的连线
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HIP_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HIP_RIGHT_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_HIP_RIGHT_2D.xy.x, P_HIP_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_CLAVICLE_RIGHT_2D.xy.x, P_CLAVICLE_RIGHT_2D.xy.y), cv::Point(P_HIP_RIGHT_2D.xy.x, P_HIP_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);


                    //右膝（计算上身倾角需要）
                    k4abt_joint_t  P_KNEE_RIGHT = skeleton.joints[K4ABT_JOINT_KNEE_RIGHT];
                    //3D转2D，并在color中画出,并画出右肩到右膝、右髋到右膝的连线
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_KNEE_RIGHT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_KNEE_RIGHT_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_KNEE_RIGHT_2D.xy.x, P_KNEE_RIGHT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_CLAVICLE_RIGHT_2D.xy.x, P_CLAVICLE_RIGHT_2D.xy.y), cv::Point(P_KNEE_RIGHT_2D.xy.x, P_KNEE_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_HIP_RIGHT_2D.xy.x, P_HIP_RIGHT_2D.xy.y), cv::Point(P_KNEE_RIGHT_2D.xy.x, P_KNEE_RIGHT_2D.xy.y), cv::Scalar(0, 0, 255), 2);

                    //左肩（计算上身倾角需要）
                    k4abt_joint_t  P_CLAVICLE_LEFT = skeleton.joints[K4ABT_JOINT_CLAVICLE_LEFT];
                    //3D转2D，并在color中画出
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_CLAVICLE_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_CLAVICLE_LEFT_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_CLAVICLE_LEFT_2D.xy.x, P_CLAVICLE_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));

                    //左髋（计算上身倾角需要）
                    k4abt_joint_t  P_HIP_LEFT = skeleton.joints[K4ABT_JOINT_HIP_LEFT];
                    //3D转2D，并在color中画出,并画出左肩到左髋的连线
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_HIP_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_HIP_LEFT_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_HIP_LEFT_2D.xy.x, P_HIP_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_HIP_LEFT_2D.xy.x, P_HIP_LEFT_2D.xy.y), cv::Point(P_CLAVICLE_LEFT_2D.xy.x, P_CLAVICLE_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);

                    //左膝（计算上身倾角需要）
                    k4abt_joint_t  P_KNEE_LEFT = skeleton.joints[K4ABT_JOINT_KNEE_LEFT];
                    //3D转2D，并在color中画出,并画出左肩到左膝、左髋到左膝的连线
                    k4a_calibration_3d_to_2d(&sensor_calibration, &P_KNEE_LEFT.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &P_KNEE_LEFT_2D, &result);
                    cv::circle(cv_rgbImage_no_alpha, cv::Point(P_KNEE_LEFT_2D.xy.x, P_KNEE_LEFT_2D.xy.y), 3, cv::Scalar(0, 255, 255));
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_CLAVICLE_LEFT_2D.xy.x, P_CLAVICLE_LEFT_2D.xy.y), cv::Point(P_KNEE_LEFT_2D.xy.x, P_KNEE_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    cv::line(cv_rgbImage_no_alpha, cv::Point(P_HIP_LEFT_2D.xy.x, P_HIP_LEFT_2D.xy.y), cv::Point(P_KNEE_LEFT_2D.xy.x, P_KNEE_LEFT_2D.xy.y), cv::Scalar(0, 0, 255), 2);
                    

                    //输出头部关键点坐标（skeleton.joints_HEAD->position.v为头部坐标点，数据结构float[3]）
                    char text_head[50];
                    std::cout << "头部坐标：";
                    for (size_t i = 0; i < 3; i++)
                    {
                        std::cout << P_HEAD.position.v[i] << " ";
                    }
                    printf("\n");

                    //输出颈部关键点坐标
                    std::cout << "颈部坐标：";
                    for (size_t i = 0; i < 3; i++)
                    {
                        std::cout << P_NECK.position.v[i] << " " ;
                    }
                    printf("\n");

                    //输出胸部关键点坐标
                    std::cout << "胸部坐标：";
                    for (size_t i = 0; i < 3; i++)
                    {
                        std::cout << P_CHEST.position.v[i] << " ";
                    }
                    printf("\n");

                    //输出髋部关键点坐标
                    std::cout << "髋部坐标：";
                    for (size_t i = 0; i < 3; i++)
                    {
                        std::cout << P_HIP.position.v[i] << " ";
                    }
                    printf("\n");

                    //估算人体身高
                    UPPER_BODY= sqrt(pow((P_NECK.position.xyz.x - P_HIP.position.xyz.x),2) +pow((P_NECK.position.xyz.y - P_HIP.position.xyz.y),2) + pow((P_NECK.position.xyz.z - P_HIP.position.xyz.z),2));
                    float HEIGHT = UPPER_BODY  * 1770/518;
                    std::cout << "估算身高高度是;" << HEIGHT << std::endl;

                    //估算上身倾角
                    float ang_clavicetohip_right = sqrt(pow((P_CLAVICLE_RIGHT.position.xyz.x - P_HIP_RIGHT.position.xyz.x), 2) + pow((P_CLAVICLE_RIGHT.position.xyz.y - P_HIP_RIGHT.position.xyz.y), 2) + pow((P_CLAVICLE_RIGHT.position.xyz.z - P_HIP_RIGHT.position.xyz.z), 2));
                    float ang_clavicetohip_left = sqrt(pow((P_CLAVICLE_LEFT.position.xyz.x - P_HIP_LEFT.position.xyz.x), 2) + pow((P_CLAVICLE_LEFT.position.xyz.y - P_HIP_LEFT.position.xyz.y), 2) + pow((P_CLAVICLE_LEFT.position.xyz.z - P_HIP_LEFT.position.xyz.z), 2));
                    float ang_hiptoknee_right = sqrt(pow((P_HIP_RIGHT.position.xyz.x - P_KNEE_RIGHT.position.xyz.x), 2) + pow((P_HIP_RIGHT.position.xyz.y - P_KNEE_RIGHT.position.xyz.y), 2) + pow((P_HIP_RIGHT.position.xyz.z - P_KNEE_RIGHT.position.xyz.z), 2));
                    float ang_hiptoknee_left = sqrt(pow((P_HIP_LEFT.position.xyz.x - P_KNEE_LEFT.position.xyz.x), 2) + pow((P_HIP_LEFT.position.xyz.y - P_KNEE_LEFT.position.xyz.y), 2) + pow((P_HIP_LEFT.position.xyz.z - P_KNEE_LEFT.position.xyz.z), 2));
                    float ang_clavicetoknee_right = sqrt(pow((P_CLAVICLE_RIGHT.position.xyz.x - P_KNEE_RIGHT.position.xyz.x), 2) + pow((P_CLAVICLE_RIGHT.position.xyz.y - P_KNEE_RIGHT.position.xyz.y), 2) + pow((P_CLAVICLE_RIGHT.position.xyz.z - P_KNEE_RIGHT.position.xyz.z), 2));
                    float ang_clavicetoknee_left = sqrt(pow((P_CLAVICLE_LEFT.position.xyz.x - P_KNEE_LEFT.position.xyz.x), 2) + pow((P_CLAVICLE_LEFT.position.xyz.y - P_KNEE_LEFT.position.xyz.y), 2) + pow((P_CLAVICLE_LEFT.position.xyz.z - P_KNEE_LEFT.position.xyz.z), 2));
                    
                    float body_angel_right = acos((ang_clavicetohip_right * ang_clavicetohip_right + ang_hiptoknee_right * ang_hiptoknee_right - ang_clavicetoknee_right * ang_clavicetoknee_right) / (2 * ang_clavicetohip_right * ang_hiptoknee_right)) * 180.0 / 3.1415926;
                    float body_angel_left = acos((ang_clavicetohip_left * ang_clavicetohip_left + ang_hiptoknee_left * ang_hiptoknee_left - ang_clavicetoknee_left * ang_clavicetoknee_left) / (2 * ang_clavicetohip_left * ang_hiptoknee_left)) * 180.0 / 3.1415926;
                    body_angel = (body_angel_right + body_angel_left)/2;
                    std::cout << "估算上身倾角是;" << body_angel << std::endl;

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
        imshow("color", cv_rgbImage_no_alpha);
        imshow("depth", cv_depth_8U);
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
import pyrealsense2 as rs
import numpy as np
import cv2

''' 
设置
'''
pipeline = rs.pipeline()  # 定义流程pipeline，创建一个管道
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)  # 配置depth流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)  # 配置color流

pipe_profile = pipeline.start(config)  # streaming流开始

# 创建对齐对象与color流对齐
align_to = rs.stream.color  # align_to 是计划对齐深度帧的流类型
align = rs.align(align_to)  # rs.align 执行深度帧与其他帧的对齐

# 全局变量存储鼠标点击的坐标
clicked_point = None

''' 
鼠标回调函数
'''
def mouse_callback(event, x, y, flags, param):
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:  # 左键点击
        clicked_point = [x, y]
        print(f"鼠标点击坐标: ({x}, {y})")

''' 
获取对齐图像帧与相机参数
'''
def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧，获取颜色和深度的框架集
    aligned_frames = align.process(frames)  # 获取对齐帧，将深度框与颜色框对齐

    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
    aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧

    #### 获取相机参数 ####
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参

    #### 将images转为numpy arrays ####
    img_color = np.asanyarray(aligned_color_frame.get_data())  # RGB图
    img_depth = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）

    return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame

''' 
获取三维坐标
'''
def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = depth_pixel[0]
    y = depth_pixel[1]
    dis = aligned_depth_frame.get_distance(x, y)  # 获取该像素点对应的深度
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    return dis, camera_coordinate

if __name__ == "__main__":
    # 创建窗口并设置鼠标回调
    cv2.namedWindow('RealSence')
    cv2.setMouseCallback('RealSence', mouse_callback)
    
    print("使用方法：")
    print("1. 在图像上点击任意位置选择点")
    print("2. 按 'q' 键退出程序")
    print("3. 按 'c' 键清除当前选择的点")
    
    while True:
        ''' 
        获取对齐图像帧与相机参数
        '''
        color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images()

        # 创建图像的副本用于显示，避免修改原始图像
        display_img = img_color.copy()
        
        # 如果有鼠标点击，获取该点的三维坐标
        if clicked_point is not None:
            try:
                dis, camera_coordinate = get_3d_camera_coordinate(clicked_point, aligned_depth_frame, depth_intrin)
                
                # 在控制台输出坐标信息
                print(f"\n选中点信息:")
                print(f"像素坐标: ({clicked_point[0]}, {clicked_point[1]})")
                print(f"深度: {dis:.3f} m")
                print(f"相机坐标系坐标: X={camera_coordinate[0]:.3f}m, Y={camera_coordinate[1]:.3f}m, Z={camera_coordinate[2]:.3f}m")
                
                # 在图像上标记点和显示信息
                cv2.circle(display_img, tuple(clicked_point), 8, [255, 0, 255], thickness=-1)
                cv2.circle(display_img, tuple(clicked_point), 10, [255, 255, 255], thickness=2)
                
                # 显示坐标信息
                info_y = 30
                cv2.putText(display_img, f"Pixel: ({clicked_point[0]}, {clicked_point[1]})", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [255, 255, 255], 2)
                info_y += 30
                cv2.putText(display_img, f"Depth: {dis:.3f} m", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [0, 0, 255], 2)
                info_y += 30
                cv2.putText(display_img, f"X: {camera_coordinate[0]:.3f} m", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [255, 0, 0], 2)
                info_y += 30
                cv2.putText(display_img, f"Y: {camera_coordinate[1]:.3f} m", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [0, 255, 0], 2)
                info_y += 30
                cv2.putText(display_img, f"Z: {camera_coordinate[2]:.3f} m", 
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [255, 255, 0], 2)
                
            except Exception as e:
                print(f"获取坐标时出错: {e}")
                cv2.putText(display_img, "Invalid depth data", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [0, 0, 255], 2)
        else:
            # 显示提示信息
            cv2.putText(display_img, "Click anywhere to select a point", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [255, 255, 255], 2)
            cv2.putText(display_img, "Press 'c' to clear, 'q' to quit", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, [255, 255, 255], 2)
        
        # 显示图像
        cv2.imshow('RealSence', display_img)
        
        # 键盘操作
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # 按q退出
            break
        elif key == ord('c'):  # 按c清除当前点
            clicked_point = None
            print("已清除选中的点")
    
    # 清理资源
    pipeline.stop()
    cv2.destroyAllWindows()
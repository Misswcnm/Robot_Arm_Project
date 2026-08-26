#include "RobotTask.h"
#include <iostream>
#include <thread>


int main() {

    positionServer=new MachineController (8888);  // 作为服务器监听8888端口
    camera=new CameraController();
    estun=new EstunController ("192.168.2.5", "9000");
    
    // 连接机械臂
    if (!estun->connect()) {
        std::cerr << "Failed to connect to robot" << std::endl;
        return 1;
    }
    
    // 设置状态回调
    // controller.setStatusCallback([](c2::RobotState state) {
    //     std::cout << "Robot state changed: " << static_cast<int>(state) << std::endl;
    // });
    
    //启用机器人
    if (!estun->enableRobot()) {
        std::cerr << "Failed to enable robot" << std::endl;
        return 1;
    }
    
    // 设置自动模式
    if (!estun->setAutoMode()) {
        std::cerr << "Failed to set auto mode" << std::endl;
        return 1;
    }
    sleep(2);
    if (!estun->resetRobot()) {
        std::cerr << "Failed to go to global reset point" << std::endl;
        return 1;
    }
    g_robotState = RobotState::IDLE;

    std::thread taskWorker(taskThread);
    std::thread positionReceiver(positionReceiverThread);
    std::thread senderWorker(statusSenderThread);
    //std::thread detect(detectThread,"/home/gao/TCP_asdun_arm/picture");
    // std::thread imageSender(imageSenderThread);


    positionReceiver.detach();
    taskWorker.detach();
    senderWorker.detach();
    //detect.detach();
    // imageSender.detach();


    while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    }  
        
    return 0;

}
#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <atomic>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class CameraController {
public:
    CameraController();
    bool captureImage();
    bool isImageCaptured() const;
    void stabilizeAfterCapture();
    void resetCaptureState();

private:
    std::atomic<bool> m_captureSuccess;
};

#endif // CAMERA_CONTROLLER_H
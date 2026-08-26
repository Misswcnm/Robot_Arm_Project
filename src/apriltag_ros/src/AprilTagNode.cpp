// ros
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#if defined(__has_include)
#  if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#    include <tf2_ros/transform_broadcaster.hpp>
#  else
#    include <tf2_ros/transform_broadcaster.h>
#  endif
#else
#  include <tf2_ros/transform_broadcaster.h>
#endif

// apriltag
#include "tag_functions.hpp"
#include <apriltag.h>

#include <cstdint>
#include <chrono>
#include <limits>


#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

bool image_to_mono8(const sensor_msgs::msg::Image& image, cv::Mat& mono,
                    const rclcpp::Logger& logger)
{
    if(image.width == 0 || image.height == 0 ||
       image.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
       image.height > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        RCLCPP_WARN_STREAM(logger, "Dropping invalid image dimensions: "
                                      << image.width << "x" << image.height);
        return false;
    }

    int type = 0;
    int conversion = -1;
    uint32_t bytes_per_pixel = 0;
    if(image.encoding == "mono8") {
        type = CV_8UC1;
        bytes_per_pixel = 1;
    }
    else if(image.encoding == "rgb8") {
        type = CV_8UC3;
        bytes_per_pixel = 3;
        conversion = cv::COLOR_RGB2GRAY;
    }
    else if(image.encoding == "bgr8") {
        type = CV_8UC3;
        bytes_per_pixel = 3;
        conversion = cv::COLOR_BGR2GRAY;
    }
    else if(image.encoding == "rgba8") {
        type = CV_8UC4;
        bytes_per_pixel = 4;
        conversion = cv::COLOR_RGBA2GRAY;
    }
    else if(image.encoding == "bgra8") {
        type = CV_8UC4;
        bytes_per_pixel = 4;
        conversion = cv::COLOR_BGRA2GRAY;
    }
    else {
        RCLCPP_WARN_STREAM(logger, "Dropping unsupported image encoding: "
                                      << image.encoding);
        return false;
    }

    const uint64_t minimum_step =
        static_cast<uint64_t>(image.width) * bytes_per_pixel;
    const uint64_t required_bytes =
        static_cast<uint64_t>(image.step) * image.height;
    if(image.step < minimum_step || required_bytes > image.data.size()) {
        RCLCPP_WARN_STREAM(
            logger, "Dropping malformed image: encoding=" << image.encoding
                    << " size=" << image.width << "x" << image.height
                    << " step=" << image.step << " data=" << image.data.size());
        return false;
    }

    try {
        const cv::Mat view(
            static_cast<int>(image.height), static_cast<int>(image.width), type,
            const_cast<unsigned char*>(image.data.data()), image.step);
        if(conversion < 0) {
            mono = view.clone();
        }
        else {
            cv::cvtColor(view, mono, conversion);
        }
    }
    catch(const cv::Exception& error) {
        RCLCPP_WARN_STREAM(logger, "Dropping image after OpenCV conversion error: "
                                      << error.what());
        return false;
    }

    return !mono.empty() && mono.type() == CV_8UC1 &&
           mono.step <= static_cast<size_t>(std::numeric_limits<int>::max());
}

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions& options);

    ~AprilTagNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    // parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    double max_detection_rate_hz;
    std::mutex detection_rate_mutex;
    std::chrono::steady_clock::time_point last_detection_time;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    std::function<void(apriltag_family_t*)> tf_destructor;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info;
    std::mutex camera_info_mutex;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    pose_estimation_f estimate_pose = nullptr;

    void onImage(sensor_msgs::msg::Image::SharedPtr msg_img);

    void onCameraInfo(
        sensor_msgs::msg::CameraInfo::SharedPtr msg_ci);

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)


AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    tf_broadcaster(
#ifdef tf2_ros_NODE_INTERFACE
        tf2_ros::TransformBroadcaster::RequiredInterfaces { *this }
#else
        this
#endif
    )
{
    // read-only parameters
    const std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));

    // get tag names, IDs and sizes
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // get method for estimating tag pose
    const std::string& pose_estimation_method =
        declare_parameter("pose_estimation_method", "pnp",
                          descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster), "
                                "set to \"\" (empty) to disable pose estimation",
                                true));

    if(!pose_estimation_method.empty()) {
        if(pose_estimation_methods.count(pose_estimation_method)) {
            estimate_pose = pose_estimation_methods.at(pose_estimation_method);
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown pose estimation method '" << pose_estimation_method << "'.");
        }
    }

    // detector parameters in "detector" namespace
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    td->refine_edges = declare_parameter<bool>(
        "detector.refine", td->refine_edges != 0,
        descr("snap to strong gradients")) ? 1 : 0;
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    td->debug = declare_parameter<bool>(
        "detector.debug", td->debug != 0,
        descr("write additional debugging images to working directory"))
        ? 1 : 0;
    max_detection_rate_hz = declare_parameter(
        "detector.max_rate_hz", 5.0,
        descr("maximum image processing rate; <= 0 processes every frame",
              true));

    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    const std::string qos_profile = declare_parameter(
        "qos_profile", "default",
        descr("qos profile to use. 'default', 'sensor_data' or "
              "'system_default'", true));
    rclcpp::QoS camera_qos(rclcpp::KeepLast(10));
    if(qos_profile == "sensor_data") {
        camera_qos = rclcpp::SensorDataQoS();
    }
    else if(qos_profile == "system_default") {
        camera_qos = rclcpp::SystemDefaultsQoS();
    }
    else if(qos_profile != "default") {
        throw std::runtime_error("Unknown qos_profile: " + qos_profile);
    }

    // CameraInfo is stable calibration data.  Caching the newest message and
    // processing every image avoids the Foxy image_transport synchronizer,
    // which can stop pairing valid RealSense image/info streams.
    sub_camera_info = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", camera_qos,
        std::bind(&AprilTagNode::onCameraInfo, this, std::placeholders::_1));
    sub_image = create_subscription<sensor_msgs::msg::Image>(
        "image_rect", camera_qos,
        std::bind(&AprilTagNode::onImage, this, std::placeholders::_1));

    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    if(!sizes.empty()) {
        // use tag specific size
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }

    RCLCPP_INFO_STREAM(
        get_logger(), "AprilTag camera subscription: image="
        << sub_image->get_topic_name() << " camera_info="
        << sub_camera_info->get_topic_name()
        << " synchronization=latest_camera_info");
}

AprilTagNode::~AprilTagNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagNode::onCameraInfo(
    sensor_msgs::msg::CameraInfo::SharedPtr msg_ci)
{
    std::lock_guard<std::mutex> lock(camera_info_mutex);
    latest_camera_info = msg_ci;
}

void AprilTagNode::onImage(
    sensor_msgs::msg::Image::SharedPtr msg_img)
{
    if(max_detection_rate_hz > 0.0) {
        const auto current = std::chrono::steady_clock::now();
        const auto minimum_period = std::chrono::duration<double>(
            1.0 / max_detection_rate_hz);
        std::lock_guard<std::mutex> lock(detection_rate_mutex);
        if(last_detection_time.time_since_epoch().count() != 0 &&
           current - last_detection_time < minimum_period) {
            return;
        }
        last_detection_time = current;
    }

    sensor_msgs::msg::CameraInfo::ConstSharedPtr msg_ci;
    {
        std::lock_guard<std::mutex> lock(camera_info_mutex);
        msg_ci = latest_camera_info;
    }
    if(!msg_ci) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Waiting for camera_info before AprilTag detection");
        return;
    }
    onCamera(msg_img, msg_ci);
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    // Raw image coordinates require the original camera matrix and distortion.
    const std::array<double, 4> intrinsics = {
        msg_ci->k[0], msg_ci->k[4], msg_ci->k[2], msg_ci->k[5]};
    const std::vector<double> distortion(msg_ci->d.begin(), msg_ci->d.end());
    const bool dimensions_match =
        msg_ci->width == msg_img->width && msg_ci->height == msg_img->height;

    // check for valid intrinsics
    const bool calibrated = msg_ci->width && msg_ci->height &&
                            dimensions_match && intrinsics[0] && intrinsics[1] &&
                            intrinsics[2] && intrinsics[3];

    if(estimate_pose != nullptr && !calibrated) {
        RCLCPP_WARN_STREAM(
            get_logger(),
            "Invalid camera calibration or image/CameraInfo size mismatch: image="
                << msg_img->width << "x" << msg_img->height << ", camera_info="
                << msg_ci->width << "x" << msg_ci->height);
    }

    // Convert without cv_bridge so a Foxy cv_bridge built against a different
    // OpenCV ABI cannot corrupt cv::Mat metadata. Bad transitional V4L frames
    // are rejected locally instead of terminating the detector process.
    cv::Mat img_uint8;
    if(!image_to_mono8(*msg_img, img_uint8, get_logger())) { return; }

    image_u8_t im{img_uint8.cols, img_uint8.rows,
                  static_cast<int>(img_uint8.step), img_uint8.data};

    // detect tags
    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;

    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    for(int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        RCLCPP_DEBUG(get_logger(),
                     "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id,
                     det->hamming, det->decision_margin);

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming > max_hamming) { continue; }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
        msg_detections.detections.push_back(msg_detection);

        // 3D orientation and position
        if(estimate_pose != nullptr && calibrated) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg_img->header;
            // set child frame name by generic tag name or configured tag name
            tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            try {
                tf.transform = estimate_pose(
                    det, intrinsics, distortion, size);
                tfs.push_back(tf);
            }
            catch(const cv::Exception& error) {
                RCLCPP_WARN_STREAM(
                    get_logger(), "Skipping tag " << det->id
                    << " after OpenCV pose error: " << error.what());
            }
        }
    }

    pub_detections->publish(msg_detections);

    if(estimate_pose != nullptr)
        tf_broadcaster.sendTransform(tfs);

    apriltag_detections_destroy(detections);
}

rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    mutex.lock();

    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);

        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        if(parameter.get_name() == "detector.refine") {
            td->refine_edges = parameter.get_value<bool>() ? 1 : 0;
            continue;
        }
        IF("detector.sharpening", td->decode_sharpening)
        if(parameter.get_name() == "detector.debug") {
            td->debug = parameter.get_value<bool>() ? 1 : 0;
            continue;
        }
        IF("max_hamming", max_hamming)
        IF("profile", profile)
    }

    mutex.unlock();

    result.successful = true;

    return result;
}

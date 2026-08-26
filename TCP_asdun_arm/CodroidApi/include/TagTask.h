#pragma once

#include "D455Camera.h"

#include <array>
#include <filesystem>
#include <memory>
#include <string>

struct TagTaskConfig
{
    std::filesystem::path record_root{"."};
    std::filesystem::path handeye_json;
    std::filesystem::path tcp_calibration_json;
    double tag_size_mm{};
};

// 配置中的相对路径统一相对于配置文件所在目录解析。
bool loadTagTaskConfig(const std::filesystem::path &config_path,
                       TagTaskConfig &config,
                       std::string &error);

// 纯 C++ AprilTag 示教：检测 Ref，并把 Tag-relative Work 写入 command JSON。
class TagTask
{
public:
    using Transform = std::array<double, 16>;

    explicit TagTask(TagTaskConfig config);
    ~TagTask();

    TagTask(const TagTask &) = delete;
    TagTask &operator=(const TagTask &) = delete;

    // command=1：检测指定 ID，计算并写入 tag_pose=T_base_tag_ref。
    bool recordReference(const std::string &mapid,
                         const std::string &poseid,
                         const D455Frame &frame,
                         int tag_id,
                         std::string &error) const;

    // command>1：写入 trans=inv(T_base_tag_ref)*T_base_flange_work。
    bool recordWork(const std::string &mapid,
                    const std::string &poseid,
                    int command,
                    std::string &error) const;

    // 执行端定位：用当前稳定法兰位姿和新彩色帧计算 T_base_tag。
    // flange_pose 顺序为 [x,y,z,a,b,c]，单位 mm/deg；输出按行展开。
    bool locateBaseTag(const D455Frame &frame,
                       int tag_id,
                       const std::array<double, 6> &flange_pose,
                       Transform &base_tag,
                       std::string &error) const;

    // 以示教 Ref 末端位姿为中心，直接沿基座 Y 轴左右平移。
    // lateral_mm: 左为正、右为负；只改变 Y，其他 XYZ/ABC 保持 Ref 原值。
    bool baseLateralObservationPose(
        const std::array<double, 6> &center_flange_pose,
        double lateral_mm,
        std::array<double, 6> &shifted_flange_pose,
        std::string &error) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::filesystem::path record_root_;
    std::filesystem::path handeye_path_;
    std::filesystem::path tcp_calibration_path_;
    double tag_size_mm_;
};

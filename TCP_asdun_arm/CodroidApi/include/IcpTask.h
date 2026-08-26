#pragma once

#include "D455Camera.h"

#include <filesystem>
#include <string>
#include <vector>

// 只负责 ICP 示教数据落盘，不负责相机取帧、机械臂状态或 ICP 匹配执行。
class IcpTask
{
public:
    explicit IcpTask(std::filesystem::path record_root = ".");

    // Ref: 合并多帧点云，保存 20/10/5 mm 模板金字塔；位姿直接读取
    // normalRecord 已保存的 command=1 JSON。
    bool recordReference(
        const std::string &mapid,
        const std::string &poseid,
        const std::vector<std::vector<D455PointXYZ>> &frames,
        std::string &error) const;

    // Work: 读取 Ref/Work 的 normalRecord 位姿，并把相对矩阵追加到 Work JSON。
    // T_ref_work = inv(T_base_flange_ref) * T_base_flange_work。
    bool recordWork(
        const std::string &mapid,
        const std::string &poseid,
        int command,
        std::string &error) const;

private:
    std::filesystem::path record_root_;
};

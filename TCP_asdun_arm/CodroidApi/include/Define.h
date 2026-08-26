#pragma once

#include "nlohmann/json.hpp"
#include <string>

using json = nlohmann::ordered_json;

namespace c2 {

enum class ResponseCode : int {
    OK = 0,         // ��������
    Timeout,        // ����ʱ
    QueueFull,      // �����������
    NetError,       // �����쳣
    RequestFailed,  // ����ʧ��
};

struct Response {
    ResponseCode code{ResponseCode::OK};
    std::string  msg;
    json         data;
};

/**
 * ������״̬.
 */
enum class RobotState : int {
    None    = -1,  // δ֪
    Init    = 0,   // ��ʼ��
    StandBy = 1,   // ���µ�
    Ready   = 2,   // �ֶ�ģʽ
    Rescue  = 3,   // ��Ԯģʽ
    Auto    = 4,   // �Զ�ģʽ
    Error   = 6,   // �����˳���
};

/**
 * �ٶ�
 */
struct Speed {
    Speed() = default;
    Speed(double j, double t, double o) : joint(j), tcp(t), ori(o) {}
    
    double joint{0};  // 关节速度,用于关节空间运动, 单位: deg/s
    double tcp{0};    // 末端速度,用于笛卡尔空间运动,单位: mm/s
    double ori{0};    // 末端旋转速度,用于笛卡尔空间运动,单位: deg/s
};

// /**
//  * ���ٶ�.
//  * double joint{0};  // �ؽڼ��ٶ�,���ùؽڿռ��˶�, ��λ����/��ƽ����deg/s2��
//  * double tcp{0};    // ĩ���߼��ٶ�,���õѿ����ռ��˶�,��λ������/��ƽ����mm/s2��
//  * double ori{0};    // ĩ����ת�Ǽ��ٶȣ����õѿ����ռ��˶�,��λ����/��ƽ����deg/s2��
//  */
using Acc = Speed;

enum class UserCommand : int {
    SwitchOn         = 1,    // �ϵ�
    SwitchOff        = 2,    // �µ�
    ToReady          = 3,    // �����ֶ�ģʽ
    ToAuto           = 5,    // �����Զ�ģʽ
    AcknowledgeError = 100,  // �������
    ClearWarning=501,//清除警告
};

enum class JogSpeed : int {
    Low  = 1,  // ����
    Mid  = 2,  // ����
    High = 3,  // ����
};

enum class Direction : int {
    Positive = 1,   // ������
    Negative = -1,  // ������
};

struct UserCoor {
    double x;  // �û�����ϵԭ���������������ϵ�� x �����λ��ƫ����, ��λ: mm;
    double y;  // �û�����ϵԭ���������������ϵ�� y �����λ��ƫ����, ��λ: mm;
    double z;  // �û�����ϵԭ���������������ϵ�� z �����λ��ƫ����, ��λ: mm;
    double a;  // �û�����ϵ�������������ϵ x ����ת��ŷ����, ��λ: deg;
    double b;  // �û�����ϵ�������������ϵ y ����ת��ŷ����, ��λ: deg;
    double c;  // �û�����ϵ�������������ϵ z ����ת��ŷ����, ��λ: deg.
};

/**
 * @brief ����ʸ�����԰�װ�Ĺ��߻���������ϵ Otool-XYZ �ϵ�λ���趨.
 *
 */
struct CenterPos {
    double Mx{0};  // ��װ�Ĺ��߻�װ�еĸ��ص����� C ������ϵ Otool-XYZ �� X �����ϵ�ƫ����, ��λ: mm;
    double My{0};  // ��װ�Ĺ��߻�װ�еĸ��ص����� C ������ϵ Otool-XYZ �� Y �����ϵ�ƫ����, ��λ: mm;
    double Mz{0};  // ��װ�Ĺ��߻�װ�еĸ��ص����� C ������ϵ Otool-XYZ �� Z �����ϵ�ƫ����, ��λ: mm.
};

/**
 * @brief �ò������԰�װ�Ĺ��߻������������ϵ Otool-XYZ �����Ĺ�������.
 *const auto&
 */
struct InertiaTensor {
    double Ixx{0};  // ��װ�Ĺ��߻�װ�еĸ��������Ĵ� X  �����ת�Ĺ���, ��λ: kg*mm2;
    double Iyy{0};  // ��װ�Ĺ��߻�װ�еĸ��������Ĵ� Y  �����ת�Ĺ���, ��λ: kg*mm2;
    double Izz{0};  // ��װ�Ĺ��߻�װ�еĸ��������Ĵ� Z  �����ת�Ĺ���, ��λ: kg*mm2;
    double Ixy{0};  // ��װ�Ĺ��߻�װ�еĸ��������Ĵ� XY ���淽��Ĺ�����, ��λ: kg*mm2;
    double Ixz{0};  // ��װ�Ĺ��߻�װ�еĸ��������Ĵ� XZ ���淽��Ĺ�����, ��λ: kg*mm2;
    double Iyz{0};  // ��װ�Ĺ��߻�װ�еĸ��������Ĵ� YZ ���淽��Ĺ�����, ��λ: kg*mm2.
};

/**
 * @brief �����洢������ĩ�˹��ߺ͸���������Ϣ���������ڻ����˶���ѧȫģ�ͼ���.
 *
 */
struct LoadDyn {
    double        M{0};  // ����&���ص�����, ��λ: kg;
    CenterPos     pos;   // �μ� CenterPos;
    InertiaTensor it;    // �μ� InertiaTensor.
};

struct Tool {
    double x{0};  // TCP����ڷ�������ϵ��x�����λ��ƫ����, ��λ: mm;
    double y{0};  // TCP����ڷ�������ϵ��x�����λ��ƫ����, ��λ: mm;
    double z{0};  // TCP����ڷ�������ϵ��z�����λ��ƫ����, ��λ: mm;
    double a{0};  // TCP����ڷ�������ϵ x ����ת��ŷ����, ��λ: deg;
    double b{0};  // TCP����ڷ�������ϵ y ����ת��ŷ����, ��λ: deg;
    double c{0};  // TCP����ڷ�������ϵ z ����ת��ŷ����, ��λ: deg;

    LoadDyn dyn;
};

/// <summary>
/// ��������
/// </summary>
enum class ZoneType : int {
    Fine,      // ������
    Relative,  // ��Թ���
    Absolute,  // ���Թ���
};


struct Zone {
    double per = 0;  // ת��ٷֱ�ֵ����������Ϊ��Թ���ʱ��Ч�����������˶�����
    double dis = 0;  // �ѿ����ռ�ת������С����λ: mm����������Ϊ���Թ���ʱ��Ч��������MovJ
};

enum class MovType : int {
    MovJ,
    MovL,
    MovC,
    MovCircle,
};

enum class PointType {
    Joint, // �ؽ�λ��
    Cart, // �ѿ���λ��
};

struct APos {
    double jntPos[7]{0};  // ������ n �ؽڵ�λ��, ��λ: m(ֱ�ߵ��) �� rad(��ת���);

    static bool toJson(nlohmann::ordered_json& json, const void* value) {
        const auto& var = *(APos*)value;

        json["jntpos1"] = var.jntPos[0];
        json["jntpos2"] = var.jntPos[1];
        json["jntpos3"] = var.jntPos[2];
        json["jntpos4"] = var.jntPos[3];
        json["jntpos5"] = var.jntPos[4];
        json["jntpos6"] = var.jntPos[5];
        json["jntpos7"] = var.jntPos[6];

        return true;
    }

    static bool fromJson(const nlohmann::ordered_json& json, void* value) {
        auto&& var = *(APos*)value;
        try {
            var.jntPos[0] = json["jntpos1"].get<double>();
            var.jntPos[1] = json["jntpos2"].get<double>();
            var.jntPos[2] = json["jntpos3"].get<double>();
            var.jntPos[3] = json["jntpos4"].get<double>();
            var.jntPos[4] = json["jntpos5"].get<double>();
            var.jntPos[5] = json["jntpos6"].get<double>();

            if (json.contains("jntpos7")) {
                var.jntPos[6] = json["jntpos7"].get<double>();
            } else {
                var.jntPos[6] = 0;
            }


        } catch (...) {
            return false;
        }

        return true;
    }
};

/**
 * @brief ����������ͬ�ĵѿ����ռ�λ���£����Ծ߱����ֹؽ�λ����϶�Ӧ
 *        (���������Ķ��).���������ڶ���ռ�Ŀ����Ӧ����̬��������.
 *
 */
struct PosCfg {
    int mode{-1};  // �����˹�������ѡȡ����;

    int cf[7]{0};  // �ؽ� n ����Ƕ����ڵ�����ȡֵ;

    static bool toJson(nlohmann::ordered_json& json, const void* value) {
        const auto& var = *(PosCfg*)value;

        json["mode"] = var.mode;
        json["cf1"]  = var.cf[0];
        json["cf2"]  = var.cf[1];
        json["cf3"]  = var.cf[2];
        json["cf4"]  = var.cf[3];
        json["cf5"]  = var.cf[4];
        json["cf6"]  = var.cf[5];
        json["cf7"]  = var.cf[6];

        return true;
    }

    static bool fromJson(const nlohmann::ordered_json& json, void* value) {
        auto&& var = *(PosCfg*)value;
        try {
            var.mode  = json["mode"];
            var.cf[0] = json["cf1"];
            var.cf[1] = json["cf2"];
            var.cf[2] = json["cf3"];
            var.cf[3] = json["cf4"];
            var.cf[4] = json["cf5"];
            var.cf[5] = json["cf6"];

            if (json.contains("cf7")) {
                var.cf[6] = json["cf7"];
            } else {
                var.cf[6] = 0;
            }

        } catch (...) {
            return false;
        }

        return true;
    }
};

/**
 * @brief TCP ���ڵѿ�������ϵ��λ��.
 *
 */
struct CPos {
    PosCfg cfgData;  // �μ� PosCfg;

    double x{0};  // TCP ���ڲο�����ϵ�� x ���������, ��λ: m;
    double y{0};  // TCP ���ڲο�����ϵ�� y ���������, ��λ: m;
    double z{0};  // TCP ���ڲο�����ϵ�� z ���������, ��λ: m;
    double a{0};  // TCP ������ڲο�����ϵ z ����ת��ŷ����, ��λ: rad;
    double b{0};  // TCP ������ڲο�����ϵ y������ת��ŷ����, ��λ: rad;
    double c{0};  // TCP ������ڲο�����ϵ x������ת��ŷ����, ��λ: rad;

    double e{0};  // 7 ���ɶȻ�������ؽ� (elbow) ��̬;

    static bool toJson(nlohmann::ordered_json& json, const void* value) {
        const auto& var = *(CPos*)value;

        json["x"] = var.x;
        json["y"] = var.y;
        json["z"] = var.z;
        json["a"] = var.a;
        json["b"] = var.b;
        json["c"] = var.c;
        json["e"] = var.e;

        PosCfg::toJson(json["poscfg"], &var.cfgData);

        return true;
    }

    static bool fromJson(const nlohmann::ordered_json& json, void* value) {
        auto&& var = *(CPos*)value;
        try {
            var.x = json["x"].get<double>();
            var.y = json["y"].get<double>();
            var.z = json["z"].get<double>();
            var.a = json["a"].get<double>();
            var.b = json["b"].get<double>();
            var.c = json["c"].get<double>();
            if (json.contains("e")) {
                var.e = json["e"].get<double>();
            } else {
                var.e = 0;
            }

        } catch (...) {
            return false;
        }
        if (!PosCfg::fromJson(json["poscfg"], &var.cfgData)) {
            return false;
        }
        return true;
    }
};

struct Point {
    PointType type{PointType::Joint};
    APos      apos;
    CPos      cpos;
};


struct MovSegment {
    MovType  type;
    Point    targetPosition;
    Point    middlePosition;
    Speed    speed;
    Acc      acc;
    ZoneType zoneType;
    Zone     zone;
};

struct MovJointSegments {
    std::vector<MovSegment> segments;

    void AddMovJ(Point position, Speed speed, Acc acc, ZoneType zoneType, Zone zone) {
        if (zoneType == ZoneType::Absolute) {
            throw std::string("zoneType can not be ABSOLUTE");
        }

        segments.emplace_back();
        MovSegment& segment = segments.back();

        segment.type = MovType::MovJ;
        segment.targetPosition = position;
        segment.speed    = speed;
        segment.acc      = acc;
        segment.zoneType = zoneType;
        segment.zone     = zone;
    }
};

struct MovCartSegments {
    std::vector<MovSegment> segments;

    void AddMovL(Point position, Speed speed, Acc acc, ZoneType zoneType, Zone zone) {
        segments.emplace_back();
        MovSegment& segment = segments.back();

        segment.type = MovType::MovL;
        segment.targetPosition = position;
        segment.speed    = speed;
        segment.acc      = acc;
        segment.zoneType = zoneType;
        segment.zone     = zone;
    }

    void AddMoC(Point middlePosition, Point targetPosition, Speed speed, Acc acc, ZoneType zoneType, Zone zone) {
        segments.emplace_back();
        MovSegment& segment = segments.back();

        segment.type = MovType::MovC;
        segment.middlePosition = middlePosition;
        segment.targetPosition = targetPosition;
        segment.speed    = speed;
        segment.acc      = acc;
        segment.zoneType = zoneType;
        segment.zone     = zone;
    }

    void AddMoCircle(Point middlePosition, Point targetPosition,
                     Speed    speed,
                     Acc      acc,
                     ZoneType zoneType,
                     Zone     zone) {
        segments.emplace_back();
        MovSegment& segment = segments.back();

        segment.type = MovType::MovCircle;
        segment.middlePosition = middlePosition;
        segment.targetPosition = targetPosition;
        segment.speed    = speed;
        segment.acc      = acc;
        segment.zoneType = zoneType;
        segment.zone     = zone;
    }
};

}  // namespace c2

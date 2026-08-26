#pragma once

#include "Request.h"
#include <cstdint>
namespace c2 {

class CodroidApi {
public:
    //创建CodroidApi对象 _request
    CodroidApi(const std::string& host, const std::string& port) : _request(host, port) {
    }

    ~CodroidApi() {

    }

    /**
     * 锟斤拷锟斤拷锟矫伙拷锟斤拷锟斤拷.
     *
     * \param cmd 锟轿匡拷UserCommand说锟斤拷
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    Response sendUserCommand(UserCommand cmd, int timeout = 30) {
        json param     = json::array();
        json paramitem = {
            {"path",  "Robot/Control/command"},
            {"value", (int)cmd               }
        };
        param.push_back(paramitem);

        auto     future = _request.send("common", "setparam", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷取锟斤拷锟斤拷锟斤拷状态.
     *
     * \param timeout
     * \return Response.data 锟斤拷锟斤拷锟斤拷json锟斤拷锟斤拷通锟斤拷 int state = res.data; 锟矫碉拷锟斤拷值锟斤拷
     * 锟斤拷值锟斤拷锟斤拷慰锟紻efine.h锟侥硷拷锟斤拷 enum class RobotState
     * None    = -1,  // 未知
     * Init    = 0,   // 锟斤拷始锟斤拷
     * StandBy = 1,   // 锟斤拷锟铰碉拷
     * Ready   = 2,   // 锟街讹拷模式
     * Rescue  = 3,   // 锟斤拷援模式
     * Auto    = 4,   // 锟皆讹拷模式
     * Error   = 6,   // 锟斤拷锟斤拷锟剿筹拷锟斤拷
     */
    Response getRobotState(int timeout = 5) {
        std::string path   = "Robot/Control/state";
        json        param  = {path};
        auto        future = _request.send("common", "getparam", param, timeout);
        Response    res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"][path];
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷取锟斤拷锟斤拷锟斤拷状态.
     *
     * \param timeout
     * \return Response.data 锟斤拷锟斤拷锟斤拷json锟斤拷锟斤拷通锟斤拷 int state = res.data; 锟矫碉拷锟斤拷值
     * 锟斤拷锟矫碉拷锟斤拷锟斤拷值锟斤拷锟斤拷锟狡伙拷锟斤拷锟斤拷锟斤拷锟斤拷锟?16位锟斤拷锟斤拷位锟斤拷锟斤拷锟斤拷0锟斤拷锟?
     * 锟斤拷0位 锟斤拷停锟斤拷锟铰憋拷志
     * 锟斤拷1位 锟较碉拷锟街?
     * 锟斤拷2位 锟较讹拷锟叫憋拷志(锟斤拷拽示锟斤拷)
     * 锟斤拷3位 锟狡讹拷锟叫憋拷志
     * 锟斤拷4锟斤拷5锟斤拷6锟斤拷7位锟斤拷未使锟斤拷
     * 锟斤拷8位锟斤拷锟斤拷锟斤拷锟街?
     * 锟斤拷9锟斤拷10位锟斤拷00锟斤拷停止锟斤拷01锟斤拷锟斤拷锟叫ｏ拷10锟斤拷锟斤拷停
     * 锟斤拷15位锟斤拷锟斤拷锟斤拷锟街?
     */
    Response getRobotStateFlag(int timeout = 5) {
        json        param = {  };
        auto        future = _request.send("common", "getRobotStates", param, timeout);
        Response    res = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"]["statusFlag"];
            }
            else {
                res.code = ResponseCode::RequestFailed;
                res.msg = data["msg"];
            }
        }
        return res;
    }

    /**
     * 锟斤拷取锟斤拷锟轿?.
     *
     * \param timeout
     * \return Response.data 锟斤拷json锟斤拷锟介，double[6] 锟斤拷锟截斤拷锟斤拷转锟角度ｏ拷锟斤拷位锟饺ｏ拷deg锟斤拷
     */
    Response getPackPosition(int timeout = 5) {
        std::string path   = "Robot/Parameter/Mechanism/packingPosition";
        json        param  = {path};
        auto        future = _request.send("common", "getparam", param, timeout);
        Response    res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"][path];
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 通锟斤拷MovJonit锟剿讹拷锟斤拷Home位.
     *
     * \param speed 锟剿讹拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷
     * \param acc 锟剿讹拷锟斤拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷平锟斤拷
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    Response goHome(double speed = 30, double acc = 30, int timeout = 30) {
        Point point;
        point.apos.jntPos[0] = _homePosition[0];
        point.apos.jntPos[1] = _homePosition[1];
        point.apos.jntPos[2] = _homePosition[2];
        point.apos.jntPos[3] = _homePosition[3];
        point.apos.jntPos[4] = _homePosition[4];
        point.apos.jntPos[5] = _homePosition[5];

        return movJ(point, speed, acc, timeout);
    }

    /**
     * 锟斤拷锟斤拷Home位锟斤拷.
     *
     * \param position double[6] 锟斤拷锟截斤拷锟斤拷转锟角度ｏ拷锟斤拷位锟饺ｏ拷deg锟斤拷
     */
    void setHomePosition(double* position) {
        auto size = sizeof(double) * 6;
        memcpy(_homePosition, position, size);
    }

    /**
     * 通锟斤拷MovJonit锟剿讹拷锟斤拷锟斤拷锟轿?.
     *
     * \param speed 锟剿讹拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷
     * \param acc 锟剿讹拷锟斤拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷平锟斤拷
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    Response goPack(double speed = 60, double acc = 80, int timeout = 30) {
        Point point;
        point.apos.jntPos[0] = _packPosition[0];
        point.apos.jntPos[1] = _packPosition[1];
        point.apos.jntPos[2] = _packPosition[2];
        point.apos.jntPos[3] = _packPosition[3];
        point.apos.jntPos[4] = _packPosition[4];
        point.apos.jntPos[5] = _packPosition[5];

        return movJ(point, speed, acc, timeout);
    }

    /**
     * 锟斤拷锟矫达拷锟轿伙拷锟?.
     *
     * \param position double[6] 锟斤拷锟截斤拷锟斤拷转锟角度ｏ拷锟斤拷位锟饺ｏ拷deg锟斤拷
     */
    void setPackPosition(double* position) {
        auto size = sizeof(double) * 6;
        memcpy(_packPosition, position, size);
    }

    /**
     * 锟截节空硷拷锟剿讹拷.
     *
     * \param position 目锟斤拷位锟矫ｏ拷锟斤拷锟斤拷为double[6]锟斤拷锟斤拷锟斤拷直锟轿拷锟?1~锟斤拷6锟斤拷锟斤拷转锟角度ｏ拷锟斤拷位锟饺ｏ拷deg锟斤拷锟斤拷锟斤拷锟斤拷[0,0,90,0,90,0]
     * \param speed 锟剿讹拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷
     * \param acc 锟剿讹拷锟斤拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷平锟斤拷
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    Response movJ(const Point& position, double speed = 50, double acc = 50, int timeout = 30) {
        json param              = json::object();
        param["type"]           = "movj";
        switch (position.type) {
            case PointType::Joint:
                param["target"]["type"] = "apos";
                APos::toJson(param["target"]["apos"], &position.apos);
                break;

            case PointType::Cart:
                param["target"]["type"] = "cpos";
                CPos::toJson(param["target"]["cpos"], &position.cpos);
                break;

            default:
                break;
        }

        param["speed"] = {
            {"sper",  speed},
            {"stcp",  0    },
            {"sori",  0    },
            {"sexjl", 0    },
            {"sexjr", 0    }
        };

        param["acc"] = {
            {"aper",  acc},
            {"atcp",  0  },
            {"aori",  0  },
            {"aexjl", 0  },
            {"aexjr", 0  }
        };

        auto     future = _request.send("common", "mov", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷喂亟诳占锟斤拷硕锟?.
     *
     * \param segements 锟斤拷锟铰凤拷锟?
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    Response movJointSegments(const MovJointSegments& segments, int timeout = 30) {
        json param      = json::object();
        param["type"]   = "movJoint";
        param["points"] = json::array();

        for (auto& segment : segments.segments) {
            param["points"].emplace_back();
            json& data = param["points"].back();

            switch (segment.targetPosition.type) {
                case PointType::Joint:
                    data["target"]["type"] = "apos";
                    APos::toJson(data["target"]["apos"], &segment.targetPosition.apos);
                    break;

                case PointType::Cart:
                    data["target"]["type"] = "cpos";
                    CPos::toJson(data["target"]["cpos"], &segment.targetPosition.cpos);
                    break;

                default:
                    break;
            }

            data["speed"] = {
                {"sper",  segment.speed.joint},
                {"stcp",  0                  },
                {"sori",  0                  },
                {"sexjl", 0                  },
                {"sexjr", 0                  }
            };

            data["acc"] = {
                {"aper",  segment.acc.joint},
                {"atcp",  0                },
                {"aori",  0                },
                {"aexjl", 0                },
                {"aexjr", 0                }
            };

            switch (segment.zoneType) {
                case ZoneType::Fine:
                    data["zone"]["type"] = "FINE";
                    break;

                case ZoneType::Relative:
                    data["zone"]["type"] = "FINE";
                    break;

                default:
                    throw std::string("Zontype error");
                    break;
            }

            data["zone"]["data"] = {
                {"zper",    segment.zone.per},
                {"zdis",    segment.zone.dis},
                {"zvconst", 0               }
            };
        }

        auto     future = _request.send("common", "movMulti", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟截节空硷拷锟剿讹拷锟斤拷锟斤拷锟饺达拷锟斤拷锟斤拷.
     *
     * \param position 目锟斤拷位锟矫ｏ拷锟斤拷锟斤拷为double[6]锟斤拷锟斤拷锟斤拷直锟轿拷锟?1~锟斤拷6锟斤拷锟斤拷转锟角度ｏ拷锟斤拷位锟饺ｏ拷deg锟斤拷锟斤拷锟斤拷锟斤拷[0,0,90,0,90,0]
     * \param speed 锟剿讹拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷
     * \param acc 锟剿讹拷锟斤拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷平锟斤拷
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    void movJNoResult(const Point& position, double speed = 60, double acc = 80, int timeout = 30) {
        json param              = json::object();
        param["type"]           = "movj";
        switch (position.type) {
            case PointType::Joint:
                param["target"]["type"] = "apos";
                APos::toJson(param["target"]["apos"], &position.apos);
                break;

            case PointType::Cart:
                param["target"]["type"] = "cpos";
                CPos::toJson(param["target"]["cpos"], &position.cpos);
                break;

            default:
                break;
        }

        param["speed"] = {
            {"sper",  speed},
            {"stcp",  0    },
            {"sori",  0    },
            {"sexjl", 0    },
            {"sexjr", 0    }
        };

        param["acc"] = {
            {"aper",  acc},
            {"atcp",  0  },
            {"aori",  0  },
            {"aexjl", 0  },
            {"aexjr", 0  }
        };

        _request.send("common", "mov", param, timeout);
    }

    /**
     * 直锟斤拷锟剿讹拷.
     *
     * \param position 目锟斤拷位锟矫ｏ拷锟斤拷锟斤拷为double[6]锟斤拷锟斤拷示[x,y,z,rx,ry,rz]锟斤拷
     *      (x,y,z)锟斤拷示锟窖匡拷锟斤拷锟秸硷拷锟轿伙拷茫锟斤拷锟轿伙拷锟斤拷祝锟絤m锟斤拷锟斤拷锟斤拷rx,ry,rz锟斤拷锟斤拷示锟斤拷3锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟阶拷嵌龋锟斤拷锟轿伙拷龋锟絛eg锟斤拷
     * \param speed 锟剿讹拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷
     * \param acc 锟剿讹拷锟斤拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷平锟斤拷
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    Response movL(const Point& position, Speed speed = Speed(0, 250, 80), Acc acc = Acc(0, 1200, 320), int timeout = 30) {
    
        json param              = json::object();
        param["type"]           = "movl";
        switch (position.type) {
            case PointType::Joint:
                param["target"]["type"] = "apos";
                APos::toJson(param["target"]["apos"], &position.apos);
                break;

            case PointType::Cart:
                param["target"]["type"] = "cpos";
                CPos::toJson(param["target"]["cpos"], &position.cpos);
                break;

            default:
                break;
        }

        param["speed"] = {
            {"sper",  0        },
            {"stcp",  speed.tcp},
            {"sori",  speed.ori},
            {"sexjl", 0        },
            {"sexjr", 0        }
        };

        param["acc"] = {
            {"aper",  0      },
            {"atcp",  acc.tcp},
            {"aori",  acc.ori},
            {"aexjl", 0      },
            {"aexjr", 0      }
        };

        auto     future = _request.send("common", "mov", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 圆锟斤拷锟剿讹拷.
     *
     * \param targetPosition 目锟斤拷位锟矫ｏ拷锟斤拷锟斤拷为double[6]锟斤拷锟斤拷示[x,y,z,rx,ry,rz]锟斤拷
     *      (x,y,z)锟斤拷示锟窖匡拷锟斤拷锟秸硷拷锟轿伙拷茫锟斤拷锟轿伙拷锟斤拷祝锟絤m锟斤拷锟斤拷锟斤拷rx,ry,rz锟斤拷锟斤拷示锟斤拷3锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟阶拷嵌龋锟斤拷锟轿伙拷龋锟絛eg锟斤拷
     * \param middlePosition 锟叫硷拷位锟矫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷同目锟斤拷位锟斤拷
     * \param speed 锟剿讹拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷
     * \param acc 锟剿讹拷锟斤拷锟劫度ｏ拷锟斤拷位锟斤拷/锟斤拷平锟斤拷
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    Response movC(const Point& middlePosition,
              const Point& targetPosition,
              Speed   speed   = Speed(0, 250, 80),
              Acc     acc     = Acc(0, 1200, 320),
              int     timeout = 30) {
        json param              = json::object();
        param["type"]           = "movc";

        switch (targetPosition.type) {
            case PointType::Joint:
                param["target"]["type"] = "apos";
                APos::toJson(param["target"]["apos"], &targetPosition.apos);
                break;

            case PointType::Cart:
                param["target"]["type"] = "cpos";
                CPos::toJson(param["target"]["cpos"], &targetPosition.cpos);
                break;

            default:
                break;
        }

        switch (middlePosition.type) {
            case PointType::Joint:
                param["middle"]["type"] = "apos";
                APos::toJson(param["middle"]["apos"], &middlePosition.apos);
                break;

            case PointType::Cart:
                param["middle"]["type"] = "cpos";
                CPos::toJson(param["middle"]["cpos"], &middlePosition.cpos);
                break;

            default:
                break;
        }

        param["speed"] = {
            {"sper",  0        },
            {"stcp",  speed.tcp},
            {"sori",  speed.ori},
            {"sexjl", 0        },
            {"sexjr", 0        }
        };

        param["acc"] = {
            {"aper",  0      },
            {"atcp",  acc.tcp},
            {"aori",  acc.ori},
            {"aexjl", 0      },
            {"aexjr", 0      }
        };

        auto     future = _request.send("common", "mov", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷蔚芽锟斤拷锟斤拷占锟斤拷硕锟?.
     *
     * \param segements 锟斤拷锟铰凤拷锟?
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    Response movCartSegments(const MovCartSegments& segments, int timeout = 30) {
        json param      = json::object();
        param["type"]   = "movCart";
        param["points"] = json::array();

        for (auto& segment : segments.segments) {
            // std::cout << "type: " << (int)segment.type << std::endl;
            // std::cout << "ZoneType: " << (int)segment.zoneType << std::endl;

            param["points"].emplace_back();
            json& data = param["points"].back();

            switch (segment.type) {
                case MovType::MovL:
                    data["type"] = "movl";
                    break;

                case MovType::MovC:
                    data["type"] = "movc";
                    break;

                case MovType::MovCircle:
                    data["type"] = "movcircle";
                    break;
                default:
                    throw std::string("MovType error");
                    break;
            }

            switch (segment.targetPosition.type) {
                case PointType::Joint:
                    data["target"]["type"] = "apos";
                    APos::toJson(data["target"]["apos"], &segment.targetPosition.apos);
                    break;

                case PointType::Cart:
                    data["target"]["type"] = "cpos";
                    CPos::toJson(data["target"]["cpos"], &segment.targetPosition.cpos);
                    break;

                default:
                    break;
            }

            if ((segment.type == MovType::MovC) || (segment.type == MovType::MovCircle)) {
                switch (segment.middlePosition.type) {
                    case PointType::Joint:
                        data["middle"]["type"] = "apos";
                        APos::toJson(data["target"]["apos"], &segment.middlePosition.apos);
                        break;

                    case PointType::Cart:
                        data["middle"]["type"] = "cpos";
                        CPos::toJson(data["target"]["cpos"], &segment.middlePosition.cpos);
                        break;

                    default:
                        break;
                }
            }

            data["speed"] = {
                {"sper",  segment.speed.joint},
                {"stcp",  segment.speed.tcp  },
                {"sori",  segment.speed.ori  },
                {"sexjl", 0                  },
                {"sexjr", 0                  }
            };

            data["acc"] = {
                {"aper",  segment.acc.joint},
                {"atcp",  segment.acc.tcp  },
                {"aori",  segment.acc.ori  },
                {"aexjl", 0                },
                {"aexjr", 0                }
            };

            switch (segment.zoneType) {
                case ZoneType::Fine:
                    data["zone"]["type"] = "FINE";
                    break;

                case ZoneType::Relative:
                    data["zone"]["type"] = "RELATIVE";
                    break;

                case ZoneType::Absolute:
                    data["zone"]["type"] = "ABSOLUTE";
                    break;

                default:
                    throw std::string("Zontype error");
                    break;
            }

            data["zone"]["data"] = {
                {"zper",    segment.zone.per},
                {"zdis",    segment.zone.dis},
                {"zvconst", 0               }
            };
        }

        auto     future = _request.send("common", "movMulti", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟矫伙拷锟斤拷锟斤拷停止锟剿讹拷.
     *
     * \param timeout
     * \return
     */
    Response stopMov(int timeout = 30) {
        auto     future = _request.send("common", "stopMov", json::array(), timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟截节点动. 锟斤拷锟斤拷锟斤拷锟酵猴拷锟斤拷锟揭拷锟斤拷值愣拷锟斤拷锟斤拷锟斤拷锟揭拷锟斤拷锟斤拷锟絢eepJog锟接口ｏ拷锟斤拷锟斤拷愣拷锟斤拷锟?1s锟斤拷锟斤拷远锟酵Ｖ?
     *
     * \param jogIndex 锟斤拷要锟姐动锟斤拷锟结，锟斤拷围1~6
     * \param jogSpeed 锟姐动锟劫讹拷
     * \param direction 锟姐动锟斤拷锟斤拷
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    Response jointJog(int jogIndex, JogSpeed jogSpeed, Direction direction, int timeout = 30) {
        json param = json::array();
        json mode  = {
            {"path",  "Robot/Control/jogMode"},
            {"value", 1                      }
        };
        param.push_back(mode);
        json speed = {
            {"path",  "Robot/Control/jogSpeed"      },
            {"value", (int)direction * (int)jogSpeed}
        };
        param.push_back(speed);
        json index = {
            {"path",  "Robot/Control/jogIndex"},
            {"value", jogIndex                }
        };
        param.push_back(index);

        auto     future = _request.send("common", "setparam", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 末锟剿点动. 锟斤拷锟斤拷锟斤拷锟酵猴拷锟斤拷锟揭拷锟斤拷值愣拷锟斤拷锟斤拷锟斤拷锟揭拷锟斤拷锟斤拷锟絢eepJog锟接口ｏ拷锟斤拷锟斤拷愣拷锟斤拷锟?1s锟斤拷锟斤拷远锟酵Ｖ?
     *
     * \param jogIndex 1~6 锟街憋拷锟接? x,y,z,rx,ry,rz
     * \param jogSpeed 锟姐动锟劫讹拷
     * \param direction 锟姐动锟斤拷锟斤拷
     * \param timeout 锟斤拷时锟饺达拷时锟斤拷
     * \return
     */
    Response tcpJog(int jogIndex, JogSpeed jogSpeed, Direction direction, int timeout = 30) {
        json param = json::array();
        json mode  = {
            {"path",  "Robot/Control/jogMode"},
            {"value", 2                      }
        };
        param.push_back(mode);
        json speed = {
            {"path",  "Robot/Control/jogSpeed"      },
            {"value", (int)direction * (int)jogSpeed}
        };
        param.push_back(speed);
        json index = {
            {"path",  "Robot/Control/jogIndex"},
            {"value", jogIndex                }
        };
        param.push_back(index);

        auto     future = _request.send("common", "setparam", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷锟街碉拷前锟姐动. 锟斤拷500锟斤拷锟斤拷锟斤拷锟揭碉拷频锟绞筹拷锟斤拷锟斤拷锟矫接口ｏ拷锟缴憋拷锟街点动锟斤拷锟斤拷
     *
     * \param timeout
     * \return
     */
    Response keepJog(int timeout = 30) {
        json param     = json::array();
        json paramitem = {
            {"path",  "Robot/Control/commandHeart"},
            {"value",
             std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
                 .count()                         }
        };
        param.push_back(paramitem);

        auto     future = _request.send("common", "setparam", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 停止锟姐动
     *
     * \param timeout
     * \return
     */
    Response stopJog(int timeout = 30) {
        json param   = json::array();
        json jogMode = {
            {"path",  "Robot/Control/jogMode"},
            {"value", 0                      }
        };
        param.push_back(jogMode);
        json jogSpeed = {
            {"path",  "Robot/Control/jogSpeed"},
            {"value", 0                       }
        };
        param.push_back(jogSpeed);
        json jogIndex = {
            {"path",  "Robot/Control/jogIndex"},
            {"value", 0                       }
        };
        param.push_back(jogIndex);

        auto     future = _request.send("common", "setparam", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷取锟斤拷前锟截斤拷位锟斤拷
     *
     * \param timeout
     * \return Response.data锟角筹拷锟斤拷为6锟斤拷json锟斤拷锟介，锟斤拷示1~6锟截节碉拷锟斤拷转锟角讹拷
     * 锟斤拷通锟斤拷 vector<double> p = res.data 锟斤拷茫锟?
     * 锟斤拷double p[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]}
     */
    Response getJointPosition(int timeout = 30) {
        auto     future = _request.send("common", "getCurAPos", json::array(), timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = {
                    data["data"]["jntpos1"].get<double>(),
                    data["data"]["jntpos2"].get<double>(),
                    data["data"]["jntpos3"].get<double>(),
                    data["data"]["jntpos4"].get<double>(),
                    data["data"]["jntpos5"].get<double>(),
                    data["data"]["jntpos6"].get<double>(),
                };
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷取锟斤拷前锟窖匡拷锟斤拷锟斤拷锟斤拷位锟斤拷
     *
     * \param timeout
     * \return Response.data锟角筹拷锟斤拷为6锟斤拷json锟斤拷锟介，[x,y,z,rx,ry,rz]
     * 锟斤拷通锟斤拷 vector<double> p = res.data 锟斤拷茫锟?
     * 锟斤拷double p[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]}
     */
    Response getCartPosition(int timeout = 30) {
        auto     future = _request.send("common", "getCurCPos", json::array(), timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = {data["data"]["x"].get<double>(),
                            data["data"]["y"].get<double>(),
                            data["data"]["z"].get<double>(),
                            data["data"]["a"].get<double>(),
                            data["data"]["b"].get<double>(),
                            data["data"]["c"].get<double>()};
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷锟捷笛匡拷锟斤拷位锟矫伙拷取锟截节斤拷
     *
     * \param cpos 锟斤拷要锟斤拷锟侥笛匡拷锟斤拷锟斤拷锟斤拷
     * \param refAPos 执锟斤拷锟斤拷锟绞憋拷牟慰锟斤拷锟?
     * \param timeout
     * \return Response.data锟角筹拷锟斤拷为6锟斤拷json锟斤拷锟介，锟斤拷示1~6锟截节碉拷锟斤拷转锟角讹拷
     * 锟斤拷通锟斤拷 vector<double> p = res.data 锟斤拷茫锟?
     * 锟斤拷double p[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]}
     */
    Response cposToAPos(const CPos& cpos, const APos refAPos, Tool* tool = nullptr, UserCoor* coor = nullptr,  int timeout = 30) {
        json param = json::object();

        CPos::toJson(param["cpos"], &cpos);
        APos::toJson(param["apos"], &refAPos);

        auto     future = _request.send("common", "cpostoapos", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = {
                    data["data"]["apos"]["jntpos1"].get<double>(),
                    data["data"]["apos"]["jntpos2"].get<double>(),
                    data["data"]["apos"]["jntpos3"].get<double>(),
                    data["data"]["apos"]["jntpos4"].get<double>(),
                    data["data"]["apos"]["jntpos5"].get<double>(),
                    data["data"]["apos"]["jntpos6"].get<double>(),
                    data["data"]["apos"]["jntpos7"].get<double>(),
                };
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷丝锟街?.
     *
     * \param port 锟剿口猴拷
     * \param val 锟剿匡拷值锟斤拷只锟斤拷锟斤拷0锟斤拷1
     * \param timeout 锟斤拷锟叫等达拷时锟斤拷
     * \return
     */
    Response setDO(int port, int val, int timeout = 30) {
        json param = {
            {"port", port},
            {"val",  val }
        };

        auto     future = _request.send("common", "setDO", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷丝锟街?.
     *
     * \param port 锟剿口猴拷
     * \param timeout
     * \return Response.data 锟斤拷锟斤拷锟斤拷json锟斤拷锟斤拷通锟斤拷 int val = res.data; 锟矫碉拷锟斤拷值,值为0锟斤拷1
     */
    Response getDI(int port, int timeout = 30) {
        json param = {
            {"port", port}
        };
        auto     future = _request.send("common", "getDI", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"];
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷丝锟街?.
     *
     * \param port 锟剿口猴拷
     * \param val 锟剿匡拷值锟斤拷只锟斤拷锟斤拷0锟斤拷1
     * \param timeout 锟斤拷锟叫等达拷时锟斤拷
     * \return
     */
    Response setDOGroup(int startPort, int endPort, int val, int timeout = 30) {
        json param = {
            {"startPort", startPort},
            {"endPort",   endPort  },
            {"val",       val      }
        };

        auto     future = _request.send("common", "setDOGroup", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷丝锟街?.
     *
     * \param port 锟剿口猴拷
     * \param timeout
     * \return Response.data 锟斤拷锟斤拷锟斤拷json锟斤拷锟斤拷通锟斤拷 int val = res.data; 锟矫碉拷锟斤拷值,值为锟剿口从低碉拷锟竭讹拷应锟侥讹拷锟斤拷锟斤拷锟斤拷锟斤拷十锟斤拷锟斤拷值
     */
    Response getDIGroup(int startPort, int endPort, int timeout = 30) {
        json param = {
            {"startPort", startPort},
            {"endPort",   endPort  }
        };
        auto     future = _request.send("common", "getDIGroup", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"];
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷锟矫碉拷前锟斤拷锟斤拷系.
     *
     * \param varName 锟斤拷锟斤拷系锟斤拷锟斤拷锟斤拷
     *      注: 锟矫憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷UI锟斤拷锟斤拷锟斤拷示锟斤拷锟矫憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷斜锟斤拷锟斤拷锟斤拷也锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷路锟斤拷谋锟斤拷锟揭伙拷拢锟斤拷员锟斤拷锟斤拷锟斤拷一些锟斤拷锟斤拷
     * \param coor 锟斤拷锟斤拷系锟斤拷锟斤拷
     * \param timeout
     * \return
     */
    Response setCurrentCoor(const std::string& varName, UserCoor coor, int timeout = 30) {
        json param;

        param["key"]   = varName;
        json& coorNode = param["value"]["USERCOOR"];
        coorNode["x"]  = coor.x;
        coorNode["y"]  = coor.y;
        coorNode["z"]  = coor.z;
        coorNode["a"]  = coor.a;
        coorNode["b"]  = coor.b;
        coorNode["c"]  = coor.c;

        auto     future = _request.send("common", "setcurusercoor", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷锟矫碉拷前锟斤拷锟斤拷.
     *
     * \param varName 锟斤拷锟竭憋拷锟斤拷锟斤拷
     *      注: 锟矫憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷UI锟斤拷锟斤拷锟斤拷示锟斤拷锟矫憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷斜锟斤拷锟斤拷锟斤拷也锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷路锟斤拷谋锟斤拷锟揭伙拷拢锟斤拷员锟斤拷锟斤拷锟斤拷一些锟斤拷锟斤拷
     * \param tool 锟斤拷锟竭诧拷锟斤拷
     * \param timeout
     * \return
     */
    Response setCurrentTool(const std::string& varName, Tool tool, int timeout = 30) {
        json param;

        param["key"]   = varName;
        json& toolNode = param["value"]["TOOL"];
        toolNode["x"]  = tool.x;
        toolNode["y"]  = tool.y;
        toolNode["z"]  = tool.z;
        toolNode["a"]  = tool.a;
        toolNode["b"]  = tool.b;
        toolNode["c"]  = tool.c;

        LoadDyn& dyn     = tool.dyn;
        json&    dynNode = toolNode["dyn"];
        dynNode["m"]     = dyn.M;

        json& posNode = dynNode["pos"];
        posNode["mx"] = dyn.pos.Mx;
        posNode["my"] = dyn.pos.My;
        posNode["mz"] = dyn.pos.Mz;

        json& tensorNode  = dynNode["tensor"];
        tensorNode["ixx"] = dyn.it.Ixx;
        tensorNode["ixy"] = dyn.it.Ixy;
        tensorNode["ixz"] = dyn.it.Ixz;
        tensorNode["iyy"] = dyn.it.Iyy;
        tensorNode["iyz"] = dyn.it.Iyz;
        tensorNode["izz"] = dyn.it.Izz;

        auto     future = _request.send("common", "setcurtool", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷锟矫碉拷前锟斤拷锟斤拷.
     *
     * \param varName 锟斤拷锟截憋拷锟斤拷锟斤拷
     *      注: 锟矫憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷UI锟斤拷锟斤拷锟斤拷示锟斤拷锟矫憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷斜锟斤拷锟斤拷锟斤拷也锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷路锟斤拷谋锟斤拷锟揭伙拷拢锟斤拷员锟斤拷锟斤拷锟斤拷一些锟斤拷锟斤拷
     * \param dyn 锟斤拷锟截诧拷锟斤拷
     * \param timeout
     * \return
     */
    Response setCurrentPayload(const std::string& varName, LoadDyn dyn, int timeout = 30) {
        json param;

        param["key"]  = varName;
        json& dynNode = param["value"]["PAYLOAD"]["dyn"];
        dynNode["m"]  = dyn.M;

        json& posNode = dynNode["pos"];
        posNode["mx"] = dyn.pos.Mx;
        posNode["my"] = dyn.pos.My;
        posNode["mz"] = dyn.pos.Mz;

        json& tensorNode  = dynNode["tensor"];
        tensorNode["ixx"] = dyn.it.Ixx;
        tensorNode["ixy"] = dyn.it.Ixy;
        tensorNode["ixz"] = dyn.it.Ixz;
        tensorNode["iyy"] = dyn.it.Iyy;
        tensorNode["iyz"] = dyn.it.Iyz;
        tensorNode["izz"] = dyn.it.Izz;

        auto     future = _request.send("common", "setcurpayload", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷取锟斤拷锟斤拷状态
     *
     * \param timeout
     * \return std::string
     * IDLE: 锟斤拷锟叫ｏ拷没锟叫癸拷锟斤拷锟斤拷锟斤拷锟斤拷
     * LOADING: 锟斤拷锟节硷拷锟截癸拷锟斤拷
     * RUNNING: 锟斤拷锟斤拷锟斤拷锟叫癸拷锟斤拷
     * PAUSE: 锟斤拷锟斤拷锟斤拷锟斤拷停
     * ERROR: 锟斤拷锟斤拷锟斤拷锟叫筹拷锟斤拷
     */
    Response getProjectState(int timeout = 30) {
        json     param  = {};
        auto     future = _request.send("projexecute", "getProjectState", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() == 0) {
                res.data = data["data"];
            } else {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷锟斤拷指锟斤拷锟斤拷锟斤拷
     *
     * \param projectName 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷使锟斤拷锟斤拷锟斤拷锟街凤拷锟斤拷
     * \param taskName 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷使锟斤拷锟斤拷锟斤拷锟街凤拷锟斤拷
     * \param label 锟斤拷签锟斤拷锟斤拷锟斤拷锟斤拷使锟斤拷锟斤拷锟斤拷锟街凤拷锟斤拷
     * \param timeout
     * \return
     */
    Response runProject(const std::string& projectName,
                        const std::string& taskName = "main1",
                        int                timeout  = 30) {
        json param = {
            {"projectName", projectName},
            {"taskName",    taskName   }
        };
        auto     future = _request.send("projexecute", "run", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷锟斤拷锟斤拷锟津开的癸拷锟斤拷
     *
     * \param timeout
     * \return
     */
    Response runLastProject(int timeout = 30) {
        json     param  = {};
        auto     future = _request.send("projexecute", "runLast", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 停止锟斤拷锟叫癸拷锟斤拷
     *
     * \param timeout
     * \return
     */
    Response stopProject(int timeout = 30) {
        json     param  = {};
        auto     future = _request.send("projexecute", "stop", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷停锟斤拷锟斤拷锟斤拷锟斤拷
     *
     * \param timeout
     * \return
     */
    Response pauseProject(int timeout = 30) {
        json     param  = {};
        auto     future = _request.send("projexecute", "pause", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟街革拷锟斤拷锟斤拷锟斤拷锟斤拷
     *
     * \param timeout
     * \return
     */
    Response resumeProject(int timeout = 30) {
        json     param  = {};
        auto     future = _request.send("projexecute", "resume", param, timeout);
        Response res    = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
			if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg  = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷始锟斤拷RS485锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
     *
     * \param type 指锟斤拷锟接匡拷, "RS485": keba锟斤拷锟斤拷锟斤拷RS485锟接口ｏ拷"EC2RS485": 锟斤拷械锟斤拷末锟斤拷RS485锟接匡拷
     * \param baudrate 锟斤拷锟斤拷锟斤拷
     * \param stopBit 停止位, 0: 1停止位; 1: 1 1/2停止位; 2: 2停止位
     * \param parity 锟斤拷偶校锟斤拷 0: 锟斤拷; 1: 锟斤拷校锟斤拷 2: 偶校锟斤拷
     * \param dataBit 锟斤拷锟斤拷位锟斤拷注锟斤拷锟斤拷械锟斤拷末锟斤拷RS485锟接口碉拷锟斤拷锟斤拷位锟教讹拷为8锟斤拷锟斤拷锟缴革拷锟斤拷
	 * \param timeout 锟饺达拷锟斤拷时时锟戒，锟斤拷位锟斤拷
	 * \return
     */
    Response rs485Init(std::string type, int baudrate, int stopBit = 0, int parity = 0, int dataBit = 8, int timeout = 30) {
        json param = {
            {"baudrate", baudrate},
            {"stopBit", stopBit},
            {"parity", parity},
            {"dataBit", dataBit}
        };
        auto future = _request.send(type, "init", param, timeout);
        Response res = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg = data["msg"];
            }
        }

        return res;
    }

    /**
     * 通锟斤拷RS485锟斤拷锟斤拷锟斤拷锟斤拷
     *
     * \param type 指锟斤拷锟接匡拷, "RS485": keba锟斤拷锟斤拷锟斤拷RS485锟接口ｏ拷"EC2RS485": 锟斤拷械锟斤拷末锟斤拷RS485锟接匡拷
     * \param msg 锟斤拷锟酵碉拷锟斤拷锟捷ｏ拷锟叫憋拷锟叫帮拷顺锟斤拷锟绞久匡拷锟斤拷纸诘锟斤拷锟街?
     * \param timeout 锟饺达拷锟斤拷时时锟戒，锟斤拷位锟斤拷
     * \return
     */
    Response rs485Write(std::string type, std::vector<uint8_t> msg, int timeout = 30) {
        json dataArray = json::array();
        std::for_each(msg.cbegin(), msg.cend(), [&](const uint8_t& data) {
            dataArray.push_back(data);
            });
        auto future = _request.send(type, "write", dataArray, timeout);
        Response res = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷斩锟斤拷锟斤拷锟?
     *
     * \param type 指锟斤拷锟接匡拷, "RS485": keba锟斤拷锟斤拷锟斤拷RS485锟接口ｏ拷"EC2RS485": 锟斤拷械锟斤拷末锟斤拷RS485锟接匡拷
     * \param timeout 锟饺达拷锟斤拷时时锟戒，锟斤拷位锟斤拷
     * \return
     */
    Response rs485FlushReadBuffer(std::string type, int timeout = 30) {
        json param = {

        };
        auto future = _request.send(type, "flushReadBuffer", param, timeout);
        Response res = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg = data["msg"];
            }
        }

        return res;
    }

    /**
     * 锟斤拷取RS485锟秸碉拷锟斤拷锟斤拷锟斤拷
     *
     * \param type 指锟斤拷锟接匡拷, "RS485": keba锟斤拷锟斤拷锟斤拷RS485锟接口ｏ拷"EC2RS485": 锟斤拷械锟斤拷末锟斤拷RS485锟接匡拷
     * \param length 锟斤拷取锟斤拷锟街斤拷锟斤拷
     * \param timeout 锟饺达拷锟斤拷时时锟戒，锟斤拷位锟斤拷
     * \return
     */
    Response rs485Read(std::string type, int length, int timeout = 30) {
        json param = {
            {"length", length},
            {"timeout", timeout * 1000000},
        };
        auto future = _request.send(type, "read", param, timeout);
        Response res = future.get();
        if (res.code == ResponseCode::OK) {
            json data = res.data;
            if (data["code"].get<int>() != 0) {
                res.code = ResponseCode::RequestFailed;
                res.msg = data["msg"];
            }
            else {
                int dataArrayLength = data["data"].size();
                std::vector<int32_t> dataArray;
                dataArray.reserve(dataArrayLength);
                for (int i = 0; i < dataArrayLength; ++i) {
                    dataArray.push_back(data["data"].at(i).get<int32_t>());
                }

                res.data = dataArray;
            }
        }

        return res;
    }

private:
    Request _request;

    double _homePosition[6] = {0, 0, 90, 0, 90, 0};
    double _packPosition[6] = {89, 0, 148.3, -31.7, 181, 180};
};
}  // namespace c2

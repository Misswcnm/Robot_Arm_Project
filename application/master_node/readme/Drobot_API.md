# D-robot API

[TOC]

# 更新日志


- 版本:1.1.2
  - ADD:
    - 点列表增加了ID码
    - 添加了获取任务状态中当前任务信息

- 版本:1.1.1
  - ADD:
    - 当前地图信息增加ID码
    - 增加编辑地图功能接口
  - REMOVE:
    - 关闭获取路径的接口,减小运行时CPU的开销

- 版本:1.1.0
  - CHANGE: 
    - 原有的:旋转接口(rotate_appoint_angle,check_rotate_finished,stop_rotate)进行了改变
  - ADD:
    - 增加了定距离控制
    - 增加了获取地图信息中的ID字段
  - 增加了继续建图功能
  
- 版本:1.0.2(2019-11-30)
- 获取地图列表中新增图片pngMD5码,用于验证远程图片与本地图片的一致性


- 版本:1.0.1(2019-10-17)
- 更新任务管理部分,使用新的任务管理
- 将初始点管理合并入点管理中

- 版本:1.0.0(2019-9-16)
- 初步更新所有url
# 简介
url地址前缀为:目标IP:1819/d-robot/ 如:192.168.1.15:1819/d-robot/xxx

# 1.建图功能

## 开始建图 
  状态允许[Idle]
  - GET请求  mapping/start_scan_map
  - response:{"errorCode": "", "msg": "successed", "data": "", "successed": true}
## 取消建图不保存地图
  状态允许[Mapping,RestartMapping]
  - GET请求  mapping/cancel_scan_map
  - response:{"errorCode": "", "msg": "successed", "data": "", "successed": true}
## 获取建图时地图数据
  状态允许[Mapping,RestartMapping]
  - GET请求: mapping/scan_map_png
  - response: 
    data数据中存放着栅格为障碍的坐标序列号.计算为(x*gridWidth+y),可以根据mapInfo中的信息反推得到坐标的x,y坐标
  ```
    {"data": [359, 360, 361, 362, 412, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 423, 425, 426, 476, 489, 490, 540, 554, 604, 618, 668, 682, 732, 746, 796, 860, 876, 924, 940, 988, 1004, 1052, 1053, 1068, 1116, 1117, 1130, 1131, 1181, 1194, 1195, 1245, 1259, 1309, 1323, 1373, 1387, 1437, 1451, 1501, 1565, 2716, 2717, 2718, 2719, 2720, 2721, 2722, 2723, 2724, 2725, 2758, 2767, 2768, 2769, 2770, 2771, 2772, 2773, 2774, 2775, 2776, 2777, 2778, 2779, 2780, 2781, 2789, 2822, 2823, 2824, 2825, 2826, 2827, 2828, 2829, 2830, 2831, 2832, 2833, 2917, 2981, 2996, 3060, 3109, 3124, 3189, 3238, 3253, 3317, 3366, 3381, 3445, 3494, 3573, 3622, 3637, 3765, 3814, 3829, 3958, 4071, 4086, 4214, 4263, 4342, 4534, 4583, 4726, 4904, 4968, 5352, 5865, 5946, 5947, 6004, 6005, 6006, 6007, 6008, 6009, 6058, 6059, 6060, 6061, 6062, 6063, 6064, 6065, 6066, 6067, 6068], "mapInfo": {"gridHeight": 100, "gridWidth": 64}}
  ```
## 结束并保存地图,传入地图名称
  状态允许[Mapping,RestartMapping]
  - GET请求  mapping/stop_scan_map?map_name=chartest&?angle=0.0
  - response: {"errorCode": "", "msg": "successed", "data": "", "successed": true}
    在url中填入`map_name`变量,用于给当前地图选择名称,`angle`可选择对地图进行一定旋转后再保存
## 继续建图
  状态允许[RuningTask](从RuningTask进入RestartMapping状态)
  - GET请求  mapping/restart_scan_map
  - response:{"errorCode": "", "msg": "successed", "data": "", "successed": true}
    该接口要求步骤流程:
    1.开启导航
    2.加载地图
    3.定位
    4.调用该接口进行继续建图
    5.进入建图界面
    6.保存地图

# 2.地图管理功能

## 获取地图列表和地图信息
  状态允许[Idle,RuningTask]
  - GET请求  map_manage/get_map_info
  - response:
  ```
  {
    "errorCode": "",
    "msg": "successed",
    "data": {
      "maps": [
        {
          "mapId": "3f3e34a7-903b-4da5-9e3c-70e20ddf112f",
          "originY": -12.977645,
          "gridWidth": 1351,
          "originX": -7.837482,
          "mapname": "3f3e34a7-903b-4da5-9e3c-70e20ddf112f",
          "gridHeight": 927,
          "pngMD5": "79944abd9a5321f2354470ee2c6d0601",
          "resolution": 0.05
        },
        {
          "mapId": "3afcb5b1-e802-47ca-b431-be36c1c1f227",
          "originY": -8.049997,
          "gridWidth": 108,
          "originX": -2.45001,
          "mapname": "Jun",
          "gridHeight": 227,
          "pngMD5": "1cf2690922af0b1059601e022b15e8d2",
          "resolution": 0.05
        },
        {
          "mapId": "7a252768-b016-4d59-ba8b-2b48e3e5d10b",
          "originY": -3.706344,
          "gridWidth": 451,
          "originX": -13.101901,
          "mapname": "Restared",
          "gridHeight": 174,
          "pngMD5": "bbd3e37cc293a6b8ebf81d6dac760235",
          "resolution": 0.05
        }
      ]
    },
    "successed": true
  }
  ```
  该接口可以获取当前所有地图的名字与地图信息

## 获取指定名称地图png图片
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/map_manage/maps_pngs?map_name=?
  - response:地图图片,png格式

## 切换地图
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/map_manage/load_map?map_name=?
  - response: {"errorCode": "", "msg": "", "data": {}, "successed": true}

## 删除地图
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/map_manage/delete_map?map_name=?
  - response: {"errorCode": "", "msg": "", "data": {}, "successed": true}
## 重命名地图
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/map_manage/rename_map?origin_map_name=?&new_map_name=?
  - response: {"errorCode": "", "msg": "", "data": {}, "successed": true}
## 获取当前地图信息
  模块开启默认会加载上次地图,可以通过该接口获取当前地图的信息,在建图时名称为变为scanning_map
  状态允许[Idle,RuningTask]
  - GET请求 /d-robot/map_manage/current_map_info
  - response:
  ```
  {
    "errorCode": "",
    "msg": "successed",
    "data": {
      "gridHeight": 195,
      "gridWidth": 433,
      "name": "localmap"
    },
    "successed": true
  }
  ```
## 下载地图
  状态允许[Idle,RuningTask]
  map_name:需要下载的地图名称
  - GET请求 /d-robot/map_manage/download_map?map_name=localmap
  - response:
    在url中输入map_name,即对应的地图名称,获取其一个zip打包的压缩文件包,里面存放着相关的地图文件
## 上传地图
  状态允许[Idle,RuningTask]
  - POST请求 /d-robot/map_manage/upload_map?map_name=localmap
  - response:
    上传一个zip打包的压缩文件包,应保持下载的地图文件目录


# 4.地图编辑相关功能
(该功能被限定于Idle下才可使用)
以下操作都是更新当前地图,故在操作前,请先加载选择地图
## 更新虚拟墙
  状态允许[Idle]
  - POST请求  /d-robot/navigate/add_virtual_obstacles
    circles : 圆形,内部参数为 `center`:圆心 `radius`半径
    lines:直线,内部参数为`start`开始点,`end`结束点
    polygons:闭合多边形区域 传入点数组,虚拟墙会自动将多点围成闭合多边形区域禁止进入
    polylines: 多边形
    rectangles:矩形
    request:传入要保存的json文件
    ```
    '{"obstacles":{"circles":[{"center":{"x":109,"y":76},"radius":60.108235708594876}],"lines":[{"start":{"x":449,"y":806},"end":{"x":560,"y":802}}],"polygons":[[{"x":476,"y":672},{"x":489,"y":748},{"x":522,"y":719},{"x":554,"y":765},{"x":569,"y":676}]],"polylines":[[{"x":476,"y":672},{"x":489,"y":748},{"x":522,"y":719},{"x":554,"y":765},{"x":569,"y":676}]],"rectangles":[{"end":{"x":613,"y":362},"start":{"x":529,"y":281}}]}}'
    ```
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}
## 获取虚拟墙
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/navigate/get_virtual_obstacles
  - response:
    ```
    '{"obstacles":{"circles":[{"center":{"x":109,"y":76},"radius":60.108235708594876}],"lines":[{"start":{"x":449,"y":806},"end":{"x":560,"y":802}}],"polygons":[[{"x":476,"y":672},{"x":489,"y":748},{"x":522,"y":719},{"x":554,"y":765},{"x":569,"y":676}]],"polylines":[[{"x":476,"y":672},{"x":489,"y":748},{"x":522,"y":719},{"x":554,"y":765},{"x":569,"y":676}]],"rectangles":[{"end":{"x":613,"y":362},"start":{"x":529,"y":281}}]}}'
    ```
## 编辑地图
  状态允许[Idle]
    operation_type:  输入四种参数:restore(恢复初始状态),add(修改为墙壁障碍物(黑色墙壁)),remove(修改为空白区域),unknown(修改为未知区域)
    - POST请求  /d-robot/map_manage/edit_map?operation_type=restore"
    - request:
    ```
    {
        "obstacles": {
            "polygons": [
                [
                    {
                        "x": 50,
                        "y": 20
                    },
                    {
                        "x": 50,
                        "y": 100
                    },
                    {
                        "x": 100,
                        "y": 50
                    }
                ]
            ]
        }
    }
    ```
    - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}
# 5.导航相关功能
原有的导航到点,与跟随路径合并至任务执行状态
## 开启导航
  (由Idle状态进入RunningTask状态)
  开始启动导航相关程序 进入执行任务状态
  - GET请求  /d-robot/navigate/start_navigation
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}

## 关闭导航
    (由RunningTask状态进入Idle状态)
    关闭导航相关程序,进行空闲状态
    - GET请求 /d-robot/navigate/cancel_navigation
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}

# 6.路径相关
## 开始录制路径
  状态允许[RuningTask]
  - GET 请求/d-robot/path_manage/start_record_path
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}
## 保存录制的路径
  状态允许[RuningTask]
  - GET请求  /d-robot/path_manage/save_record_path?path_name=?
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}

## 取消录制路径
  状态允许[RuningTask]
  - GET 请求  /d-robot/path_manage/cancel_record_path
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}

## 删除路径
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/path_manage/delete_path?path_name=?
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}
## 获取当前地图路径信息
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/path_manage/get_path_list
  - response:
    ```
    {
        "errorCode": "",
        "msg": "",
        "data": {
            "paths": [
                {
                    "mapname": "yuanqu",
                    "pointCount": 22,
                    "createtime": "2019-04-20 18:04:51",
                    "name": "test123",
                    "filename": "test123.csv"
                },
                {
                    "mapname": "yuanqu",
                    "pointCount": 1,
                    "createtime": "2019-04-22 10:13:37",
                    "name": "test234",
                    "filename": "test234.csv"
                }
            ]
        },
        "successed": true
    }
    ```

## 获取路径
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/path_manage/get_path?path_name=?
    指定路径名字获取路径点
  - response:
    ```
    {
    "errorCode": "",
    "msg": "successed",
    "data": {
        "path": {
        "name": "MANUAL_test",
        "pointCount": 18,
        "filename": "MANUAL_test",
        "mapName": "localmap",
        "length": "300.0",  #暂未启用
        "createtime": "2019-05-19" #暂未启用
        },
        "data": [
        {
            "y": 1,
            "x": 1
        },
        {
            "y": 1,
            "x": 1
        },
        {
            "y": 1,
            "x": 1
        },
        {
            "y": 2,
            "x": 2
        },
        {
            "y": 3,
            "x": 3
        },
        {
            "y": 3,
            "x": 3
        },
        {
            "y": 4,
            "x": 4
        },
        {
            "y": 5,
            "x": 5
        },
        {
            "y": 5,
            "x": 5
        },
        {
            "y": 6,
            "x": 6
        },
        {
            "y": 7,
            "x": 7
        },
        {
            "y": 8,
            "x": 8
        },
        {
            "y": 8,
            "x": 8
        },
        {
            "y": 9,
            "x": 9
        },
        {
            "y": 10,
            "x": 10
        },
        {
            "y": 10,
            "x": 10
        },
        {
            "y": 11,
            "x": 11
        },
        {
            "y": 11,
            "x": 11
        }
        ],
        "mapInfo": {
        "gridHeight": 195,
        "gridWidth": 433
        }
    },
    "successed": true
    }
    ```

## 生成手绘路径(即将废弃,请勿使用)
  - POST请求 /d-robot/path_manage/add_path
  - request:
      `name`:为要生成的路径名称,对于手绘路径程序在生成时会加上前缀MANUAL_XX
      `keyPoint`:为关键点,路径会将多个关键点连成直线作为路径 
                `pointName`:关键点名,一条路径中关键点名不要重名
                `pointAction`:在该点执行的动作 :
                              `type`: [`pause`: 暂停,输入param中应有`millisecond`,暂停毫秒时间] //目前只有暂停 待拓展
                `gridPose`:关键点的坐标

    ```
    {
        "name": "test",
        "keyPoint": [
            {
                "pointName": "0",
                "pointAction": [
                    {
                        "type": "pause",
                        "param": {
                            "millisecond": 500
                        }
                    }
                ],
                "gridPose": {
                    "x": 0,
                    "y": 0
                }
            },
            {
                "pointName": "1",
                "pointAction": [],
                "gridPose": {
                    "x": 10,
                    "y": 10
                }
            }
        ]
    }
    ```
## 生成手绘路径
  状态允许[Idle,RuningTask]
  - POST请求 /d-robot/path_manage/generate_path
  ```
  {
      "name": "testpath",
      "points": [
          {
              "name": "p1",
              "gridPosition": {
                  "x": 10,
                  "y": 20
              },
              "actions": [
                  {
                      "type": "pause",
                      "param": {
                          "millisecond": 500
                      }
                  }
              ]
          },
          {
              "name": "p2",
              "gridPosition": {
                  "x": 15,
                  "y": 20
              },
              "actions": []
          },
          {
              "name": "p3",
              "gridPosition": {
                  "x": 15,
                  "y": 20
              },
              "actions": []
          }
      ],
      "lines": [
          {
              "name": "0_1",
              "start": "p0",
              "end": "p1",
              "radius": 0
          },
          {
              "name": "1_2",
              "start": "p1",
              "end": "p2",
              "radius": 0
          },
          {
              "name": "2_3",
              "start": "p2",
              "end": "p3",
              "radius": 0
          }
      ]
  }
  ```
## 验证两点之间是否可以生成线段
  状态允许[Idle,RuningTask]
  - POST请求 /d-robot/path_manage/verify_path_line
  - request:
  ```
  {
      "start": {
          "x": 157,
          "y": 117
      },
      "end": {
          "x": 136,
          "y": 78
      },
      "radius": 24.113682
  }
  ```
## 获取手绘路径列表
  状态允许[Idle,RuningTask]
  - GET请求 /d-robot/path_manage/get_manual_path
  - response:
  ```
  {
      "errorCode": "",
      "msg": "successed",
      "data": {
          "paths": [
              {
                  "points": [
                      {
                          "gridPosition": {
                              "y": 64,
                              "x": 45
                          },
                          "actions": [],
                          "name": "0"
                      },
                      {
                          "gridPosition": {
                              "y": 67,
                              "x": 104
                          },
                          "actions": [],
                          "name": "1"
                      },
                      {
                          "gridPosition": {
                              "y": 71,
                              "x": 165
                          },
                          "actions": [],
                          "name": "2"
                      }
                  ],
                  "lines": [
                      {
                          "radius": 0,
                          "end": "1",
                          "name": "0_1",
                          "start": "0"
                      },
                      {
                          "radius": 0,
                          "end": "2",
                          "name": "1_2",
                          "start": "1"
                      }
                  ],
                  "name": "Aaa"
              },
              {
                  "points": [
                      {
                          "gridPosition": {
                              "y": 70,
                              "x": 44
                          },
                          "actions": [],
                          "name": "0"
                      },
                      {
                          "gridPosition": {
                              "y": 73,
                              "x": 142
                          },
                          "actions": [],
                          "name": "1"
                      }
                  ],
                  "lines": [
                      {
                          "radius": 0,
                          "end": "1",
                          "name": "0_1",
                          "start": "0"
                      }
                  ],
                  "name": "Bbbxx"
              }
          ]
      },
      "successed": true
  }
  ```
## 更新手绘路径
  状态允许[Idle,RuningTask]
  - POST请求 /d-robot/path_manage/update_path
  ```
  {
      "name": "testpath",
      "points": [
          {
              "name": "p1",
              "gridPosition": {
                  "x": 10,
                  "y": 20
              },
              "actions": [
                  {
                      "type": "pause",
                      "param": {
                          "millisecond": 500
                      }
                  }
              ]
          },
          {
              "name": "p2",
              "gridPosition": {
                  "x": 15,
                  "y": 20
              },
              "actions": []
          },
          {
              "name": "p3",
              "gridPosition": {
                  "x": 15,
                  "y": 20
              },
              "actions": []
          }
      ],
      "lines": [
          {
              "name": "0_1",
              "start": "p0",
              "end": "p1",
              "radius": 0
          },
          {
              "name": "1_2",
              "start": "p1",
              "end": "p2",
              "radius": 0
          },
          {
              "name": "2_3",
              "start": "p2",
              "end": "p3",
              "radius": 0
          }
      ]
  }
  ```
# 7.点管理
## 添加一个点位置
  状态允许[RuningTask]
  - POST请求 /d-robot/pose_manage/add_pose
- request:
  ```json
  {
      "position": {
          "point": {
              "x": 10,
              "y": 23.4
          },
          "angle": 1.57
      },
      "name": "testpoint",
      "type": 0
  }
  ```
## 添加机器人当前所在位置点
  状态允许[RuningTask]
- GET请求  /d-robot/pose_manage/add_cur_pose?pose_name=?
- response:
  ```
  {
    "errorCode": "", 
    "msg": "", 
    "data": {}, 
    "successed": true
  }
  ```

## 编辑点
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/pose_manage/edit_pose?pose_name=?
  - request:
    ```
    {
        "position":{
            "point":{
                "x": 10.0, //单位是米
                "y": 23.4  //单位是米
            },
            "angle":  1.57  //单位是弧度
        }
    }
    ```
## 获得点信息
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/pose_manage/get_pose?pose_name=""
    若输入pose_name则返回单个pose信息，若pose_name为空则返回所有pose信息
  - response:
    ```json
    {
      "PoseList": [
        {
          "angle": -0.8488051295280457,
          "name": "111",
          "gridY": 64,
          "gridX": 96,
          "poseId": "744b4529-6d7d-49d2-81b4-b6e261ae70ea",
          "world": {
            "position": {
              "y": 1.6130320476837159,
              "x": -9.493859928474427,
              "z": 0
            },
            "orientation": {
              "y": 0,
              "x": 0,
              "z": -0.4117764215508053,
              "w": 0.9112849053149149
            }
          },
          "type": 2
        },
        {
          "angle": -2.8714394569396973,
          "name": "3222",
          "gridY": 64,
          "gridX": 156,
          "poseId": "41568fdb-c18a-4ec7-9dce-7e21efd07894",
          "world": {
            "position": {
              "y": 1.6130320476837159,
              "x": -6.493859883770943,
              "z": 0
            },
            "orientation": {
              "y": 0,
              "x": 0,
              "z": -0.9908910189052078,
              "w": 0.1346662119946911
            }
          },
          "type": 2
        },
        {
          "angle": -0.45881673693656927,
          "name": "333",
          "gridY": 54,
          "gridX": 128,
          "poseId": "588cf732-e1b3-416c-8ec4-3837ea971148",
          "world": {
            "position": {
              "y": 1.1130320402331353,
              "x": -7.893859904632569,
              "z": 0
            },
            "orientation": {
              "y": 0,
              "x": 0,
              "z": -0.227401431928895,
              "w": 0.9738011032837702
            }
          },
          "type": 2
        }
      ],
      "mapInfo": {
        "gridHeight": 195,
        "gridWidth": 433
      },
      "successed": true
    }
    ```
## 删除点
  状态允许[Idle,RuningTask]
  - GET请求  /d-robot/pose_manage/delete_pose?pose_name=
  - response:
    ```
    {
      "errorCode": "", 
      "msg": "", 
      "data": {}, 
      "successed": true
    }
    ```
## 自定义初始化
  (直接传入坐标点进行位置初始化,不保存为点)
  状态允许[RuningTask]
  - POST请求  /d-robot/pose_manage/initialize_customized
    request:
  ```
    {
      "point": {
        "angle": 3.14,
        "position": {
            "x": 0,
            "y": 0
        }
      }
    }
  ```
  - response:
    {"errorCode": "", "msg": "", "data": {}, "successed": true}

## 自定义初始化
  (传入指定初始坐标点名称)
  状态允许[RuningTask]
  - Get请求  /d-robot/pose_manage/initialize_customized?point_name=test1&turning=false
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}
    选择已保存点进行初始化,在url中传入`point_name`,可以选择已保存的初始点进行初始化,`turning`选项可以选择是否旋转初始化

# 8.任务管理
## 执行一个导航到点的匿名任务
  状态允许[RuningTask]
  - GET请求  /d-robot/navigate/move_to?pose_name=?
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}
    导航到指定名称的点
## 执行一个导航到点的匿名任务
  状态允许[RuningTask]
  - POST请求  /d-robot/navigate/move_to
  - request:
        ```
        {
            "position":{
                "point":{
                    "x": 10,  栅格位置坐标
                    "y": 23   
                },
                "angle":  1.57  //单位是弧度
            },
            "type": 0
        }
        ```
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}

## 跟随路径
  状态允许[RuningTask]
  - GET请求  /d-robot/navigate/follow_path?path_name=?
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}

## 获取导航规划的路径
  状态允许[RuningTask]
  - GET请求  /d-robot/navigate/get_realtime_path
  - response:
  ```
  {
      "errorCode": "", 
      "msg": "", 
      "data": {
          "mapInfo": {
              "gridWidth": 150,
              "gridHeight": 200,
              "originX": 15,
              "originY": 30,
              "resolution": 0.050000000456
          },
          "grid_points": [
              {
                  "x": 20,
                  "y": 30
              },
              {
                  "x": 21,
                  "y": 33
              }
          ]
      }
      "successed": true}
  ```
## 创建任务队列
  状态允许[RuningTask]
  - POST请求  /d-robot/task_manager/save_task_queue
  - requese:'{"name":"aaa","tasks":[{"name":"PlayPathTask","param":{"path_name":"MANUAL_test"}},{"name":"NavigationTask","param":{"point_name":"test1"}}]}
  - response:
  ```
  {
      "errorCode": "", 
      "msg": "", 
      "data": {}, 
      "successed": true
  }
  ```

## 删除任务
  状态允许[Idle,RuningTask]
  - GET请求 /d-robot/task_manager/delete_task_queue?task_name=
  - response:
    ```json
    {
      "errorCode": "", 
      "msg": "", 
      "data": {}, 
      "successed": true
    }
    ```
## 获取任务队列
  状态允许[RuningTask]
  tasks中的name分为:[NavigationTask,PlayPathTask]
  NavigationTask: 导航至记录的坐标点 输入参数: point_name:目标点的名称
  PlayPathTask: 沿记录的路径开始寻迹, 输入参数: path_name:记录的路径名称
  当不输入task_name时,返回所有的任务队列,否则只返回一个任务队列,即获取的任务队列,data仍为json数组
  - GET请求 /d-robot/task_manager/get_task_queue?task_name=
  - response:
  ```json
      {
          "errorCode": "",
          "msg": "successed",
          "data": [
              {
                  "tasks": [
                      {
                          "name": "NavigationTask",  
                          "param": {
                              "point_name": "test"
                          }
                      },
                      {
                          "name": "PlayPathTask",
                          "param": {
                              "path_name": "test"
                          }
                      },
                      {
                          "name": "PlayPathTask",
                          "param": {
                              "path_name": "test2"
                          }
                      }
                  ],
                  "name": "test1"
              },
              {
                  "tasks": [
                      {
                          "name": "NavigationTask",
                          "param": {
                              "point_name": "test1"
                          }
                      },
                      {
                          "name": "PlayPathTask",
                          "param": {
                              "path_name": "MANUAL_test"
                          }
                      }
                  ],
                  "name": "testtest"
              }
          ],
          "successed": true
      }
  ```

## 开始任务
  状态允许[RuningTask]
  - POSE请求 /d-robot/task_manager/start_task_queue
  - request: {"name":"yu","loop":false,"loop_time":0}
    name为任务名,loop为是否循环,如果设置了loop则loop_time至少为1
## 停止任务
  状态允许[RuningTask]
  - GET请求 /d-robot/task_manager/stop_task_queue
  - response:
  ```json
  {
    "errorCode": "", 
    "msg": "", 
    "data": {}, 
    "successed": true
  }
  ```
## 暂停任务
  状态允许[RuningTask]
  - GET请求 /d-robot/task_manager/pause_task_queue"
  - response:
    ``json
    {
    "errorCode": "", 
    "msg": "", 
    "data": {}, 
    "successed": true
    }

## 继续任务
  状态允许[RuningTask]
  - GET请求 /d-robot/task_manager/resume_task_queue"
  - response:
    ```json
    {
      "errorCode": "", 
      "msg": "", 
      "data": {}, 
      "successed": true
    }
    ```
## 检查任务是否停止(将废弃)
  建议使用<获取当前任务状态>,该功能重复,
  - GET请求 /d-robot/task_manager/is_task_queue_finished"
  - response:
    ```
    {
      "errorCode": "", 
      "msg": "", 
      "data": True,//False 如果是Ture 代表已停止 
      "successed": true
    }
    ```
## 获取当前任务状态
  状态允许[RuningTask]
  - GET请求 /d-robot/task_manager/get_task_status"
  - response:
  ```
    {
      "errorCode": "",
      "msg": "successed",
      "data": {
        "remainingLoopTime": 0,
        "task": {
          "mapName": "localmap",
          "tasks": [
            {
              "name": "NavigationTask",
              "param": {
                "point_name": "111"
              }
            },
            {
              "name": "PlayPathTask",
              "param": {
                "path_name": "Test"
              }
            },
            {
              "name": "NavigationTask",
              "param": {
                "point_name": "3222"
              }
            },
            {
              "name": "PlayPathTask",
              "param": {
                "path_name": "fff"
              }
            }
          ],
          "mapId": "cd3f28ee-d4d6-4849-a1e6-87010392ab1c",
          "name": "Test"
        },
        "currentTask": {
          "name": "NavigationTask",
          "param": {
            "point_name": "111"
          }
        },
        "finished": false,
        "statusMessage": "Task is RUNNING",
        "statusCode": 1
      },
      "successed": true
    }
  ```
## 获取导航状态推送
  状态允许[RuningTask]
  - GET请求  /d-robot/navigate/get_navigator_status
  - response:
    ```
    {
        "nav_status_type": "idle",
        "nav_status_data": {
            "finished": "value",
            "information": "NO_PATH"
        },
        "process": {
            "path_name": "__DEFAULT",
            "index": "563",
            "total_milleage": "5.786121",
            "total_time": "16.072559",
            "remaining_milleage": "0.156121",
            "remaining_time": "0.433670",
            "passed_points": "[]",
            "next_point": "__DEFAULT_1"
        }
    }
    ```
    在data中 
    nav_status_type: ["idle","paused","follow_path","navigation","UNKNOW"]
                             idle:空闲状态
                             paused:暂停状态
                             follow_path:寻路状态
                             navigation:导航至点状态
                             UNKNOW:未知
    nav_status_data: finished: 如果导航结束(无论到达或放弃),则为true,如果仍在运行,则为false
                      information: 导航状态信息
    process:在不同的idle下有不同的状态信息:目前只在巡线过程中进程显示
    - 巡线process:
    ```
    "process": {
        "path_name": "__DEFAULT",  
        "index": "563",
        "total_milleage": "5.786121",  //总里程
        "total_time": "16.072559",      //总共用时
        "remaining_milleage": "0.156121",   //剩余里程
        "remaining_time": "0.433670",       //预计剩余时间
        "passed_points": "[]",
        "next_point": "__DEFAULT_1"
    }
    ```
    - 其他状态目前为空
# 9.快速轨道图编辑(暂未使用)
## 更新快速轨道图 
  - POSE请求 /d-robot/track_manage/add_track_graph
  - request: 
  ```
{
    "Edges": [
        {
            "bidirectional": true,
            "end": "1",
            "radius": 0,
            "start": "0"
        },
        {
            "bidirectional": true,
            "end": "2",
            "radius": 10,
            "start": "1"
        }
    ],
    "Nodes": [
        {
            "gridPosition": {
                "x": 101,
                "y": 68
            },
            "RecordName": Null
            "name": "0"
        },
        {
            "gridPosition": {
                "x": 0,
                "y": 0
            },
            "RecordName": Jzzj
            "name": "1"
        },
        {
            "gridPosition": {
                "x": 0,
                "y": 0
            },
            "RecordName": "Origin"
            "name": "2"
        }
    ]
}
  ```
## 获取轨道图
  - GET请求 /d-robot/track_manage/get_track_graph
  - response:
  ```
  {
    "errorCode": "",
    "msg": "successed",
    "data": {
    "Edges": [
                {
                    "bidirectional": true,
                    "end": "1",
                    "radius": 0,
                    "start": "0"
                },
                {
                    "bidirectional": true,
                    "end": "2",
                    "radius": 10,
                    "start": "1"
                }
            ],
            "Nodes": [
                {
                    "gridPosition": {
                        "x": 101,
                        "y": 68
                    },
                    "RecordName": Null
                    "name": "0"
                },
                {
                    "gridPosition": {
                        "x": 0,
                        "y": 0
                    },
                    "RecordName": Jzzj
                    "name": "1"
                },
                {
                    "gridPosition": {
                        "x": 0,
                        "y": 0
                    },
                    "RecordName": "Origin"
                    "name": "2"
                }
            ]
        },
    "successed": true
  }
  ```
# 10.控制命令
# 10.控制命令
## 控制机器人移动
  - POSE请求 /d-robot/cmd/move
  - request: {"speed":{"linearSpeed":0.2,"angularSpeed":0}} 
    linearSpeed为向前的线速度(前进为正后退为负),angularSpeed为旋转的线速度(左转为正右转为负)
## 控制云台各项开关
  - POSE请求 /d-robot/cmd/ptz_switch
  - request: 
    {
        "wiper": true, 
        "lighting": true,
        "thermal": true
    }
    如果不控制目标,则不发送关键字
## 控制云台移动指定位置
  - POSE请求 /d-robot/cmd/ptz_location_move
  - request: 
    {
        "location": {
            "yaw": 0,
            "pitch": 0,
            "wiper_state": false
        }
    }
    控制云台移动到指定角度: 角度限制yaw(0~360)pitch(0~90,270~360)
## 控制云台向指定方向移动
  - POSE请求 /d-robot/cmd/ptz_direction_move
  - request: 
    {
        "direction": {
            "direction": 0,
            "speed": 0
        }
    }
    控制云台指定方向移动:
    direction为:
      0为停止
      1为向上
      2为向下
      3为向左
      4为向右
      5为雨刷开关,当speed:0为关,其他值为开
      6为灯光开关,当speed:0为关,1为开
      7为热成像开关,当speed:0为关,1为开
    speed限制为整形,取值范围为0~64
## 获取机器人状态
  - GET请求 /d-robot/cmd/get_robot_state
  - response:
  ```
  {
    "errorCode": "",
    "msg": "", 
    "data": {
        "state_initialized":true
        "state_operation" : "Idle"
    }, 
    "successed": true
  }
  ```
  state_operation: "Idle" :空闲状态  "Mapping":建图中 "RunningTask":开启导航中

## 灯光控制(暂空)

- POSE请求 /d-robot/cmd/light_control 
  
## 定角度旋转
  - POSE请求 /d-robot/cmd/rotate_move
  - request: {"angle":359,"speed":0.5}  
    angle: 制定旋转度数, speed:指定旋转速度(正值向左转,负值向右转)
## 定距离移动
  - POSE请求 /d-robot/cmd/linear_move
  - request: {"distance":0.5,"speed":0.2} 速度不允许超过0.6
    distance: 指定移动距离, speed:指定移动速度(正值向前走,负值向后走)
## 检查移动是否完成
  检查定距离或定角度旋转是否完成
  - GET请求: /d-robot/cmd/check_move_finished
  - response:{"errorCode": "", "msg": "", "data": true, "successed": true}
    data中的true或者false表示是否完成旋转
## 停止移动控制
  停止定角度旋转或停止定距离移动
  - GET请求: /d-robot/cmd/stop_move
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}

## 开始录制bag
  - GET请求: /d-robot/cmd/start_record_bag?bag_name=&record_type=
    bag_name中输入要录制的包的名称
    record_type:应该传输的种类为["default","mapping","running"],为 默认状态,建图状态与运行状态下的包录制
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}
## 停止录制bag
  - GET请求: /d-robot/cmd/stop_record_bag
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}
## bag列表获取
  - GET请求: /d-robot/cmd/get_bag_list
  - response:
  ```
  {
      "errorCode": "",
      "msg": "",
      "data": {
          "baglist": [
              {
                  "name": "bagname2",
                  "size": 1000
              },
              {
                  "name": "bagname1",
                  "size": 2000
              },
              {
                  "name": "bagname2",
                  "size": 1000
              }
          ]
      },
      "successed": true
  }
  ```
## 删除bag文件
  - GET请求: /d-robot/cmd/delete_bag?bag_name=
  - response:{"errorCode": "", "msg": "", "data": {}, "successed": true}
## 下载bag
  - GET请求: /d-robot/cmd/download_bag?bag_name=
  - response 下载的bag文件
## 判断是否正在录制bag
  - GET请求: /d-robot/cmd/is_bag_recording
  - response
  ```
  {
    "errorCode": "",
    "msg": "successed",
    "data": {
      "state": false,  //录制bag状态,如果正在录制,则为true,同时name会发布正在录制路径的前缀名称(无法直接用作下载,该名称为前缀名)
      "name": ""
    },
    "successed": true
  }
  ```
## 获取版本信息
  - GET请求: /d-robot/cmd/get_product_version
  ```
  {
    "errorCode": "",
    "msg": "successed",
    "data": {
      "version": "SmartDonkeyV1.1.11.13.1"
    },
    "successed": true
  }
  ```
# 11.获取数据
## 获取原始速度数据
  - GET请求: /d-robot/sensor_data/raw_velocity
  - response:
    ```
    {
        "errorCode": "",
        "msg": "successed",
        "data": {
            "device_type": "motor_vel",
            "device_id": 1,
            "device_data": {
                "linear_vel": 0,
                "linear_y": 0,
                "angular_rotate": 0
            }
        },
        "successed": true
    }
    ```
    
## 获取原始激光数据
  - GET请求: /d-robot/sensor_data/raw_scan
  - response:
    ```
    {
        "errorCode": "",
        "msg": "successed",
        "data": {
            "header":{
                "stamp":14321212,
                "frame_id":laser
            },
            "angle_min":-2.3, // 数据开始角度，单位弧度
            "angle_max":2.3, // 数据结束角度
            "angle_increment":0.0058171823620796204, // 角度步长
            "range_min":0.05000000074505806, // 激光最小范围，单位米
            "range_max":10, // 激光最大范围
            "ranges":[4.941999912261963, 4.9710001945495605, ... (省略), 2.4079999923706055],
            "intensities":[0.0, 284.0, 282.0, 286.0, 284.0, 283.0, 284.0,　... (省略), 230.0, 0.0, 0.0] 
        },
        "successed": true
    }
    ```
## 获取栅格化激光数据
  在[RuningTask]的情况才能获得栅格数据
  - GET请求: /d-robot/sensor_data/scan_grid
  - response:
  ```
  {
      "errorCode": "",
      "msg": "successed",
      "data":     {
      "sensor_type": "laser_scan_grid",
      "sensor_id": 1,
      "sensor_data": {
          "header": {
              "seq": "value",
              "stamp": "value",
              "frame_id": "laser"
          },
          "angle_min": "value",
          "angle_max": "value",
          "angle_increment": "value",
          "time_increment": "value",
          "scan_time": "value",
          "range_min": "value",
          "range_max": "value",
          "points": [
              {
                  "x": "value",
                  "y": "value",
                  "z": "value"
              }
          ],
          "gridpoint": [
              {
                  "x": 0,
                  "y": 0
              }
          ],
          "intensities": [],
          "mapinfo": {
              "mapname": "value",
              "gridWidth": "value",
              "gridHeight": "value",
              "originX": "value",
              "originY": "value",
              "resolution": "value"
          }
      }
  },
      "successed": true
  }

  ```
  其中gridpoint数组里的值为栅格化后点云数据,坐标系为图片左下角为(0,0)为原点
## 获取原始里程计数据
  - GET请求: /d-robot/sensor_data/raw_odom
  - response :
  ```
  {
      "errorCode": "",
      "msg": "successed",
      "data": {
          {
              "header": {
                  "frame_id": "base_odom",
                  "stamp": 848461584
              },
              "pose": {
                  "orientation": {
                      "w": 1,
                      "x": 0,
                      "y": 0,
                      "z": 0
                  },
                  "position": {
                      "x": 0,
                      "y": 0,
                      "z": 0
                  }
              },
              "twist": {
                  "angular": {
                      "x": 0,
                      "y": 0,
                      "z": 0
                  },
                  "linear": {
                      "x": 0,
                      "y": 0,
                      "z": 0
                  }
              }
          }
      },
      "successed": true
  }

  ```
## 获取机器人外观数据
  在[RuningTask]的情况获得机器人外观在地图上的坐标数据
  - GET请求: /d-robot/sensor_data/robot_footprint
  - response:
  ```
  {
      "errorCode": "",
      "msg": "successed",
      "data":     {
      "mapInfo": {
          "gridWidth": "value",
          "gridHeight": "value"
      },
      "world_points": [
          {
              "x": 0,
              "y": 0,
              "z": 0
          }
      ],
      "grid_points": [
          {
              "x": 0,
              "y": 0
          }
      ]
  },
      "successed": true
  }

  ```
  其中主要数据在grid_points,为栅格化后的机器人外观栅格数据
## 获取机器人电量信息
  - GET请求: /d-robot/sensor_data/battery
  - response:
  ```
  {
      "errorCode": "",
      "msg": "successed",
      "data":     {
        "errorCode": "",
        "msg": "successed",
        "data": {
          "battery_24voltage": 0,
          "battery_12voltage": 0,
          "battery_capacity_percentage": 4,
          "battery_temperature": 2,
          "battery_current": 3,
          "battery_voltage": 1,
          "battery_12current": 0,
          "battery_power": 0,
          "battery_24current": 0
        },
        "successed": true
      },
      "successed": true
  }
  ```
## 获取气象信息
  - GET请求: /d-robot/sensor_data/meteorology
  - response:
  ```json
  {
      "errorCode": "",
      "msg": "successed",
      "data":     {
        "pm2dot5": 0,  //pm2.5
        "pm10": 0,     //pm10
        "air_temperature": 0, //空气温度
        "air_hiumidity": 0  //空气湿度
      },
      "successed": true
  }
  ```
## 获取机器人状态信息
  - GET请求: /d-robot/sensor_data/robot_status
  - response:
  ```
  {
      "errorCode": "",
      "msg": "successed",
      "data":     {
      "battery_status": { //电池状态
          "battery_voltage": "value",
          "battery_current": "value",
          "battery_capacity_percentage": "value",
          "battery_temperature": "value",
          "battery_power": "value",
          "battery_24voltage": "value",
          "battery_24current": "value",
          "battery_12voltage": "value",
          "battery_12current": "value"
      },
      "driving_motor_status": { //驱动电机状态
          "motor_state": "value",
          "motor_temperature": "value",
          "motor_current": "value",
          "motor_speed": "value",
          "motor_brakes": "value"
      },
      "turning_motor_status": { //转向电机状态
          "motor_state": "value",
          "motor_temperature": "value",
          "motor_current": "value",
          "motor_speed": "value",
          "motor_brakes": "value"
      },
      "TPZ_status": "value",  //云台状态
      "camera_status": "value",  //摄像机状态
      "TI_status": "value",   //
      "vidio_status": "value",    //录像机状态
      "navigator_status": "value", //导航状态
      "laser_status": "value", //激光状态
      "protector_status": "value", //防撞条状态
      "drop_status": "value", //防跌落状态
      "ultrasonic_status": "value"  //超声波状态
      },
      "successed": true
  }

  ```


## 获取机器人位置数据
  - GET请求: /d-robot/sensor_data/robot_pose_grid
  - response:
  ```
  {
      "errorCode": "",
      "msg": "successed",
      "data":     {
    "nav_status_type": "robot_pose",
    "robot_id": "1",
    "worldPose": {
      "position": {
        "y": 0.027344112849991272,
        "x": -0.13367524441632958,
        "theta": -0.007225303706825474
      }
    },
    "nav_status_data": {
      "angle": -0.007225303706825474,
      "gridPosition": {
        "y": 32,
        "x": 283
      },
      "mapinfo": {
        "originY": -1.586968,
        "gridWidth": 433,
        "originX": -14.29386,
        "mapname": "localmap",
        "gridHeight": 195,
        "resolution": 0.05000000074505806
      }
    }
  },
      "successed": true
  }

  ```

## 获取防撞条数据
  - GET请求: /d-robot/sensor_data/bumper
    从右至左数据依次为急停开关,前防撞条,后防撞条,左防撞条(未使用),右防撞条(未使用)
  - response:
  ```
  {
  "errorCode": "",
  "msg": "successed",
  "data": "001",
  "successed": true
  }
  ```
## 获取超声波数据
  - GET请求: /d-robot/sensor_data/raw_ultrasonic
  - response:
    ```
    {
      "errorCode": "",
      "msg": "successed",
      "data": [  //数据的个数为超声波收到的信息个数
        {
          "angle": 0.5200000000000001,
          "range": 0.20000000298023224,
          "originY": 0.1,
          "originX": 0.33,
          "frameid": "ultrasound1"
        },
        {
          "angle": 0,
          "range": 0,
          "originY": 0,
          "originX": 0.33,
          "frameid": "ultrasound2"
        },
        {
          "angle": -0.5200000000000001,
          "range": 0.15000000596046448,
          "originY": -0.1,
          "originX": 0.33,
          "frameid": "ultrasound3"
        }
      ],
      "successed": true
    }
    ```

## 获取红外线数据
  - GET请求: /d-robot/sensor_data/raw_irsensor
  - response:
    ```
    {
      "errorCode": "",
      "msg": "successed",
      "data": [  //数据的个数为红外线收到的信息个数
        {
          "angle": 0.5200000000000001,
          "range": 0.20000000298023224,
          "originY": 0.1,
          "originX": 0.33,
          "frameid": "irsensor1"
        },
        {
          "angle": 0,
          "range": 0,
          "originY": 0,
          "originX": 0.33,
          "frameid": "irsensor2"
        },
        {
          "angle": -0.5200000000000001,
          "range": 0.15000000596046448,
          "originY": -0.1,
          "originX": 0.33,
          "frameid": "irsensor3"
        }
      ],
      "successed": true
    }
    ```


13. 获取底盘状态数据
  - GET请求: /d-robot/sensor_data/chassis_state
    状态返回值为0时,设备正常
  - response:
  ```
  {
    "errorCode": "",
    "msg": "successed",
    "data": {
      "Laser": 1,//激光雷达状态
      "Infrared": 2,  //红外线状态,请将其转换为2进制类型后,以位的方式计算,如:2 = (二进制)0010 ,从右向左,表示第一个红外线正常,第二个红外线异常,第三个红外线正常
      "Ultrasonic": 2,  //超声波状态,同上
      "Right_Motor": 1, //右电机状态
      "Left_Motor": 0,//左电机状态
      "E_stop": 1 ,//急停开关状态
      "ChargingState": 0,  //如果为1,表示当前正在充电
      "ChargedState" : 0   //如果为0,表示充电已完成
    },
    "successed": true
  }
  ```
# 12.参数配置

## 获取参数列表
  - GET请求 /d-robot/param/get_param
  - response
  ```
{
  "errorCode": "",
  "msg": "successed",
  "data": [
  {
    "type": "double",
    "namespace": "/devices/battery/empty_voltage",
    "value": "22.000000"
  },
  {
    "type": "double",
    "namespace": "/devices/battery/full_voltage",
    "value": "28.000000"
  },
  {
    "type": "double",
    "namespace": "/devices/irsensor/irsensor1_max_range",
    "value": "0.150000"
  },
  {
    "type": "double",
    "namespace": "/devices/irsensor/irsensor1_min_range",
    "value": "0.050000"
  },
  {
    "type": "bool",
    "namespace": "/devices/irsensor/irsensor1_switch",
    "value": "true"
  },
  {
    "type": "double",
    "namespace": "/devices/irsensor/irsensor2_max_range",
    "value": "0.150000"
  },
  {
    "type": "double",
    "namespace": "/devices/irsensor/irsensor2_min_range",
    "value": "0.050000"
  },
  {
    "type": "bool",
    "namespace": "/devices/irsensor/irsensor2_switch",
    "value": "true"
  },
  {
    "type": "double",
    "namespace": "/devices/irsensor/irsensor3_max_range",
    "value": "0.150000"
  },
  {
    "type": "double",
    "namespace": "/devices/irsensor/irsensor3_min_range",
    "value": "0.050000"
  },
  {
    "type": "bool",
    "namespace": "/devices/irsensor/irsensor3_switch",
    "value": "true"
  },
  {
    "type": "double",
    "namespace": "/devices/ultrasonic/ultrasound1_max_range",
    "value": "0.400000"
  },
  {
    "type": "double",
    "namespace": "/devices/ultrasonic/ultrasound1_min_range",
    "value": "0.200000"
  },
  {
    "type": "bool",
    "namespace": "/devices/ultrasonic/ultrasound1_switch",
    "value": "true"
  },
  {
    "type": "double",
    "namespace": "/devices/ultrasonic/ultrasound2_max_range",
    "value": "0.400000"
  },
  {
    "type": "double",
    "namespace": "/devices/ultrasonic/ultrasound2_min_range",
    "value": "0.200000"
  },
  {
    "type": "bool",
    "namespace": "/devices/ultrasonic/ultrasound2_switch",
    "value": "true"
  },
  {
    "type": "double",
    "namespace": "/devices/ultrasonic/ultrasound3_max_range",
    "value": "0.400000"
  },
  {
    "type": "double",
    "namespace": "/devices/ultrasonic/ultrasound3_min_range",
    "value": "0.200000"
  },
  {
    "type": "bool",
    "namespace": "/devices/ultrasonic/ultrasound3_switch",
    "value": "true"
  },
  {
    "type": "double",
    "namespace": "/navigation/charger/backward_dis",
    "value": "0.500000"
  },
  {
    "type": "double",
    "namespace": "/navigation/charger/backward_velocity",
    "value": "0.100000"
  },
  {
    "type": "double",
    "namespace": "/navigation/charger/goal_offset_dis",
    "value": "0.400000"
  },
  {
    "type": "double",
    "namespace": "/navigation/charger/going_forward_offset_dis",
    "value": "0.500000"
  },
  {
    "type": "double",
    "namespace": "/navigation/charger/rotate_yaw_tolerance",
    "value": "0.050000"
  },
  {
    "type": "bool",
    "namespace": "/navigation/follow/avoid_obstacle",
    "value": "true"
  },
  {
    "type": "double",
    "namespace": "/navigation/follow/front_safe_check_dis",
    "value": "2.000000"
  },
  {
    "namespace": "/navigation/follow/speed_level",
    "limit": {
      "lower_bound": "0",
      "upper_bound": "2"
    },
    "value": "1",
    "type": "int"
  },
  {
    "type": "double",
    "namespace": "/navigation/nav/find_next_goal_duration",
    "value": "2.000000"
  },
  {
    "type": "double",
    "namespace": "/navigation/nav/goal_unsafe_quit_duration",
    "value": "8.000000"
  },
  {
    "namespace": "/navigation/nav/speed_level",
    "limit": {
      "lower_bound": "0",
      "upper_bound": "2"
    },
    "value": "1",
    "type": "int"
  }
  ],
  "successed": true
}
  ```

2. 参数更新
  - POST请求:/d-robotparam/update_param
  - request:
  ```
  {
    "params" : [
      {
        "type": "double",
        "namespace": "/devices/battery/empty_voltage",
        "value": "22.000000"
      },
      {
        "type": "double",
        "namespace": "/devices/battery/full_voltage",
        "value": "28.000000"
      },
      {
        "type": "double",
        "namespace": "/devices/irsensor/irsensor1_max_range",
        "value": "0.150000"
      }
    ]
  }
  ```
  - response:
  ```
  {
    "errorCode": "",
    "msg": "successed",
    "data": "",
    "successed": true
  }
  ```

## 参数列表
  ```
  导航设置-跟随路径-避障开关(开启或关闭跟随路径时是否选择进行避障)
  - namespace: /navigation/follow/avoid_obstacle 
    type: bool
  导航设置-跟随路径-安全检测距离(检测前方障碍物距离)
  - namespace: /navigation/follow/front_safe_check_dis
    type: double
  导航设置-跟随路径-速度设置(跟随路径时的速度)
  - namespace: /navigation/follow/speed_level
    type: int
  导航设置-自主导航-等待目标点空闲最大时间()
  - namespace: /navigation/nav/goal_unsafe_quit_duration
    type: double
  导航设置-自主导航-避障重规划路径最大时间()
  - namespace: /navigation/nav/find_next_goal_duration
    type: double
  导航设置-自主导航-速度设置(自主导航速度)
  - namespace: /navigation/nav/speed_level
    type: int
  导航设置-自主导航-是否使用轨道
  - namespace: /navigation/nav/topo_enable
    type: bool
  导航设置-充电动作-后退距离
  - namespace: /navigation/charger/backward_dis  
    type: double
  导航设置-充电动作-后退速度
  - namespace: /navigation/charger/backward_velocity  
    type: double
  导航设置-充电动作-充电完成后前进距离
  - namespace: /navigation/charger/going_forward_offset_dis
    type: double
  导航设置-充电动作-充电角度偏差
  - namespace: /navigation/charger/rotate_yaw_tolerance  
    type: double
  导航设置-充电动作-导航至充电点前距离
  - namespace: /navigation/charger/goal_offset_dis
    type: double

  设备设置-电池电量-空电量电压
  - namespace: /devices/battery/empty_voltage
    type: double
  设备设置-电池电量-满电量电压
  - namespace: /devices/battery/full_voltage
    type: double
  设备设置-红外线设置-红外线设备1开关
  - namespace: /devices/irsensor/irsensor1_switch
    type: bool
  设备设置-红外线设置-红外线设备2开关
  - namespace: /devices/irsensor/irsensor2_switch
    type: bool
  设备设置-红外线设置-红外线设备3开关
  - namespace: /devices/irsensor/irsensor3_switch
    type: bool
  设备设置-红外线设置-红外线设备1最小检测距离
  - namespace: /devices/irsensor/irsensor1_min_range
    type: double
  设备设置-红外线设置-红外线设备2最小检测距离
  - namespace: /devices/irsensor/irsensor2_min_range
    type: double
  设备设置-红外线设置-红外线设备3最小检测距离
  - namespace: /devices/irsensor/irsensor3_min_range
    type: double
  设备设置-红外线设置-红外线设备1最大检测距离
  - namespace: /devices/irsensor/irsensor1_max_range
    type: double
  设备设置-红外线设置-红外线设备2最大检测距离
  - namespace: /devices/irsensor/irsensor2_max_range
    type: double
  设备设置-红外线设置-红外线设备3最大检测距离
  - namespace: /devices/irsensor/irsensor3_max_range
    type: double
  设备设置-超声波设置-超声波设备1开关
  - namespace: /devices/ultrasonic/ultrasound1_switch
    type: bool
  设备设置-超声波设置-超声波设备2开关
  - namespace: /devices/ultrasonic/ultrasound2_switch
    type: bool
  设备设置-超声波设置-超声波设备3开关
  - namespace: /devices/ultrasonic/ultrasound3_switch
    type: bool
  设备设置-超声波设置-超声波设备1最小距离
  - namespace: /devices/ultrasonic/ultrasound1_min_range
    type: double
  设备设置-超声波设置-超声波设备2最小距离
  - namespace: /devices/ultrasonic/ultrasound2_min_range
    type: double
  设备设置-超声波设置-超声波设备3最小距离
  - namespace: /devices/ultrasonic/ultrasound3_min_range
    type: double
  设备设置-超声波设置-超声波设备1最大距离
  - namespace: /devices/ultrasonic/ultrasound1_max_range
    type: double
  设备设置-超声波设置-超声波设备2最大距离
  - namespace: /devices/ultrasonic/ultrasound2_max_range
    type: double
  设备设置-超声波设置-超声波设备3最大距离
  - namespace: /devices/ultrasonic/ultrasound3_max_range
    type: double
  ```

# 系统更新接口(该系列,接口端口统一修改为10086端口)
1. 获取回滚列表
    - GET请求: :10086/get_roll_back_lists
    - response:
    ```
{
    "data": [
        {
            "name": "201911151026_OLD_SYSTEM_BACKUP.tar.gz",
            "size": 47.58606719970703
        },
        {
            "name": "201911151032_OLD_SYSTEM_BACKUP.tar.gz",
            "size": 47.58683395385742
        },
        {
            "name": "201911151409_OLD_SYSTEM_BACKUP.tar.gz",
            "size": 47.59159469604492
        },
        {
            "name": "201911151029_OLD_SYSTEM_BACKUP.tar.gz",
            "size": 47.58598327636719
        },
        {
            "name": "201911151016_OLD_SYSTEM_BACKUP.tar.gz",
            "size": 47.59076690673828
        },
        {
            "name": "201911151020_OLD_SYSTEM_BACKUP.tar.gz",
            "size": 47.58732223510742
        },
        {
            "name": "201911151011_OLD_SYSTEM_BACKUP.tar.gz",
            "size": 47.58657455444336
        },
        {
            "name": "201911151100_OLD_SYSTEM_BACKUP.tar.gz",
            "size": 47.58612823486328
        },
        {
            "name": "201911151412_OLD_SYSTEM_BACKUP.tar.gz",
            "size": 47.58699417114258
        },
        {
            "name": "201911151051_OLD_SYSTEM_BACKUP.tar.gz",
            "size": 47.58995056152344
        }
    ],
    "errorCode": "",
    "msg": "successed",
    "successed": true
}
    ```
2. 系统回滚
    param: backFile为回滚列表中的列表名称
    - GET请求: :10086/rollback_system?backFile=
    - response:
    ```
    {
        "data": "",
        "errorCode": "",
        "msg": "successed",
        "successed": true
    }
    ```
3. 清除用户数据
    清除会擦除用户所有数据,包括回滚包列表,清除后需要重启机器
    - GET请求: :10086/clear_user_data
    - response:
    ```
    {
        "data": "",
        "errorCode": "",
        "msg": "successed",
        "successed": true
    }
    ```
## 错误代码
- 100 : 状态不匹配
- 101 : 没有正在运行的地图
- 102 : 无法打开文件
- 103 : 未收到消息
- 200 : 输入要求的参数为空
- 201 : 参数格式错误
- 202 : 参数输入非法字符
- 203 : POST传参为空
- 210 : 无法调用服务
- 211 : 服务成功调用,但返回了错误
- 212 : 等待服务超时
- 300 : 建图程序打开失败
- 301 : 建图程序关闭失败 
- 801 : 旋转失败


## Q&A
- 出现错误代码100时: 机器人状态图为 Idle -> Mapping :Idle->RuningTask,无法从mapping跳转到mapping或者从RuningTask跳转到RuningTask. 故当app出现该错误时,请重启app,如果重启app无法解决问题,则需要重启机器人.

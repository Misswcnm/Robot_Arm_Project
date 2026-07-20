import json
import numpy as np
import math
import sys
import os

env_dist = os.environ
run_workspace_dir = env_dist.get('DROBOT_RUNTIME_DIR')
param_dir = env_dist.get('DROBOT_PARAM_DIR')
locationJson = run_workspace_dir + '/latest_position.json'
luaFille = param_dir+ "/init_pose.lua"
def get_initial_pose():
    # filename ='/root/Drobot_Navigation_Module/RUN_TIME_DIR/latest_position.json'
    with open(locationJson) as ip:
      initpose = json.loads(ip.read())
    orientation_x=initpose['latestPose']['orientation']['x']
    orientation_y=initpose['latestPose']['orientation']['y']
    orientation_z=initpose['latestPose']['orientation']['z']
    orientation_w=initpose['latestPose']['orientation']['w']
    position_x=initpose['latestPose']['position']['x']
    position_y=initpose['latestPose']['position']['y']
    position_z=0

    translation = np.asarray([ position_x, position_y, position_z ])
    rotation_quaternion = np.asarray([orientation_w, orientation_x, orientation_y, orientation_z])

    rotation_r = math.atan2(2*(orientation_w*orientation_x+orientation_y*orientation_z), 1-2*(orientation_x*orientation_x+orientation_y*orientation_y))
    rotation_p = math.asin(2*(orientation_w*orientation_x-orientation_y*orientation_z))
    rotation_y = math.atan2(2*(orientation_w*orientation_z+orientation_x*orientation_y), 1-2*(orientation_z*orientation_z+orientation_y*orientation_y))

    fileWrite = open(luaFille, 'w')
    fileWrite.writelines(['{\n', '    to_trajectory_id = 0,\n', '    relative_pose = {\n', '        translation = {\n', '            '+str(position_x)+',\n', '            '+str(position_y)+',\n', '            '+str(position_z)+',\n', '        },\n', '        rotation = {\n', '            '+str(rotation_p)+',\n', '            '+str(rotation_r)+',\n', '            '+str(rotation_y)+',\n', '        }\n', '                }\n', '}'])
    fileWrite.close

    return


def reset_initial_pose():
    fileWrite = open(luaFille, 'w')
    fileWrite.writelines(['{\n', '    to_trajectory_id = 0,\n', '    relative_pose = {\n', '        translation = {\n', '            0.0,\n', '            0.0,\n', '            0.0,\n', '        },\n', '        rotation = {\n', '            0.0,\n', '            0.0,\n', '            0.0,\n', '        }\n', '                }\n', '}'])
    fileWrite.close
    return


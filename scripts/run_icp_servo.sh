#!/bin/bash
# ICP Visual Servoing — launch script
cd ~/Robot_Arm_Project
source install/setup.bash
python3 -c "
import sys; sys.path.insert(0,'src')
from icp_servoing.main import main
main()
"

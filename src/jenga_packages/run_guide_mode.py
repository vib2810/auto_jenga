import numpy as np
import time
from autolab_core import RigidTransform
import sys
sys.path.append("/home/ros_ws/src/jenga_packages/frankapy")
from frankapy import FrankaArm
if __name__ == "__main__":
    
    start = time.time()
    fa = FrankaArm()
    # fa.reset_joints()
    fa.run_guide_mode(10000,block=False)

    while((time.time()-start) < 10000):
        input_num = int(input("Enter a number: "))
        if (input_num==1):
            T_ee_world = fa.get_pose()
            print("pose: ")
            print(T_ee_world)
            time.sleep(0.01)
        if(input_num==2):
            joints = fa.get_joints()
            print("joints: ")
            print(joints)
            time.sleep(0.01)
        if(input_num==3):
            fa.stop_skill()
            break
        if(input_num==4):
            fa.stop_skill()
            fa.reset_joints()
            break

    print("done")
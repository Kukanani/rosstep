import sys
import subprocess
sys.path.insert(0, '../src')
import rosstep

processes = []

if __name__ == "__main__":
    processes.insert(0, subprocess.Popen("roscore"))
    rosstep.blockForRoscore()

    awesome = "roslaunch ct_fixtures framework.launch --wait"
    processes.insert(0, subprocess.Popen(awesome.split()))

    awesome = "rosrun rqt_gui rqt_gui"
    processes.insert(0, subprocess.Popen(awesome.split()))
    rosstep.blockForCompletion(0)

    rosstep.blockForTopic("/foo")
    rosstep.blockForParam("/bar")
    rosstep.blockForSeconds(10.0)
    rosstep.blockForService("/plan_kinematic_path")
    rosstep.blockForUser("Click OK when ready to launch RQT!")

    awesome = "rosrun rqt_gui rqt_gui"
    processes.insert(0, subprocess.Popen(awesome.split()))

    for popen in processes:
        popen.wait()

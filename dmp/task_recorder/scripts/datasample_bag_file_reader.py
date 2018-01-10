PKG =  "task_recorder"
import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag

import task_recorder.msg
import csv
import os.path, time
import re
import tables as tb
import subprocess, yaml

class DataSampleBagFileReader:

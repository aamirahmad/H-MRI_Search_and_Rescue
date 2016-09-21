import sys
print("=====================================")
print("The currently used python version")
print (sys.version) 
print("=====================================")

import os
print("=====================================")
print (os.environ.get('ROS_PACKAGE_PATH'))
print (os.environ.get('PYTHONPATH'))
print("=====================================")

import subprocess
import shlex
proc1 = subprocess.Popen(shlex.split('printenv'),stdout=subprocess.PIPE)
proc2 = subprocess.Popen(shlex.split('grep -i ROS'),stdin=proc1.stdout,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
proc1.stdout.close()
out,err = proc2.communicate()
print("=====================================")
print(out.decode("utf-8"))
print("=====================================")
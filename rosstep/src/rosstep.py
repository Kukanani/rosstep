#!/usr/bin/env python
## Runs ROSSTEP files sequentially.
#
# ROSSTEP is designed to make up for the shortcomings of the node-based parallel ROS paradigm, which are not adequately addressed by roslaunch.
# Each step of a ROSSTEP YAML file has either an action, a condition, or both. In this way, ROSSTEP can ensure that ROS components are
# started in the correct order to avoid crashes.

import subprocess
import signal
import sys
import os
import time
import yaml
import socket

import rospy
import rosgraph
import Tkinter, tkMessageBox

__author__     = "Adam Allevato"
__copyright__  = "Copyright 2015, Los Alamos National Laboratory"
__credits__    = ["Adam Allevato, Clinton Peterson"]
__license__    = "Proprietary"
__version__    = "1.2.1"
__maintainer__ = "Adam Allevato"
__email__      = "allevato@utexas.edu"
__status__     = "Production"

# VARS ######################################################

processes = []          # A list of all processes started by ROSSTEP
required_processes = [] # If any of these processes die, ROSSTEP will kill everything
print_timeout = 10.0    # Time between each printout reminding the user what we're waiting for

# HELPER ######################################################

## Defines ANSI color codes for colored terminal printing
class bcolors:
    INFO      = '\033[96m'  # cyan
    BOLD      = '\033[1m'   # sets bold
    UNDERLINE = '\033[4m'   # adds underline
    ENDC      = '\033[0m'   # clears formatting

## Print the specified string with all rosstep formatting.
#  In this case, cyan, bold, underline
def color_print(msg):
    print bcolors.UNDERLINE + bcolors.INFO + bcolors.BOLD + msg + bcolors.ENDC

def shutdown():
    color_print("Ending processes...");
    for proc in processes:
        if proc.poll() is None:
            try:
                proc.send_signal(signal.SIGTERM)
                proc.wait()
            except OSError: # Not running, ignore error because the goal is to shut them all down anyway
                pass
    color_print("Shutdown complete.");
    sys.exit()

## Handle a terminal signal by gracefully shutting down.
#  I'm not sure if this is ever called, since many ROS nodes (and roscore itself) handle these signals themselves.
def signal_handler(signal, frame):
    shutdown()
    
def loop_until_die():
    color_print("Press Ctrl-C to shut down.")
    
    keep_going = True
    while keep_going:
        for proc in required_processes:
            if proc.poll() is not None: #terminated required, shutdown
                color_print("required process has ended.")
                shutdown()
        if len(required_processes) <= 0:
            color_print("All conditions are met and no required processes are running.")
            shutdown()

def kill_last_task():
    color_print("ending temporary step")
    try:
        required_processes.remove(processes[0])
        processes[0].send_signal(signal.SIGTERM)
        processes[0].wait()
        del processes[0]
    except OSError:
        pass

# THE BASIC BLOCK TYPE #########################################

## Block until the predicate returns true.
#  This function will repeatedly call the predicate (which should return a boolean) using the given arguments. It will call the predicate as fast as possible and only return when the predicate returns true.
#  The function keeps the user well informed with an initial message (infoMsg), repeated messages if the wait lasts for a while (infoMsg every print_timeout seconds), and a message when the predicate returns true (doneMsg).
def block_until_predicate(infoMsg, doneMsg, predicate, *args):
    start = 0   #ensures that infoMsg will print immediately regardless of print_timeout, if the predicate fails
    elapsed = 0 #time since the last infoMsg was printed
    while True:
        if(predicate(*args)): #call the predicate
            #if it worked, tell the user and return
            color_print(doneMsg)
            return
        # the predicate failed, so calculate timing
        elapsed = time.time() - start
        
        #print the info message at the timeout rate
        if elapsed > print_timeout:
            color_print(infoMsg)
            start = time.time()
            elapsed = 0
    return

# ROSCORE ######################################################

## Returns whether or not roscore is currently running
def is_roscore_running():
    try:
        rosgraph.Master('/rostopic').getPid()
        return True
    except socket.error:
        return False

## Return whether or not roscore is running, but also start it if it is not. 
def force_roscore():
    ret = is_roscore_running()
    if not ret:
        processes.insert(0, subprocess.Popen("roscore"))
        rospy.sleep(2.0) # even if this is not long enough, return false, so this function will be called again.
    return ret

## Block until roscore is running and initialized
def block_roscore():
    return block_until_predicate("Waiting for roscore", "Roscore is running. Moving to next step", force_roscore);
    
# TOPIC ######################################################

## Return whether or not a topic exists on roscore.
#  Note that just because a topic exists, does not mean it is being published to, especially if the roscore has been running over multiple program tests.
def is_topic_published(topic):
    topics = rospy.get_published_topics() # returns list of lists
    flat_topics = [val for sublist in topics for val in sublist] # a double flatten
    if topic in flat_topics:
        return True
    return False

## Block until a topic exists on roscore
def block_topic(topic):
    if topic[0] is not '/':
        topic = '/' + topic
    return block_until_predicate("Waiting for topic " + topic, "Topic " + topic + " is published. Moving to next step", is_topic_published, topic)

# SERVICE ######################################################

## Returns whether or not a service is being published.
#  There is no equivalent of rospy.get_published_topics() for services, so we simply wait for a short time.
def wait_for_service(service, timeout = 0.1):
    try:
        rospy.wait_for_service(service, timeout)
        return True
    except: #error out means no service
        return False

## Block until a service is being published on roscore
def block_service(service):
    if service[0] is not '/':
        service = '/' + service
    return block_until_predicate("Waiting for service " + service, "Service " + service + " is available. Moving to next step", wait_for_service, service)

# PARAM ######################################################

## Returns whether or not a parameter exists on roscore.
def does_param_exist(param):
    try:
        params = rospy.get_param_names() # list of params
        if param in params:
            return True
    except:
        pass
    return False

## Block until a parameter exists on roscore
def block_param(param):
    if param[0] is not '/':
        param = "/" + param
    return block_until_predicate("Waiting for param " + param, "Param " + param + " exists. Moving to next step", does_param_exist, param)

# OTHER ######################################################

## Block for a specified number of seconds
def block_seconds(seconds):
    color_print("Waiting for " + str(seconds) + " seconds.")
    rospy.sleep(seconds)
    color_print("Wait finished. Continuing")

## Show a dialog message to the user and block until they press the OK button or close the window
def block_user_prompt(message):
    color_print(message);
    color_print("Waiting for user")
    title = "Waiting for User"
    root = Tkinter.Tk()
    root.withdraw()
    tkMessageBox.showinfo(title, message)
    color_print("User input received. Moving to next step")

## Block until the latest process has finished
def block_completion(index):
    if len(processes) > index:
        color_print("Waiting for process to complete")
        processes[index].wait()
        if processes[index] in required_processes:
            required_processes.remove(processes[index])
        del processes[index]
        color_print("Process completed. Moving to next step")

## Perform a block. The type is determined by the condition parameter.
def block(condition, condition_text, temporary):
    if condition == "user prompt":
        block_user_prompt(condition_text)
    elif condition == "ROS topic":
        block_topic(condition_text)
    elif condition == "ROS service":
        block_service(condition_text)
    elif condition == "ROS param":
        block_param(condition_text)
    elif condition == "completion":
        block_completion(0)
    elif condition == "seconds":
        block_seconds(condition_text)
    elif condition == "none" or condition == "":
        if temporary:
            color_print("This step is temporary, but has no condition and will die immediately.")
        # Else do nothing
    else:
        if not condition:
            condition = ""
        color_print("condition " + condition + " not recognized.")

# LOAD YAML FILE #######################

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Must specify rosstep YAML file as command-line argument"
        sys.exit(0)
    script = sys.argv[1]
    stream = open(script, "r")
    docs = yaml.load(stream)
    
    color_print("ROSSTEP 1.2 (2015)")
    
    try:
        block_roscore()
            
        for doc in docs:
            # initialize variables
            command = ""
            
            if "action" in doc:
                new_action = doc["action"]
            else:
                new_action = ""
            
            temporary = False
            if "temporary" in doc and doc["temporary"]:
                temporary = True
            
            # process action type and add script command accordingly
            if new_action == "rosrun" or new_action == "roslaunch":
                command += new_action
                command += " "
                command += doc["action_text"]
            elif new_action == "bash":
                command += doc["action_text"]
            elif new_action == "none":
                pass
            else:
                color_print("action type " + new_action + " not supported.")
            
            # if there was a command...DO IT!
            if(command):
                color_print("".join(command))
                
                proc = subprocess.Popen(command.split());
                processes.insert(0, proc)
                if "required" in doc and doc["required"]:
                    required_processes.insert(0, proc)
            
            # get the condition information
            condition = doc["condition"];
            if "condition_text" in doc:
                condition_text = doc["condition_text"]
            else:
                condition_text = ""
                
            # wait for the condition
            block(condition, condition_text, temporary)

            #if it was a temporary step, kill it.
            if temporary and condition != "completion":
                kill_last_task()
        
        loop_until_die()
    except KeyboardInterrupt: #Ctrl-C
        shutdown()

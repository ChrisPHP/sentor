#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  1 09:25:27 2022

@author: Adam Binch (abinch@sagarobotics.com)
"""
###################################################################################################################
import rospy, sys
from sentor.srv import Client, ClientRequest


def usage():
    print("\nLoads/stops/starts topic monitors:")
    print("\nFor loading monitors:")
    print("\t rosrun sentor sentor_client.py load CONFIG TAG1 TAG2 TAGN")
    print("\nFor stopping monitors:")
    print("\t rosrun sentor sentor_client.py stop TAG1 TAG2 TAGN")
    print("\nFor starting monitors:")
    print("\t rosrun sentor sentor_client.py start TAG1 TAG2 TAGN")
    print("\nTags are optional args used to load/stop/start specific monitors")
    print("\n\n")
    
    
def sentor_client(mode, config, tags):
    
    rospy.wait_for_service(mode, timeout=5.0)
    try:
        s = rospy.ServiceProxy(mode, Client)
        req = ClientRequest()
        req.config = config
        req.topic_tags = tags
        resp = s.call(req)
        rospy.loginfo(resp)
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))
###################################################################################################################
    
    
###################################################################################################################
if __name__ == "__main__":
    
    if '-h' in sys.argv or '--help' in sys.argv or len(sys.argv) < 2:
        usage()
        sys.exit(1)
    else:
        mode = ""
        config = ""
        tags = []
        if sys.argv[1] == "load":
            mode = "/sentor/load_monitors"
            if len(sys.argv) > 2:
                config = sys.argv[2]
                if len(sys.argv) > 3:
                    tags = sys.argv[3:]
            else:
                usage()
                sys.exit(1)
        elif sys.argv[1] == "stop":
            mode = "/sentor/stop_monitors"
            if len(sys.argv) > 2:
                tags = sys.argv[2:]
        elif sys.argv[1] == "start":
            mode = "/sentor/start_monitors"
            if len(sys.argv) > 2:
                tags = sys.argv[2:]
        else:
            usage()
            sys.exit(1)

    rospy.init_node("sentor_client")
    sentor_client(mode, config, tags)
###################################################################################################################
#!/usr/bin/env python
"""
Created on Fri Dec  6 08:51:15 2019

@author: Adam Binch (abinch@sagarobotics.com)
"""
#####################################################################################
from __future__ import division
import rospy
from sentor.msg import Monitor, MonitorArray
from threading import Event


class MultiMonitor(object):
    
    
    def __init__(self):

        rate = rospy.get_param("~safety_pub_rate", 10.0) 
        
        self.topic_monitors = []
        self._stop_event = Event()
        self.error_code = []
        self.actives = []
        
        self.monitors_pub = rospy.Publisher("/sentor/monitors", MonitorArray, latch=True, queue_size=1)
        self.monitors_pub.publish(MonitorArray())
        rospy.Timer(rospy.Duration(1.0/rate), self.callback)


    def register_monitors(self, topic_monitor):
        self.topic_monitors.append(topic_monitor)
        
        
    def callback(self, event=None):
        
        if not self._stop_event.isSet():
            
            error_code_new = [monitor.conditions[expr]["satisfied"] for monitor in self.topic_monitors for expr in monitor.conditions]
            actives_new = [monitor.active for monitor in self.topic_monitors]
            
            if error_code_new != self.error_code or actives_new != self.actives:
                self.error_code = error_code_new
                self.actives = actives_new
                
                conditions = MonitorArray()
                conditions.header.stamp = rospy.Time.now()
                
                count = 0                
                for monitor in self.topic_monitors:
                    topic_name = monitor.topic_name
                    topic_tags = monitor.topic_tags
                    active = monitor.active
                        
                    for expr in monitor.conditions:
                        condition = Monitor()
                        condition.topic = topic_name
                        condition.condition = expr
                        condition.safety_critical = monitor.conditions[expr]["safety_critical"]
                        condition.autonomy_critical = monitor.conditions[expr]["autonomy_critical"]
                        condition.tags = monitor.conditions[expr]["tags"]
                        condition.topic_tags = topic_tags
                        condition.satisfied = self.error_code[count]
                        condition.active = active
                        conditions.conditions.append(condition)
                        count+=1
                        
                self.monitors_pub.publish(conditions)
                        
                
    def stop_monitor(self):
        self._stop_event.set()


    def start_monitor(self):
        self._stop_event.clear()
#####################################################################################
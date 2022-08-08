#!/usr/bin/env python
"""
@author: Adam Binch (abinch@sagarobotics.com)
@author: Francesco Del Duchetto (FDelDuchetto@lincoln.ac.uk)
"""
#####################################################################################
from __future__ import division
import rospy, math, numpy, json
# imported the packages math and numpy so that they can be used in the lambda expressions

def _import(location, name):
    mod = __import__(location, fromlist=[name])
    return getattr(mod, name)


class ROSTopicFilter(object):

    def __init__(self, topic_name, lambda_fn_str, config, throttle_val):
        self.topic_name = topic_name
        self.lambda_fn_str = lambda_fn_str
        self.config = config
        self.throttle_val = throttle_val
        self.throttle = self.throttle_val
        self.lambda_fn = None
        self.custom_lambda = False
        try:
            if config["file"] is not None and config["package"] is not None:
                lambda_fn = _import("{}.{}".format(config["package"], config["file"]), self.lambda_fn_str)
                if config["init_args"] is not None:
                    args = config["init_args"]
                    self.lambda_fn = lambda_fn(*args)
                else:
                    self.lambda_fn = lambda_fn()
                self.custom_lambda = True
            else:
                self.lambda_fn = eval(self.lambda_fn_str)
        except Exception as e:
            rospy.logerr("Error evaluating lambda function %s : %s" % (self.lambda_fn_str, e))

        self.filter_satisfied = False
        self.unread_satisfied = False
        self.value_read = False
        self.sat_callbacks = []
        self.unsat_callbacks = []


    def callback_filter(self, msg):
        if self.lambda_fn is None:
            return
        try:
            if not self.custom_lambda:
                self.filter_satisfied = self.lambda_fn(msg)
            else:
                if self.config["run_args"] is not None:
                    args = self.config["run_args"]
                    self.filter_satisfied = self.lambda_fn.run(msg, *args)
                else:
                    self.filter_satisfied = self.lambda_fn.run(msg)
        except Exception as e:
            rospy.logwarn("Exception while evaluating %s: %s" % (self.lambda_fn_str, e))

        # if the last value was read: set value_read to False
        if self.value_read:
            self.value_read = False
        elif self.filter_satisfied:
            self.unread_satisfied = True
            # notify the listeners

        if self.filter_satisfied:
            for func in self.sat_callbacks:
                func(self.lambda_fn_str, msg, self.config)
        else:
            for func in self.unsat_callbacks:
                func(self.lambda_fn_str)


    def callback_filter_throttled(self, msg):
        if (self.throttle % self.throttle_val) == 0:
            self.callback_filter(msg)
            self.throttle = 1
        else:
            self.throttle += 1


    def is_filter_satisfied(self):
        self.value_read = True
        if self.unread_satisfied:
            self.unread_satisfied = False
            return True

        return self.filter_satisfied


    def register_satisfied_cb(self, func):
        self.sat_callbacks.append(func)


    def register_unsatisfied_cb(self, func):
        self.unsat_callbacks.append(func)
#####################################################################################
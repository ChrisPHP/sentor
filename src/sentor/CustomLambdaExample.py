#!/usr/bin/env python
"""
Created on Fri Nov 20 11:35:22 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
# SENTOR CUSTOM LAMBDA

class CustomLambda(object):
    
    def __init__(self):
        pass
        
    def run(self, msg):
        return msg.data == "r1-cy"
#########################################################################################################
#!/usr/bin/env python

##
# Software License Agreement (BSD License)
#
#  Copyright (c) 2020, MID Academic Promotions, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the MID Academic Promotions nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
##

# Author: Yosuke Matsusaka

import sys
import subprocess
import xml.etree.ElementTree as ET

def odom_t_joint():
    j = ET.Element('joint', {'name': 'odom_t', 'type': 'continuous'})
    j.append(ET.Element('axis', {'xyz': '0 0 1'}))
    j.append(ET.Element('parent', {'link': 'odom_xt_link'}))
    j.append(ET.Element('child', {'link': 'base_footprint'}))
    j.append(ET.Element('limit', {'effort': '0.1', 'velocity': '1'}))
    return (j)

def odom_x_joint():
    j = ET.Element('joint', {'name': 'odom_x', 'type': 'prismatic'})
    j.append(ET.Element('axis', {'xyz': '1 0 0'}))
    j.append(ET.Element('parent', {'link': 'odom_yx_link'}))
    j.append(ET.Element('child', {'link': 'odom_xt_link'}))
    j.append(ET.Element('limit', {'lower': '-10', 'upper': '10', 'effort': '0.1', 'velocity': '0.2'}))
    return(j)
    
def odom_y_joint():
    j = ET.Element('joint', {'name': 'odom_y', 'type': 'prismatic'})
    j.append(ET.Element('axis', {'xyz': '0 1 0'}))
    j.append(ET.Element('parent', {'link': 'odom'}))
    j.append(ET.Element('child', {'link': 'odom_yx_link'}))
    j.append(ET.Element('limit', {'lower': '-10', 'upper': '10', 'effort': '0.1', 'velocity': '0.2'}))
    return(j)

def link(name):
    l = ET.Element('link', {'name': name})
    i = ET.Element('inertial') # inertial element is required to pass urdf to sdf conversion in gazebo
    mass = 0.1
    inertia = 2.0 * mass * 0.01 * 0.01 / 5.0 # assume as sphere
    i.append(ET.Element('mass', {'value': str(mass)}))
    i.append(ET.Element('inertia', {'ixx': str(inertia), 'ixy': '0', 'ixz': '0', 'iyy': str(inertia), 'iyz': '0', 'izz': str(inertia)}))
    l.append(i)
    return(l)

def transmission(name):
    t = ET.Element('transmission', {'name': '{0}_trans'.format(name)})
    tp = ET.Element('type')
    tp.text = 'transmission_interface/SimpleTransmission'
    t.append(tp)
    j = ET.Element('joint', {'name': name})
    hw = ET.Element('hardwareInterface')
    hw.text = 'hardware_interface/EffortJointInterface'
    j.append(hw)
    t.append(j)
    a = ET.Element('actuator', {'name': '{0}_motor'.format(name)})
    hw2 = ET.Element('hardwareInterface')
    hw2.text = 'hardware_interface/EffortJointInterface'
    a.append(hw2)
    t.append(a)
    return(t)

if __name__ == '__main__':
    root = ET.fromstring(subprocess.check_output(sys.argv[1:]))

    root.append(link('odom'))
    root.append(link('odom_xt_link'))
    root.append(link('odom_yx_link'))
    root.append(odom_t_joint())
    root.append(odom_x_joint())
    root.append(odom_y_joint())
    #root.append(transmission('odom_x'))
    #root.append(transmission('odom_y'))
    #root.append(transmission('odom_t'))
    if sys.version_info.major == 3:
        print('<?xml version="1.0" ?>')
        print(ET.tostring(root, encoding='unicode'))
    else:
        print(ET.tostring(root, encoding='utf-8'))

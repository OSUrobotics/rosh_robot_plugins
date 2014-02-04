# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: camera.py 11372 2010-10-01 20:52:44Z kwc $

from __future__ import with_statement

PKG = 'rosh_common'
# we define this in the rosh_common package because the camera data
# representation is part of common_msgs

import os
import sys
import time

import roslib.names
import roslib.packages

import rosh #for get_topics()
import rosh.impl.proc
import rosh.impl.topic

from rosh.impl.namespace import Namespace, Concept
    
_TIMEOUT_CAM_INFO = 3

class Camera(Namespace):

    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance with additional 'listener' attribute. 
        @type  config: L{NamespaceConfig}
        """
        super(Camera, self).__init__(name, config)
        self._cam_info = None
            
    def _list(self):
        """
        Override Namespace._list()
        """
        try:
            pubs = self._config.master.getPublishedTopics(self._ns)
            candidates = [p[0] for p in pubs if p[0].endswith('/image_raw')]
            pub_names = [p[0] for p in pubs]
            matches = []
            for c in candidates:
                stem = c[:-len('image_raw')]
                if stem and stem+'camera_info' in pub_names:
                    matches.append(stem)
            return matches
        except:
            return []

    def __repr__(self):
        return self.__str__()
        
    def __str__(self):
        # TODO: camera info?
        if self._cam_info is None:
            self._init_cam_info()
        if self._cam_info is None:
            return self._ns
        else:
            return str(self._cam_info)

    def _init_cam_info(self):
        if self._ns == '/':
            return
        
        if self._cam_info is None:
            # this is a bit ugly. need cleaner/level-consistent API for rosh.impl.topic access
            try:
                ns_obj = rosh.get_topics()[roslib.names.ns_join(self._ns, 'camera_info')]
                self._cam_info = rosh.impl.topic.next_msg(ns_obj, _TIMEOUT_CAM_INFO)
            except:
                pass

    def __call__(self, *args, **kwds):
        """
        Poll image from camera.
        
        @return: current transform
        @rtype: L{Transform}
        """
        ns_obj = rosh.get_topics()[self._ns]
        return rosh.impl.topic.next_msg(ns_obj)

    def _show(self):
        return self._config.ctx.rosh_common__show_handler(self)

class Cameras(Concept):

    def __init__(self, ctx, lock):
        ctx.rosh_common__show_handler = show_camera_py_image_view
        super(Cameras, self).__init__(ctx, lock, Camera)

    def _register_show_handler(self, handler):
        self._config.ctx.rosh_common__show_handler = handler

def show_camera_py_image_view(ns_obj):
    """
    Default show camera handler that uses
    py_image_view. rosh_visualization provides an optimized rviz-based
    viewer.

    TODO: 
    @return: image viewer node (if single camera), or tuple of image viewer nodes (if stereo)
    @rtype: Node or (Node, Node)
    """

    if ns_obj._cam_info is None:
        # check to see if it's stereo
        l = ns_obj._getAttributeNames()
        if 'left' in l and 'right' in l:
            n1 = show_camera_py_image_view(ns_obj.left)
            n2 = show_camera_py_image_view(ns_obj.right)
            return n1, n2
        else:
            # NOTE: this will block
            try:
                ns_obj._init_cam_info()
            except:
                print >> sys.stderr, "%s does not appear to be a camera topic"%ns_obj._name
                return None
            
    # image_view doesn't die when the user closes the window, use py_image_view or rviz instead.
    # execute in subprocess to keep wx out of this VM

    #TODO: convert to Node
    #TODO: pass in height/width from cam info into py_image_view for default window size
    d = roslib.packages.get_pkg_dir(PKG)
    mod = os.path.join(d, 'src', PKG, 'py_image_view.py')
    cmd = ['python', mod, '-t', roslib.names.ns_join(ns_obj._name, 'image_raw')]
    rosh.impl.proc.run(ns_obj._config, cmd)
    print "running py_image_viewer, this may be slow over a wireless network"
    return True

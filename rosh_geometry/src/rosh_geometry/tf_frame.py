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
# Revision $Id: tf_frame.py 13189 2011-02-11 21:42:57Z kwc $

from __future__ import with_statement

import os
import sys

import roslib.message
import roslib.packages

import rospy

_error = None
try:
    import geometry_msgs.msg
except ImportError:
    _error = "cannot import geometry_msgs, tf integration will be disabled"
try:
    import tf
except ImportError:
    _error = "cannot import tf, tf integration will be disabled"
    
# fairly stock plugin imports
from rosh.impl.exceptions import InvalidPlugin
from rosh.impl.namespace import Namespace, Concept

# use rosh's process manager
import rosh.impl.proc
    
from rosh_geometry.geometry import PoseStamped, Point, Quaternion

class TFFrame(Namespace):

    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance with additional 'listener' attribute. 
        @type  config: L{NamespaceConfig}
        """
        super(TFFrame, self).__init__(name, config)

    def _list(self):
        """
        Override Namespace._list()
        """
        try:
            return self._config.listener.getFrameStrings()
        except:
            return []

    def _getAttributeNames(self):
        # strip off the leading slash in the ns since frame names are returned without it
        ns = self._ns.lstrip('/')

        # filter by frames that are in the current ns
        return set([s[len(ns):].split('/')[0].strip('/') for s in self._list() if s.startswith(ns)])

    def __repr__(self):
        return self.__str__()
        
    def __str__(self):
        return self._name

    def __call__(self, target):
        """
        @return: current transform
        @rtype: L{Transform}
        """
        translation, rotation = self._config.listener.lookupTransform(self._name, target, rospy.Time(0))
        return PoseStamped(Point(*translation), Quaternion(*rotation), target)
    
class Transform(object):
    def __init__(self, translation, rotation):
        self.translation = translation
        self.rotation = rotation

    def __repr__(self):
        return self.__str__()
    
    def __str__(self):
        return "- Translation: %s\n- Rotation: %s"%(self.translation, self.rotation)

class TFFrames(Concept):

    def __init__(self, ctx, lock):
        if _error:
            raise InvalidPlugin(_error)
        super(TFFrames, self).__init__(ctx, lock, TFFrame)
        # TODO: lazy-init on attribute access
        self._config.listener = tf.TransformListener()

    def __setattr__(self, key, value):
        if key.startswith('_'):
            return object.__setattr__(self, key, value)
        else:
            return self._root.__setitem__(key, value)

    def _show(self):
        show_concept(self)

    def __call__(self, obj, frame_id):
        l = self._config.listener
        if hasattr(obj, '_transform'):
            return obj._transform(l, frame_id)
        elif isinstance(obj, roslib.message.Message):

            #TODO: delegate this to rosh_geometry.geometry

            # another possibility is to add the _transform method to the classes
            if isinstance(obj, geometry_msgs.msg.PointStamped):
                return l.transformPoint(frame_id, obj)
            elif isinstance(obj, geometry_msgs.msg.PoseStamped):
                return l.transformPose(frame_id, obj)
            elif isinstance(obj, geometry_msgs.msg.QuaternionStamped):
                return l.transformQuaternion(frame_id, obj)
            elif isinstance(obj, geometry_msgs.msg.Vector3Stamped):
                return l.transformVector3(frame_id, obj)
            else:
                raise ValueError("message is not transformable")
        else:
            raise ValueError("object is not transformable")

def show_concept(tf_frames):
    dotcode = tf_frames._config.listener.allFramesAsDot()
    d = roslib.packages.get_pkg_dir('rosh')
    mod = os.path.join(d, 'src', 'rosh', 'impl', 'xdot.py')

    rosh.impl.proc.run(tf_frames._config, ['python', mod, '--raw', dotcode])

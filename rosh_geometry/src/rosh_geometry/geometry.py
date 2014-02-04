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
# Revision $Id: geometry.py 13195 2011-02-13 20:09:44Z kwc $

"""
Convenience classes for using code from the ROS geometry stack.
"""

import rospy

_error = None
try:
    import geometry_msgs.msg
except ImportError:
    _error = "cannot import geometry_msgs.msg, geometry integration will be disabled"

if not _error:    
    try:
        import tf
        import tf.posemath
    except ImportError:
        _error = "cannot import tf, geometry integration will be disabled"

# API review suggested this being in geometry, but no consensus on what the correct API is
def header(stamp=None, frame_id=''):
    """
    Construct a std_msgs/Header instance.  The timestamp defaults to
    the current ROS time if no stamp is specified (or None).
    """
    if stamp is None:
        return rospy.Header(stamp=rospy.get_rostime(), frame_id=frame_id)
    else:
        return rospy.Header(stamp=stamp, frame_id=frame_id)
    
#TODO: what's the best default frame_id?
_DEFAULT_FRAME_ID = 'base_link'

class Point(geometry_msgs.msg.Point):
    def __init__(self, x=0., y=0., z=0.):
        geometry_msgs.msg.Point.__init__(self, x, y, z)

class Quaternion(geometry_msgs.msg.Quaternion):
    def __init__(self, x=0., y=0., z=0., w=1.):
        geometry_msgs.msg.Quaternion.__init__(self, x, y, z, w)

class QuaternionStamped(geometry_msgs.msg.Quaternion):
    def __init__(self, x=0., y=0., z=0., w=1., frame_id=_DEFAULT_FRAME_ID, stamp=None):
        """
        @param stamp: Header timestamp. defaults to rospy.Time(0)
        """
        geometry_msgs.msg.QuaternionStamped.__init__(self, None, geometry_msgs.msg.Quaternion(x, y, z, w))
        self.header.frame_id = frame_id
        if stamp is None:
            self.header.stamp = rospy.Time(0)
        else:
            self.header.stamp = stamp            

    def _transform(self, listener, target):
        """
        API for auto-transforming via ROSH tf bindings.
        """
        return listener.transformQuaternion(target, self)
        
class PointStamped(geometry_msgs.msg.PointStamped):
    def __init__(self, x=0., y=0., z=0., frame_id=_DEFAULT_FRAME_ID, stamp=None):
        """
        @param stamp: Header timestamp. defaults to rospy.Time(0)
        """
        geometry_msgs.msg.PointStamped.__init__(self, None, geometry_msgs.msg.Point(x, y, z))
        self.header.frame_id = frame_id
        if stamp is None:
            self.header.stamp = rospy.Time(0)
        else:
            self.header.stamp = stamp            
            
    def _transform(self, listener, target):
        """
        API for auto-transforming via ROSH tf bindings.
        """
        return listener.transformPoint(target, self)

class Vector3Stamped(geometry_msgs.msg.Vector3Stamped):
    def __init__(self, x=0., y=0., z=0., frame_id=_DEFAULT_FRAME_ID, stamp=None):
        """
        @param stamp: Header timestamp. defaults to rospy.Time(0)
        """
        geometry_msgs.msg.Vector3Stamped.__init__(self, None, geometry_msgs.msg.Vector3(x, y, z))
        self.header.frame_id = frame_id
        if stamp is None:
            self.header.stamp = rospy.Time(0)
        else:
            self.header.stamp = stamp            

    def _transform(self, listener, target):
        """
        API for auto-transforming via ROSH tf bindings.
        """
        return listener.transformVector3(target, self)
    
class PoseStamped(geometry_msgs.msg.PoseStamped):

    def __init__(self, position=None, orientation=None, frame_id=_DEFAULT_FRAME_ID, stamp=None):
        """
        @param stamp: Header timestamp. defaults to rospy.Time(0)
        """
        geometry_msgs.msg.PoseStamped.__init__(self)
        self.header.frame_id = frame_id
        if stamp is None:
            self.header.stamp = rospy.Time(0)
        else:
            self.header.stamp = stamp            

        if position is not None:
            p = self.pose.position
            p.x = position.x
            p.y = position.y
            p.z = position.z

        if orientation is None:
            self.pose.orientation.w = 1
        else:
            o = self.pose.orientation
            o.x = orientation.x
            o.y = orientation.y
            o.z = orientation.z
            o.w = orientation.w            
            
    def _transform(self, listener, target):
        """
        API for auto-transforming via ROSH tf bindings.
        """
        return listener.transformPose(target, self)
    
    @staticmethod
    def from_2d(x=0., y=0., th=0., frame_id=_DEFAULT_FRAME_ID):
        """
        Create PoseStamped instance from x, y, theta
        """
        pose = tf.posemath.fromEuler(x, y, 0., 0., 0., th).toMsg()
        ps = PoseStamped(frame_id=frame_id)
        ps.pose = pose
        return ps

    def to_2d(self):
        """
        Convert pose to x, y, theta. This removes any z elements as well as roll and pitch.
        @return: (x, y, rz)
        """
        pm = tf.posemath.PoseMath(self)
        x, y, z, rx, ry, rz = pm.asEuler()
        return x, y, rz

    def to_euler():
        """
        Convert pose to x, y, z, rx, ry, rz. 
        @return: (x, y, z, rx, ry, rz)
        """
        pm = tf.posemath.PoseMath(self)
        return pm.asEuler()



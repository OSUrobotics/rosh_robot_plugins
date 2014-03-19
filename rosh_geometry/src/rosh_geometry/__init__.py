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
# Revision $Id: __init__.py 13195 2011-02-13 20:09:44Z kwc $

import roslib; roslib.load_manifest('rosh_geometry')

import rosh.plugin
from rosh_geometry.tf_frame import TFFrames
from rosh_geometry.geometry import Point, Quaternion, PointStamped, PoseStamped, QuaternionStamped, Vector3Stamped, header

_loaded_symbols = None
def rosh_plugin_load(plugin_context, globals_=None):
    """
    Initialize rosh_geometry plugin
    """
    global _loaded_symbols
    if rosh.plugin.reentrant_load(_loaded_symbols, globals_):
        return

    try:
        #TODO: log this somewhere
        
        # do not initialize if a master is not active. we should also
        # test for the presence of an initialized node.
        if not plugin_context.ctx.master.is_online():
            print 'Cannot load geometry without roscore running.'
            return
    except:
        return
        
    # NOTE: this can exception out if tf is broken. Should we catch/recast it?
    _loaded_symbols = {'transforms' : TFFrames(plugin_context.ctx, plugin_context.rosh_lock),
                       'header': header,
                       'Point' :Point,
                       'Quaternion': Quaternion,
                       'PointStamped': PointStamped,
                       'PoseStamped': PoseStamped,
                       'QuaternionStamped': QuaternionStamped,
                       'Vector3Stamped': Vector3Stamped
      }
    rosh.plugin.globals_load(plugin_context, globals_, _loaded_symbols)

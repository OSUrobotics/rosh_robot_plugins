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
# Revision $Id: __init__.py 11149 2010-09-18 04:17:59Z kwc $

import roslib; roslib.load_manifest('rosh_common')

import rosh.plugin
from rosh_common.action import Actions
from rosh_common.camera import Cameras

_loaded_symbols = None

PLUGIN_CAMERAS_SHOW = 'plugin-rosh_common-cameras_show'

def rosh_plugin_load(plugin_context, globals_=None):
    """
    Initialize rosh_common plugin
    """
    global _loaded_symbols
    if rosh.plugin.reentrant_load(_loaded_symbols, globals_):
        return

    cameras = Cameras(plugin_context.ctx, plugin_context.rosh_lock)
    _loaded_symbols = {
        'actions': Actions(plugin_context.ctx, plugin_context.rosh_lock),
        'cameras': Cameras(plugin_context.ctx, plugin_context.rosh_lock)
        }
    rosh.plugin.globals_load(plugin_context, globals_, _loaded_symbols)

    plugin = rosh.plugin.PluginData()
    plugin.add_api(PLUGIN_CAMERAS_SHOW, cameras._register_show_handler)
    #TODO: register nav_msgs handler
    #plugin.add_handler()
    return plugin


#TODO: convert to plugin
def show_occ_grid(ns_obj):
    # internal py_image_view can convert occ grids to images
    mod = os.path.join(get_pkg_dir('rosh'), 'src', 'rosh', 'impl', 'py_image_view.py')
    #TODO: convert to launch instead
    rosh.impl.proc.run(ns_obj._config, ['python', mod, '--map', '-t', ns_obj._name])
    print "running py_image_viewer, this may be slow over a wireless network"


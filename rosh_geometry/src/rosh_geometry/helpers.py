# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Oregon State University
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

import genpy

def _hasattrs(obj, attrs):
    return all(hasattr(obj, attr) for attr in attrs)

def _children_hasattrs(obj, attrs):
    for child in dir(obj):
        if not child.startswith('_'):
            if _hasattrs(obj.__getattribute__(child), attrs):
                return True, obj.__getattribute__(child)
    return False, None

def _search_attr(obj, attrs):
    if _hasattrs(obj, attrs):
        return True, obj
    for child in dir(obj):
        if not child.startswith('_'):
            childobj = obj.__getattribute__(child)
            if issubclass(type(childobj), genpy.message.Message):
                found, found_obj = _search_attr(childobj, attrs)
                if found:
                    return found, found_obj
    return False, None

def setSeq(obj, **kwargs):
    found, unstamped = _search_attr(obj, kwargs.keys())
    if found:
        for k, v in kwargs.iteritems():
            unstamped.__setattr__(k, v)
    else:
        raise TypeError('%s and its children do not have the correct attributes' % type(obj))

def setXYZ(obj, x=0.0, y=0.0, z=0.0):
    setSeq(obj, x=x, y=y, z=z)

def setQuat(obj, x=0.0, y=0.0, z=0.0, w=1.0):
    setSeq(obj, x=x, y=y, z=z, w=w)
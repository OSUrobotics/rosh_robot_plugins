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
# Revision $Id: action.py 11460 2010-10-08 05:21:05Z kwc $

from __future__ import with_statement

import time

import roslib.packages
import rosmsg
import rospy

import actionlib
import rostopic

from rosh.impl.namespace import Namespace, Concept
import rosh.impl.msg
import rosh.impl.topic
    
class Container(object):
    def __init__(self):
        self.obj = None
    def __call__(self, arg):
        self.obj = arg
        
#TODO: wait for result?
class Goal(object):
    
    def __init__(self, topic, goal_id):
        self.topic = topic
        self.goal_id = goal_id
    
    def __repr__(self):
        return self.goal_id.id
    
    #def result(self):
    #    return self.topic.result()
        
    def cancel(self):
        # Contract of actionlib is id kills id, adding a stamp kills
        # all goals prior to stamp.
        self.topic.cancel(id=self.goal_id.id)
    
class Action(Namespace):

    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance with additional 'listener' attribute. 
        @type  config: L{NamespaceConfig}
        """
        super(Action, self).__init__(name, config)
        self._type = self._type_name = None
        self._goal = None

        self._init_action()

    def _get_type(self):
        "rostype() API"
        return self._type

    def status(self):
        base_topic = self._config.ctx.topics[self._name]
        status_topic = base_topic['status']
        # this is an unstable API
        c = Container()
        with rosh.impl.topic.subscribe(status_topic, c):
            while c.obj is None:
                time.sleep(0.001)
        return c.obj
    
    def goals(self):
        s = self.status()
        # type is actionlib_msgs/GoalStatusArray
        base_topic = self._config.ctx.topics[self._name]
        return [Goal(base_topic, s.goal_id) for s in s.status_list]

    def __getitem__(self, key):
        if key in ['goals', 'status']:
            return object.__getattribute__(self, key)
        else:
            return super(Action, self).__getitem__(key)
        
    def _list(self):
        """
        Override Namespace._list()
        """
        try:
            # feedback may become optional in the future, so we don't
            # match against it

            #TODO: setup caching policy for getPublishedTopics
            pubs = self._config.master.getPublishedTopics(self._ns)
            candidates = [p[0] for p in pubs if p[0].endswith('/status')]
            pub_names = [p[0] for p in pubs]
            matches = []
            for c in candidates:
                stem = c[:-len('status')]
                if stem and stem+'result' in pub_names:
                    matches.append(stem)
            if self._name and matches:
                return matches + ['/goals', '/status']
            else:
                return matches
        except:
            return []

    def __repr__(self):
        return self.__str__()
        
    def __str__(self):
        # re-initialize
        if self._type is None and self._type_name is not None:
            self._init_action()
            
        if self._type is not None:        
            return rosmsg.get_msg_text(self._goal._type)
        else:
            return self._ns

    def _init_action(self):
        if self._ns == '/':
            return
        if self._type is None:
            #TODO: shift away from feedback, use status instead
            feedback_topic = roslib.names.ns_join(self._ns, 'feedback')
            feedback_type_name = rostopic.get_topic_type(feedback_topic, blocking=False)[0]
            if feedback_type_name and feedback_type_name.endswith('Feedback'):
                self._type_name = feedback_type_name[:-len('Feedback')]
                if not self._type_name.endswith('Action'):
                    # abort init
                    self._type_name = None
                else:
                    # rosh's loader has extra toys
                    self._type = rosh.impl.msg.get_message_class(self._type_name)
                    if self._type is not None:
                        # only load if the previous load succeeded
                        self._goal = rosh.impl.msg.get_message_class(self._type_name[:-len('Action')]+'Goal')

    def __call__(self, *args, **kwds):
        """
        @return: current transform
        @rtype: L{Transform}
        """
        if self._type is None:
            self._init_action()
        goal = self._goal(*args, **kwds)
        client = actionlib.SimpleActionClient(self._name[1:], self._type)
        client.wait_for_server()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

class Actions(Concept):

    def __init__(self, ctx, lock):
        super(Actions, self).__init__(ctx, lock, Action)


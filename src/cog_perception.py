#! /usr/bin/env python

"""
    Created by Bradley Sheneman
    Uses new functionality allowing local functions to be used in GroundedSchemaNode
    and evaluation without connecting to CogServer
    Sets up a basic ROS node 

4.10.2015:
    add the part to receive visibility message and make action if the specific block opencog can see.

"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import controller_msg,visibility_msg

from opencog.atomspace import AtomSpace, types
from opencog.utilities import initialize_opencog, finalize_opencog
import opencog.scheme_wrapper as scheme
from opencog.scheme_wrapper import load_scm, scheme_eval


class SimpleAgent():

    def __init__(self):
        self.a = AtomSpace()
        self.nodes = {}
        
        # Initialize Scheme
        scheme_preload = [  
                    "opencog/atomspace/core_types.scm",
                    "opencog/scm/utilities.scm"          ]
        scheme.__init__(self.a)
        for scheme_file in scheme_preload:
            load_scm(self.a, scheme_file)
        initialize_opencog(self.a)
        
        #add 3 nodes with integer values
        self.nodes[0] = self.a.add(types.ConceptNode, "0")
        self.nodes[1] = self.a.add(types.ConceptNode, "1")
        self.nodes[2] = self.a.add(types.ConceptNode, "2")

    def search_block_in_view(self,data):
        #Test block is sand, which block id is 12:
        SAND=12
        #randomly select a link from those available and add the nodes
        for x in data.visibleblocks:
            if x.btype==SAND:
                print 'SAND!'
                fnode = self.a.add_node(types.GroundedSchemaNode, "py: sendValue")
                current_link = self.a.add_link(types.ExecutionOutputLink, [
                    fnode,
                    self.a.add_link(types.ListLink, [self.nodes[1]])])
        
                scheme_eval(self.a, '(cog-execute! (cog-atom %d))'%(current_link.handle_uuid()))
                break

    def remove(self):
        # make sure this is called by the time script exits
        finalize_opencog()
        del self.a
    
def mainloop():
    global spock_pub

    # Initialize ROS node
    rospy.init_node('opencog_perception')

    # Create publisher to send out ControllerMsg messages
    Agent = SimpleAgent()
    spock_pub = rospy.Publisher('controller_data', controller_msg)
    rospy.Subscriber('visibility_data',visibility_msg,Agent.search_block_in_view)    
    
    while not rospy.is_shutdown():
        rospy.sleep(1.)
    
    Agent.remove()


def sendValue(atom):
    message = controller_msg()
    message.action = int(atom.name)
    spock_pub.publish(message)
    print("published action: %d" %(message.action))
    return atom



if __name__ == "__main__":
    mainloop()

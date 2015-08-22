import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import map_block_msg, position_msg, map_block_multi_msg
from minecraft_bot.srv import visible_blocks_srv

subscribed_msg_dict = {"client_position_data" : position_msg,
                       "camera_vis_data" : map_block_multi_msg}

class ROSPerceptionInterface:
    
    def __init__(self, _ros_handle_dict):

        for topic in subscribed_msg_dict:
            print topic
            print _ros_handle_dict
            #print _ros_handle_dict[topic]
            print subscribed_msg_dict
            try:
                rospy.Subscriber(topic, subscribed_msg_dict[topic], _ros_handle_dict[topic])
            except KeyError, e:
                print "ROSPerceptionProcessor.__init__: %s: topic %s is not in the handle_dict" %(e,topic)

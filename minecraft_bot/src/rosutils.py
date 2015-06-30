import rospy

def setMessage(msg, data):
    
    for key in data:
        if hasattr(msg, key):
            setattr(msg, key, data[key])

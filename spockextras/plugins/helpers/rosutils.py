import rospy

def setMessage(msg, data):
    
    for key in msg:
        if hasattr(data, key):
            if isinstance(data[key], dict):
                pass
            elif isinstance(data[key], list):
                pass
            else:
                setattr(msg, key, data[key])

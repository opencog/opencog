# 
# Test functions for test_bindlink.py -> test_satisfy

green = 0
red = 0

def initialize_counts():
    global red
    global green
    green = 0
    red = 0

def green_count():
    global green
    return green

def red_count():
    global red
    return red

def increment_green():
    global green
    green += 1

def increment_red():
    global red
    red += 1


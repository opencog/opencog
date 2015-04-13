from math import tanh


def getBeginning(dist):
    """
    Given a dictionary formed by <timestamp,certainty> and its ordered list
    of keys this function returns the beginning of it, i.e., the
    subinterval between the first timestamp with certainty 1 and the
    first timestamp from the beginning with certainty!=0
    This method assumes there's at least one timestamp with certainty=1
    it may work unexpectedly otherwise
    """
    #we will return a sub-distribution
    subdist = {}
    #obtain a sorted list of keys
    keys = dist.keys()
    ordered_keys = sorted(keys)
    #find the beginning
    found_start = False
    for t in ordered_keys[0:]:
        #the first certainty>0 is the start (first key) of the beginning
        if not (found_start) and dist[t] > 0:
            #firstkey1=t
            found_start = True
            #once we found the first, we keep adding them
        if found_start:
            subdist[t] = dist[t]
            #the first certainty==1 is the end (last key) of the beginning
        if dist[t] == 1: # and found_start (implicit)
            break

            #return firstkey1, secondkey1
    return subdist


def getEnding(dist):
    """
    Given a dictionary formed by <timestamp,certainty> and its ordered list
    of keys this function returns the beginning of it, i.e., the
    subinterval between the last timestamp with certainty 1
    and the first after that one with certainty=0
    This method assumes there's at least one timestamp with certainty=1
    it may work unexpectedly otherwise
    """
    #we will return a sub-distribution
    subdist = {}
    #obtain a sorted list of keys
    keys = dist.keys()
    ordered_keys = sorted(keys)
    #find the end, starting from the last
    found_start = False
    for t in reversed(ordered_keys[:]):
        #the first certainty>0 is the last (second key) of the ending
        if not (found_start) and dist[t] > 0:
            #secondkey2 = t
            found_start = True
            #once we found the first, we keep adding them
        if found_start:
            subdist[t] = dist[t]
            #the first certainty==1 is the start (first key) of the ending
        if dist[t] == 1: # and found_start (implicit)
            #firstkey2=t
            break

    #return firstkey2,secondkey2
    return subdist


def getSize(dist):
    """
    Given a dictionary formed by <timestamp,certainty> this function returns the size of it, i.e., the
    difference between the fist timestamp and the last
    This method assumes there's at least one timestamp in the dictionary
    """
    if len(dist) == 0:
        return 0
    keys = dist.keys()
    ordered_keys = sorted(keys)
    firsttime = ordered_keys[0]
    ordered_keys.reverse()
    lasttime = ordered_keys[0]
    return lasttime - firsttime


def calculateCenterMass(dist):
    """
    this function calculates the centers of mass of a temporal distribution
    it requires a distribution in the form of a dictionary <timestamp,certainty>
    it returns the two coordinates of the center of mass, the timestamp and the certainty
    """
    #obtain an ordered list of the keys
    keys = dist.keys()
    ordered_keys = sorted(keys)

    total_area = 0
    centroid_t = 0
    centroid_c = 0

    size_times = len(ordered_keys)
    #Special case if there's only one point
    if size_times == 1:
        return ordered_keys[0], dist[ordered_keys[0]] / float(2)

    init_segment = ordered_keys[0]

    for t in ordered_keys[1:]:
        end_segment = t
        #If they are equal its a rectangle
        #we calculate its area, and its centroid ponderated by the area
        #if they are both zero, there's no area
        if dist[init_segment] == dist[end_segment] and dist[init_segment] != 0:
            area_rectangle = (end_segment - init_segment) * dist[init_segment]
            total_area = total_area + area_rectangle
            centroid_t += area_rectangle * ((end_segment + init_segment) / float(2))
            centroid_c += area_rectangle * (dist[init_segment] / float(2))
        #If they are different but one is zero, its a triangle rectangle
        #we calculate its area, and its centroid ponderated by the area
        elif dist[init_segment] != dist[end_segment] and dist[init_segment] == 0:
            area_triangle = (dist[end_segment] * (end_segment - init_segment)) / float(2)
            total_area += area_triangle
            centroid_t += area_triangle * ((((end_segment - init_segment) / float(3)) * 2) + init_segment)
            centroid_c += area_triangle * (dist[end_segment] / float(3))
        elif dist[init_segment] != dist[end_segment] and dist[end_segment] == 0:
            area_triangle = (dist[init_segment] * (end_segment - init_segment)) / float(2)
            total_area += area_triangle
            centroid_t += area_triangle * (((end_segment - init_segment) / float(3)) + init_segment)
            centroid_c += area_triangle * (dist[init_segment] / float(3))
        #If both are different and none is zero its the most complex case
        #it is both a rectangle and a triangle
        elif dist[init_segment] != dist[end_segment]:
            if dist[init_segment] > dist[end_segment]:
                rectangle_height = dist[end_segment]
                triangle_height = dist[init_segment] - dist[end_segment]
            else:
                rectangle_height = dist[init_segment]
                triangle_height = dist[end_segment] - dist[init_segment]
                #first we calculate the rectangle
            area_rectangle = rectangle_height * (end_segment - init_segment)
            total_area = total_area + area_rectangle
            centroid_t += area_rectangle * ((end_segment + init_segment) / float(2))
            centroid_c += area_rectangle * (rectangle_height / float(2))
            #then we calculate the triangle. The method depends on which side is the right angle
            if dist[end_segment] > dist[init_segment]:
                area_triangle = (triangle_height * (end_segment - init_segment)) / float(2)
                total_area += area_triangle
                centroid_t += area_triangle * (
                    (((end_segment - init_segment) / float(3)) * 2) + init_segment)
                centroid_c += area_triangle * (
                    ((dist[end_segment] - dist[init_segment]) / float(3)) + dist[init_segment])
            else:
                area_triangle = (triangle_height * (end_segment - init_segment)) / float(2)
                total_area += area_triangle
                centroid_t += area_triangle * (((end_segment - init_segment) / float(3)) + init_segment)
                centroid_c += area_triangle * (
                    ((dist[init_segment] - dist[end_segment]) / float(3)) + dist[end_segment])
            #we move forward
        init_segment = end_segment

    return centroid_t / float(total_area), centroid_c / float(total_area)

# arbitrary
NORMALIZE_FACTOR = 0.5


def normalize(diff):
    """
    Takes the scaled difference between (the centers of mass of) two intervals.
    Uses an (arbitrary) formula to normalize it to a fuzzy truth value between 0 and 1.
    A higher diff produces a fuzzy TV closer to 1
    """

    return (1 - tanh(NORMALIZE_FACTOR * diff)) / 2


def reverse_normalize(diff):
    """
    Normalizes a diff in the opposite direction. A higher diff produces a fuzzy TV closer to 0.
    """

    return tanh(NORMALIZE_FACTOR * diff) / 2

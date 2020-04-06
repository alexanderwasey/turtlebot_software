import math
import rospy
from sensor_msgs.msg import LaserScan


def laser_scan_callback(data):
    global currlaser
    currlaser = data

def is_clear_in_front(): 
    if (currlaser != []):
        return collision_in_box(currlaser, math.pi, SCAN_DEPTH, SCAN_WIDTH)
    else: 
        print("Laser Data Missing")
        return False

def is_clear_behind(): 
    if (currlaser != []):
        return collision_in_box(currlaser, 0, SCAN_DEPTH, SCAN_WIDTH)
    else: 
        return False

def is_left_turn_clear(): 
    if (currlaser != []):
        return collision_in_box(currlaser, 0.5*math.pi, 0.35, SCAN_WIDTH)
    else: 
        return False

def is_right_turn_clear(): 
    if (currlaser != []):
        return collision_in_box(currlaser, 1.5*math.pi, 0.35, SCAN_WIDTH)
    else: 
        return False

def get_ranges(data, minang, maxang): 
    ranges = get_in_angle(data.ranges, minang, maxang, data.angle_min, data.angle_increment)
    ranges = filter_invalid(ranges, data.range_min, data.range_max)
    return ranges

def get_in_angle(ranges, minang, maxang, startang, increment): 
    #Need to ensure always above 0
    while (minang < 0): 
        minang += 2*math.pi
    
    currang = startang
    values = list()
    for dist in list(ranges): 
        if ((maxang - minang) < 0): #In the case that the angle range includes 0 
            if (((currang < maxang) and (currang > 0)) or ((currang > minang) and (currang < (2*math.pi)))): 
                values.append((dist, currang))
        else: 
            #Otherwise
            if ((currang < maxang) and (currang > minang)):
                values.append([dist, currang])

        currang += increment
        if (currang > 2*math.pi): 
            currang -= 2*math.pi

    return values

def filter_invalid(ranges, minrange, maxrange): 
    final = list()
    minrange = max(0.17, minrange) #Eliminate the smallest values
    for x in ranges: 
        if ((x[0] > minrange) and (x[0] < maxrange)): 
            final.append(x)

    return final

def clear_in_box(scanpoints, direction, depth, width):
    for point in scanpoints: 
        dist = point[0]
        angle = min((2 * math.pi) - abs(point[1] - direction), abs(point[1] - direction))

        y = math.sin(angle) * dist
        #Only bother calculating the depth if we are within the box sides
        if (y < width / 2): 
            x = math.cos(angle) * dist #Calc if in the depth we need
            if ((x < depth) and (x > 0.17)): #Only if not one of the supports
                #print(x, point[2] * 180/ math.pi) This causes crashes
                return False #If so then there is something within the box
    
    #In the box
    return True

def collision_in_box(data, direction, depth, width):
    #Calculate the min and angle to bother looking at
    minang = direction - 0.5*math.pi
    if (minang < 0): 
        minang += 2*math.pi
    maxang = direction + 0.5*math.pi
    if (maxang > 2*math.pi):
        maxang -= 2*math.pi

    #Get the ranges and angles that can possibly be within the box
    scanpoints = get_in_angle(data.ranges, minang, maxang, data.angle_min, data.angle_increment)

    #Get the ranges that are valid
    scanpoints = filter_invalid(scanpoints, data.range_min, data.range_max)

    return clear_in_box(scanpoints, direction, depth, width)


def calc_distances(scanpoints, direction, width): 
    distances = list()
    for point in scanpoints: 
        dist = point[0]
        angle = min((2 * math.pi) - abs(point[1] - direction), abs(point[1] - direction))

        y = math.sin(angle) * dist

        if (y < width / 2):
            x = math.cos(angle) * dist
            distances.append(x)

    return distances


def get_distances_in_direction(direction, width):
    #Calculate the min and angle to bother looking at
    minang = direction - 0.5*math.pi
    if (minang < 0): 
        minang += 2*math.pi
    maxang = direction + 0.5*math.pi
    if (maxang > 2*math.pi):
        maxang -= 2*math.pi

    #Set the data to be static for this run 
    data = currlaser

    #Get the ranges and angles that can possibly be within the box
    scanpoints = get_in_angle(data.ranges, minang, maxang, data.angle_min, data.angle_increment)

    #Get the ranges that are valid
    scanpoints = filter_invalid(scanpoints, data.range_min, data.range_max)

    return calc_distances(scanpoints, direction, width)

def get_closest_in_direction(direction, width): 
    return min(get_distances_in_direction(direction, width) + [100])

def get_closest_in_front(width): 
    return get_closest_in_direction(math.pi, width)


rospy.Subscriber('scan',LaserScan, laser_scan_callback)
currlaser = []
SCAN_WIDTH = 0.35 #This should be *CONSTANT*
SCAN_DEPTH = 0.3 #For forward & backward motion - Gives a stopping distance of approx 10cm from object in worse case

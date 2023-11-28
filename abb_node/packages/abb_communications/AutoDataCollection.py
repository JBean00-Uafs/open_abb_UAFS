import abb
import time
from pyquaternion import Quaternion
import math
import random
import csv
import os





class moveableObj:
    """A class for movable objects in our workspace. Tracks X,Y,Z position and rotation of objects, and the tool they are used to interact with"""
    def __init__(self, initLoc, initIsRot, initRot, toolData, toolMinMax, tool_command, radius):

        #Tracks the current location of the cylinder in the world plane as [X, Y, Z] and its rotation relative to the quaternion of it's tool [q1,q2,q3,q4]
        self.cartLoc = initLoc

        #Tracks the degree and distance value given by the random number generator [deg, distance]
        self.polarLoc = [0.0,120.0]

        #Bool value that determines if object should have random rotational values
        self.isRot = initIsRot

        #relative to the world, the degree value of rotation with respect to the XY plane
        self.rot = initRot

        #Establishes the tool used to interact with object
        self.tool = toolData

        #Stores the min and max polar coord value for the given tool
        self.toolMinMax = toolMinMax

        #Refers to the Set_Do command associated with the tool
        self.activate_tool = tool_command

        #Refers to the loose radial bounding box around the centerpoint of the object
        self.radialEdge = radius

class metricTracker:
    """A Class for stat tracking and time calculation"""
    def __init__(self, csv_filename="metric_log.csv"):
        self.start_time = 0
        self.loop_time = 0
        self.total_time = 0
        self.iteration_count = 0
        self.fastest_time = float('inf')
        self.slowest_time = 0
        self.csv_filename = csv_filename
        self.create_csv_file()

    def create_csv_file(self):
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Iteration", "Previous Time", "Average Time", "Fastest Time", "Slowest Time", "Total Time"])

    def start_timer(self):
        self.start_time = time.time()

    def end_timer(self):
        end_time = time.time()
        self.loop_time = end_time - self.start_time
        self.update_metrics(self.loop_time)

    def update_metrics(self, loop_time):
        self.total_time += loop_time
        self.iteration_count += 1
        self.fastest_time = min(self.fastest_time, loop_time)
        self.slowest_time = max(self.slowest_time, loop_time)

    def get_average_time(self):
        return self.total_time / self.iteration_count if self.iteration_count else 0

    def estimate_remaining_time(self, total_iterations):
        average_time = self.get_average_time()
        remaining_iterations = total_iterations - self.iteration_count
        return remaining_iterations * average_time

    def output_to_csv(self):
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(
                [self.iteration_count, self.loop_time, self.get_average_time(), self.fastest_time, self.slowest_time, self.total_time])

    def format_time(self, seconds):
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        seconds = seconds % 60
        return f"{int(hours)}h {int(minutes)}m {int(seconds)}s"



## GLOBAL VARS ##

robIP = '127.0.0.1'

defaultSpeed = [800, 500, 5000, 1000]
defaultZone = [0,0,0]

#TOOLS
#tGripper = [[1.1,11.5,139.6],[-0.70711,0.70711,.70711,.70711]]
tGripper = [[1.1,11.5,139.6],[0.5,-0.5,-0.5,-0.5]]
tGripper_adv = [1,[[13.941,-25.105,74.149],[1,0,0,0]]]

tVac = [[1.58, -47.25, 24.06],[0.5,-0.5,-0.5,-0.5]]
tVac_adv = [1,[[13.941,-25.105,74.149],[1,0,0,0]]]

#Manual Joint rotations
jHome = [0,45,45,0,-90,90]
jZero = [0,0,0,0,0,0]
jOffScreen = [90,0,0,0,0,0]

#Placeholders, need to set these based on physical location
cylinderStartPos = [[-337.2, 581.91, 100.0], [0.5, 0.0, 0.0, 0.866]]
squareStartPos = [[-179.9, 308.39, 0], [0.5,0.0,0.0,0.866]]

#Program Parameters
picCount = 20

#Spacing and Overlap Parameters
"""
General Sizing Notes:
Table = 48x49, 32 inches high (1219.2, 1244.6, 812.8 (mm))
Red Square = 7x7 (177.8 x 177.8)
cylinder whole = 2x6 (50.8 x 152.4)
non-top-hat-cylinder = 2x4 (50.8, 101.6)
top-hat-cylinder = 2x4 (50.8, 101.6)
arm = 27 high 31 wide

"""
armWidth = 124
#longMinDegreeSep = 11.2
#shortMinDegreeSep = 25

redSquareRadialEdge = 125.72
cylinderRadialEdge = 25.4
minSeperation = 10


#Random number generator parameters
#Determined by tool size and reachable position relative to the table.
#ROTATIONAL VALUES: [degree values (-90 - 90), distance from center (tool dependant)]

#tGripper - range of 275.00
tGripperRange = [375, 650]

#tVac - range of 335.00
tVacRange = [285, 620]

#Degree Values
degMin = -90.0
degMax = 90.0




def initRobot():
    # Primary Robot Interface
    R = abb.Robot(ip=robIP)

    R.set_speed(defaultSpeed)
    R.set_zone(point_motion=True)

    R.set_joints(jZero)
    R.set_tool_advanced(tGripper_adv)

    return R

def quatPlaneRotate(quat = [1, 0, 0, 0], deg = 0):
    q1 = Quaternion(quat)
    q2 = Quaternion(axis=[0.0, 0.0, 1.0], degrees=deg)

    q3 = q1 * q2

    return list(q3)
"""
def getRelativePos(pos, mod):
    newPos = pos[:]
    newPos[0] = newPos[0] + mod[0]
    newPos[1] = newPos[1] + mod[1]
    newPos[2] = newPos[2] + mod[2]

    return newPos

def getRelativeJoint(joint, mod):
    newJoint = joint[:]

    newJoint[0] = newJoint[0] + mod[0]
    newJoint[1] = newJoint[1] + mod[1]
    newJoint[2] = newJoint[2] + mod[2]
    newJoint[3] = newJoint[3] + mod[3]
    newJoint[4] = newJoint[4] + mod[4]
    newJoint[5] = newJoint[5] + mod[5]

    return newJoint
"""

def getRelativeVal(val, mod):
    newVal = val[:]
    for i in range(len(newVal)):
        newVal[i] += mod[i]

    return newVal

def getAdjustedVal(val, adj):
    newVal = val[:]
    for i in range(len(newVal)):
        if adj[i] != 0:
            newVal[i] = adj[i]

    return newVal

def getBaseAngle(loc = [200,200,0]):
    '''Input [X,Y,Z] location and return the angle necessary to align axis 1 with the object'''
    angle = 0
    if(loc[1] != 0):
        angle = math.degrees(math.atan(abs(loc[1])/abs(loc[0])))
        if(loc[1] < 0):
            angle = angle*(-1)


    return angle

def getRelativeJHome(R1, mod = [0,0,0]):
    R1.set_joints(jHome)
    cuCoord = R1.get_cartesian()

    R1.set_cartesian([getAdjustedVal(cuCoord[0],mod), cuCoord[1]])
    print(str(R1.get_cartesian()))
    return R1.get_cartesian()

def get_min_degree_seperation(stillObject):
    """
    Calculates and returns the angle of seperation necessary to avoid unwanted collisions between two move objects

    Parameters:
    stillObject: The object we are avoiding

    Returns:
    A tuple of two values representing the angle range to avoid
    """
    if stillObject.polarLoc[0] != 0:
        angle = math.degrees(math.atan((armWidth/2) / stillObject.polarLoc[0]))
    else:
        angle = 0
    return (stillObject.polarLoc[1] - angle, stillObject.polarLoc[1] + angle)

def random_value_with_exclusions(min_val, max_val, exclusions, decimal_places=2):
    """
    Generates a random value between min_val and max_val, excluding specified ranges.
    Exclusions should be a list of tuples, each tuple representing a range to exclude.
    """
    while True:
        value = random.uniform(min_val, max_val)
        value = round(value, decimal_places)
        if not any(lower <= value <= upper for lower, upper in exclusions):
            return value


def calculate_distance(coords_list1, coords_list2):
    """
    Calculates the Euclidean distance between two points represented by two lists of X and Y coordinates.

    Parameters:
    coords_list1: A list containing the X and Y coordinates of the first point [X1, Y1].
    coords_list2: A list containing the X and Y coordinates of the second point [X2, Y2].

    Returns:
    The Euclidean distance between the two points.
    """

    if len(coords_list1) != 2 or len(coords_list2) != 2:
        raise ValueError("Each list must contain exactly two elements representing X and Y coordinates.")

    x_diff = coords_list1[0] - coords_list2[0]
    y_diff = coords_list1[1] - coords_list2[1]

    distance = math.sqrt(x_diff ** 2 + y_diff ** 2)
    return distance

def polar_to_cartesian(r, theta):
    """
    Converts polar coordinates to Cartesian coordinates.

    Parameters:
    r: Radius (distance from the origin).
    theta: Angle in degrees from the X-axis (-90 to 90 degrees).

    Returns:
    A tuple (x, y) representing Cartesian coordinates.
    """

    # Convert theta from degrees to radians
    theta_rad = math.radians(theta)

    # Calculate x and y using the polar to Cartesian conversion formulas
    x = r * math.cos(theta_rad)
    y = r * math.sin(theta_rad)

    return [x, y]


def executeObjectRotation(R1, rotationObject, deg = 0.0):
    '''Executes robot movements to rotate movable objects'''

    #move robot to rotation work area
    R1.set_joints(getRelativeVal(jHome,[-120,0,0,0,0,0]))
    currentLoc = R1.get_cartesian()

    workspaceLoc = [[currentLoc[0][0], currentLoc[0][1], 0], currentLoc[1]]

    #Place rot object down
    R1.set_cartesian(workspaceLoc)
    rotationObject.activate_tool(0)
    R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, 50]), workspaceLoc[1]])


    #Determine rotation direction
    rotationDirection = 0
    if(rotationObject.rot <= deg):
        #clockwise rotation
        rotationDirection = 1
    else:
        #counterclockwise rotation
        rotationDirection = -1


    while(rotationObject.rot != deg):
        #repeate this loop until rotation of object is correct
        #turn at maximum 90 degree turns
        rotAmount = 0
        if(rotationDirection*(deg - rotationObject.rot) >= 90):
            #current degree is more than 90 degrees away, turn 90
            rotAmount = 90*rotationDirection
        else:
            rotAmount = deg - rotationObject.rot

        rotQuat = quatPlaneRotate(workspaceLoc[1], rotAmount)



        #do pickup
        R1.set_cartesian(workspaceLoc)
        rotationObject.activate_tool(1)
        R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, 50]), workspaceLoc[1]])

        #rotate
        R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, 50]), rotQuat])

        #place back
        R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, 0]), rotQuat])
        rotationObject.activate_tool(0)
        R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, 50]), rotQuat])
        R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, 50]), workspaceLoc[1]])

        #update object rotation
        rotationObject.rot += rotAmount

        #If object is correctly rotated, the while loop will end. Otherwise, it continues
    #pickup object again for next move
    R1.set_cartesian(workspaceLoc)
    rotationObject.activate_tool(1)
    R1.set_cartesian([getRelativeVal(workspaceLoc[0], [0, 0, 50]), workspaceLoc[1]])

def collectData():
    '''Method for initiating data collection for the data set. Takes image using webcam, records data to CSV'''

def main():
    #Init robot
    R = initRobot()

    #Init Metric Tracker
    metrics = metricTracker()

    #create mobile objects
    cylinder = moveableObj(cylinderStartPos,False, 0, [tGripper, tGripper_adv], tGripperRange, R.set_Do_Grip, cylinderRadialEdge)
    square = moveableObj(squareStartPos,True, 0, [tVac, tVac_adv], tVacRange, R.set_Do_Vacume, redSquareRadialEdge)
    objectList = [cylinder, square]

    getRelativeJHome(R, [0,0,250])


    for setCount in range(0, picCount):
        #Main Data Collection Loop - Runs once for every image in the dataset

        #In program data collection and time keeping
        metrics.start_timer()

        #for each moving object
        for moveObj in objectList:
            validPosition = False

            while not validPosition:
                ###### DETERMINE NEW RANDOM LOCATION ######
                #Pick random polar coord based on min and max tool values
                randDistance = random_value_with_exclusions(moveObj.toolMinMax[0], moveObj.toolMinMax[1], [], 3)

                #### Pick random polar angle while avoiding potential trouble angles ####

                excludedRanges = []

                #For every other movable object, determine problem angle ranges and add them to list
                for otherObj in objectList:
                    # for every object in the object list, determine all excluded ranges
                    if otherObj != moveObj:
                        excludedRanges.append(get_min_degree_seperation(otherObj))

                randAngle = random_value_with_exclusions(-90.0, 90.0, excludedRanges, 2)

                #convert Polar Coords to X,Y Coords

                randCartesianCoord = polar_to_cartesian(randDistance, randAngle)

                #determine if coord is the minimum distance from all other movable objects
                validPosition = True
                for otherObj in objectList:
                    # for every object in the object list
                    if otherObj != moveObj:
                        seperationDistance = calculate_distance(randCartesianCoord, [otherObj.cartLoc[0][0], otherObj.cartLoc[0][1]])
                        if seperationDistance < (minSeperation+otherObj.radialEdge+moveObj.radialEdge):
                            validPosition = False
                            break

            #With a valid set of coordinates...

            #set object tool
            R.set_tool(moveObj.tool[0])
            moveObj.activate_tool(0)

            #
            R.set_joints(getRelativeVal(jHome, [moveObj.polarLoc[1], 0, 0, 0, 0, 0]))

            #move to above objects current location
            R.set_cartesian([getAdjustedVal(moveObj.cartLoc[0],[0,0,250]), moveObj.cartLoc[1]])
            #lower
            R.set_cartesian(moveObj.cartLoc)
            #grab
            moveObj.activate_tool(1)
            #Raise
            R.set_cartesian([getAdjustedVal(moveObj.cartLoc[0], [0, 0, 250]), moveObj.cartLoc[1]])

            #If object has rotation variety, pick a random rotation from 0-359.99
            randRot = 0
            if moveObj.isRot:
                randRot = random_value_with_exclusions(0.0, 359.99, [], 2)
                executeObjectRotation(R, moveObj, randRot)



            #Rotate joint 1 to new position
            R.set_joints(getRelativeVal(jHome, [randAngle, 0, 0, 0, 0, 0]))
            #get updated cartesian rotation data
            newQuant = R.get_cartesian()[1]
            #above
            R.set_cartesian([[randCartesianCoord[0],randCartesianCoord[1], 250], newQuant])
            #lower
            R.set_cartesian([[randCartesianCoord[0],randCartesianCoord[1], moveObj.cartLoc[0][2]], newQuant])

            #Update Object Data Values for: cartLoc, polarLoc, rot
            moveObj.cartLoc = R.get_cartesian()
            moveObj.polarLoc = [randDistance, randAngle]
            moveObj.rot = randRot

            #drop
            moveObj.activate_tool(0)
            # Raise
            R.set_cartesian([getAdjustedVal(moveObj.cartLoc[0], [0, 0, 250]), moveObj.cartLoc[1]])

        ##All Move Objects have new positions##

        #Move Robot out of the way
        R.set_joints(jOffScreen)

        #Record Data
        collectData()

        #Lap Data Collection
        metrics.end_timer()
        metrics.output_to_csv()

        print(f"Average Time  : {metrics.get_average_time()}")
        print(f"Previous Loop : {metrics.loop_time}")
        print(f"Estimated Remaining Time: {metrics.format_time(metrics.estimate_remaining_time(picCount))}")
        #Continue to next image...

    R.set_joints(jZero)
    R.close()


main()


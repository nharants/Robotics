#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain = Brain()

# Robot configuration code
brain_inertial = Inertial()
potentiometerV2_h = PotentiometerV2(brain.three_wire_port.h)
potentiometerV2_a = PotentiometerV2(brain.three_wire_port.a)
motor_2 = Motor(Ports.PORT2, False)
bicep_motor_a = Motor(Ports.PORT1, False)
bicep_motor_b = Motor(Ports.PORT6, True)
bicep = MotorGroup(bicep_motor_a, bicep_motor_b)
servo_c = Servo(brain.three_wire_port.c)
distance_7 = Distance(Ports.PORT7)


# Wait for sensor(s) to fully initialize
wait(100, MSEC)

# generating and setting random seed
def initializeRandomSeed():
    wait(100, MSEC)
    xaxis = brain_inertial.acceleration(XAXIS) * 1000
    yaxis = brain_inertial.acceleration(YAXIS) * 1000
    zaxis = brain_inertial.acceleration(ZAXIS) * 1000
    systemTime = brain.timer.system() * 100
    urandom.seed(int(xaxis + yaxis + zaxis + systemTime)) 

# Initialize random seed 
initializeRandomSeed()

#endregion VEXcode Generated Robot Configuration
# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode EXP Python Project
# 
# ------------------------------------------

# Library imports
from vex import *

# Begin project code

# Bicep, Forearm Length
length1 = 12
length2 = 6

# Convert pot degrees to degrees relative to the bicep/forearm start
offset1 = 86
offset2  = 145

# takes two input angles and check for angle and position validity
def safetycheck(angle1, angle2):
        angle1rad = math.radians(angle1)
        angle2rad = math.radians(angle2)
        anglealpha = angle1rad-angle2rad

        x1 = length1 * math.cos(angle1rad)
        y1 = length1 * math.sin(angle1rad)
        x2 = length2 * math.cos(anglealpha)
        y2 = length2 * math.sin(anglealpha)
        
        finalX = x1 + x2
        finalY = y1 + y2
        
        # print("x: " + str(x1) + " y: " + str(y1))
        print("final x: " + str(finalX) + " final y: " + str(finalY))
        
        # is the angle valid?
        if ((angle1 < 0) or (angle1 > 90)):
            print("error: invalid shoulder angle. Only ranges between 0 and 90 are accepted.")
            return(-1)
        elif((angle2 < -91) or (angle2 > 120)):
            print("error: invalid forearm angle. Only ranges between -40 and 120 are accepted")
            return(-1)

        # does the angle result in valid "Y" values    
        elif (y1 < -1):
            print(x1,y1)
            print("error: angle results in invalid elbow position")
            return(-1)
        elif((y1+y2) < -1):
            print("error: angle results in invalid forearm position")
            return(-1)

        # safetycheck(a1,a2) must equal 1 for any movement to occur    
        else:
            print("good to go !")
            return(1)


def  movearms(angle1, angle2):
        # reverse angle: forearm angle increases towards the table (overhand)
        angle2 = -angle2
        
        # slow enough for predictable outcomes or fall apart
        bicep.set_velocity(10,PERCENT)
        motor_2.set_velocity(10,PERCENT)

        #move shoulder/bicep into position
        # above desired angle, move down bicep
        if (potentiometerV2_h.angle(DEGREES) > (angle1+offset1)):
            while True:
                bicep.spin(FORWARD)
                if (potentiometerV2_h.angle(DEGREES) < (angle1+offset1)):
                    bicep.stop()
                    break

        # below desired angle, move up bicep
        elif(potentiometerV2_h.angle(DEGREES) < (angle1+offset1)):
            while True:
                bicep.spin(REVERSE)
                if (potentiometerV2_h.angle(DEGREES) > (angle1+offset1)):
                    bicep.stop()
                    bicep.stop()
                    break
            
        # above desired angle, move down forearm
        if (potentiometerV2_a.angle(DEGREES) > (angle2+offset2)):
            print("moving down")
            while True:
                motor_2.spin(FORWARD)
                if(potentiometerV2_a.angle(DEGREES) < (angle2+offset2)):
                    motor_2.stop()
                    break

        # below desired angle, move up forearm
        elif(potentiometerV2_a.angle(DEGREES) < (angle2+offset2)):
            print("moving up")
            while True:
                motor_2.spin(REVERSE)
                if(potentiometerV2_a.angle(DEGREES) > (angle2+offset2)):
                    motor_2.stop()
                    break
       
        print("move done")
        
# functions which sets claw to max and min opening states
def clawgrab():
    servo_c.set_position(50,DEGREES)

def clawrelease():
    servo_c.set_position(-50,DEGREES)

# check that angles are valid, then move arms, then check again
def run(a1,a2):
    if (safetycheck(a1,a2) == 1):
        movearms(a1,a2)
        safetycheck(a1,a2)

# find the proper angles to allow the arms to move to the endpoint
def inverseKinematics(x, y):    
    theta2 = math.acos(((math.pow(x,2)) + (math.pow(y,2)) - (math.pow(length1,2)) - (math.pow(length2, 2))) / (2 * length1 * length2))
    # print(math.degrees(theta2))
    
    # tan^-1 (y/x) = alpha, which is going to be added to the rest instead of subtracted due to the overhand design
    theta1 = math.atan(y / x) + math.asin((length2 * math.sin(theta2)) / math.sqrt((math.pow(x,2)) + (math.pow(y,2))))
    
    # print(math.degrees(theta1)) 
    return theta1, theta2

while True:
    # start with open claw
    clawrelease()

    # distance: from bottom of robot to can +5 inch buffer to allow bot to grab center of can
    goal_distance = distance_7.object_distance(INCHES) + 5

    # find proper angles
    # always pick up a can at y = 3.1
    t1, t2 = inverseKinematics(goal_distance,3.1)

    print("Theta1: " + str(math.degrees(t1)) + "Theta2: " + str(math.degrees(t2)))

    # convert kinematics angles into degrees and run bot
    run(math.degrees(t1), math.degrees(t2))

    # ideally ends at correct endpoint and grabs can
    clawgrab()

    # position which bot drops can over its axis
    run(80,-40)

    # drop can
    clawrelease()

print("object distance: " + str(distance_7.object_distance(INCHES)))







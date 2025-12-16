"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the e-puck motors and distance sensors.
cruiseVelocity = 5.0
num_dist_sensors = 8

# Get left and right wheel motors.
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

# Get frontal distance sensors.
dist_sensors = [robot.getDevice('ps' + str(x)) for x in range(num_dist_sensors)]  # distance sensors
list(map((lambda s: s.enable(timeStep)), dist_sensors))  # Enable all distance sensors

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))


# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(cruiseVelocity)
rightMotor.setVelocity(cruiseVelocity)

camera = robot.getDevice("camera")
camera.enable(timeStep)


near_front = 100
near_side = 90

while robot.step(timeStep) != -1:

    # if(robot.getTime()>10.0):
        # exit(0)

    dist_sensor_values = [g.getValue() for g in dist_sensors]
    camera_values = camera.getImageArray()
    
    print(dist_sensor_values)
    print(camera_values[16][16])  # center pixel RGB values
    
    if (camera_values[16][16][0] + camera_values[16][16][1]) > 430:
        color = 'yellow'
    elif camera_values[16][16][0] > 230:
        color = 'red'
    elif camera_values[16][16][2] > 230:
        color = 'blue'
    else:
        color = 'corridor'
        
    print(color)
    
    if dist_sensor_values[0] > near_front \
       or dist_sensor_values[7] > near_front:
        
        if (dist_sensor_values[1] + dist_sensor_values[2] + dist_sensor_values[0]) \
        > (dist_sensor_values[5] + dist_sensor_values[6] + dist_sensor_values[7]):
            print('rotate left')
            leftMotor.setVelocity (-cruiseVelocity)
            rightMotor.setVelocity( cruiseVelocity)
        else:
            print('rotate right')
            leftMotor.setVelocity ( cruiseVelocity)
            rightMotor.setVelocity(-cruiseVelocity)
    
    elif dist_sensor_values[1]>near_side and dist_sensor_values[1]>dist_sensor_values[6]:
        print('turn left')
        leftMotor.setVelocity ( 0.1*cruiseVelocity)
        rightMotor.setVelocity( 1.2*cruiseVelocity)
        
    elif dist_sensor_values[6]>near_side:
        print('turn right')
        leftMotor.setVelocity ( 1.2*cruiseVelocity)
        rightMotor.setVelocity( 0.1*cruiseVelocity)
    else:
        print('go')
        leftMotor.setVelocity(1.2 * cruiseVelocity)
        rightMotor.setVelocity(1.2 * cruiseVelocity)


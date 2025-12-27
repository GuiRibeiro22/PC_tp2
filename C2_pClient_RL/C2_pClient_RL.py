from controller import Robot
import random

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

receiver = robot.getDevice("receiver")
receiver.enable(timeStep)
receiver.nextPacket()

emitter = robot.getDevice("emitter")

actions = ["Forward","Left","Right","MediumLeft","MediumRight"]
Q = {}

alpha = 0.5
gamma = 0.9
epsilon = 0.1

near_wall = 100
near_front = 100
near_side = 90
danger_distance = 500

reset = False


def perform_action(action):
    if action == "Forward":
        leftMotor.setVelocity(1.2*cruiseVelocity)
        rightMotor.setVelocity(1.2*cruiseVelocity)
    elif action == "Right":
        leftMotor.setVelocity(1.2*cruiseVelocity)
        rightMotor.setVelocity(0.1*cruiseVelocity)
    elif action == "Left":
        leftMotor.setVelocity(0.1*cruiseVelocity)
        rightMotor.setVelocity(1.2*cruiseVelocity)
    elif action == "MediumLeft":
        leftMotor.setVelocity(-cruiseVelocity)
        rightMotor.setVelocity(cruiseVelocity)
    elif action == "MediumRight":
        leftMotor.setVelocity(cruiseVelocity)
        rightMotor.setVelocity(-cruiseVelocity)


def get_state():
    Result = []
    dist_sensor_values = [g.getValue() for g in dist_sensors]
    for dist_sensor_value in dist_sensor_values:
        if dist_sensor_value >= near_wall:
            Result.append(0)
        else:
            Result.append(1)

    return tuple(Result)
    

def avoid_walls():
    dist_sensor_values = [g.getValue() for g in dist_sensors]
    #print(dist_sensor_values)

    for dist_sensor_value in dist_sensor_values:
        if dist_sensor_value > danger_distance:
            print('back')
            leftMotor.setVelocity (-cruiseVelocity)
            rightMotor.setVelocity(-cruiseVelocity)
            return


    if dist_sensor_values[0] > near_front \
    or dist_sensor_values[7] > near_front:

        if (dist_sensor_values[5] + dist_sensor_values[6] +dist_sensor_values[7]) < (dist_sensor_values[2] + dist_sensor_values[1]+dist_sensor_values[0]):
            #print('Rotate Left')
            leftMotor.setVelocity (-cruiseVelocity)
            rightMotor.setVelocity(cruiseVelocity)
        else:
            #print('Rotate Right')
            leftMotor.setVelocity (cruiseVelocity)
            rightMotor.setVelocity(-cruiseVelocity)

    elif dist_sensor_values[1]>near_side and dist_sensor_values[1]>dist_sensor_values[6]:
        #print('turn left')        
        leftMotor.setVelocity ( 0.1*cruiseVelocity)
        rightMotor.setVelocity(cruiseVelocity)
    elif dist_sensor_values[6]>near_side:
        #print('turn right')
        leftMotor.setVelocity (cruiseVelocity)
        rightMotor.setVelocity( 0.1*cruiseVelocity)
    else:
        #print('go')
        leftMotor.setVelocity(1.2*cruiseVelocity)
        rightMotor.setVelocity(1.2*cruiseVelocity)

    return False

def choose_action(state):
    if random.random() < epsilon:
        return random.choice(actions)
    qs = [Q.get((state, a), 0.0) for a in actions]
    max_q = max(qs)
    return random.choice([a for a, q in zip(actions, qs) if q == max_q])


def update_Q(state, action, reward, next_state):
    old = Q.get((state, action), 0.0)
    next_max = max(Q.get((next_state, a), 0.0) for a in actions)
    Q[(state, action)] = old + alpha * (reward + gamma * next_max - old)


print("Started")

while robot.step(timeStep) != -1:

    current_state = get_state()
    current_action = choose_action(current_state)
    if not avoid_walls():
        perform_action(current_action)

    reset = False

    if receiver.getQueueLength() > 0 and not reset:
        reward = receiver.getFloats()
        print(reward)

        next_state = get_state()
        update_Q(current_state, current_action, reward, next_state)

        emitter.send(b"Reset")

        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

        current_state = next_state
        current_action = choose_action(current_state)
        if not avoid_walls():
            perform_action(current_action)

        reset = True

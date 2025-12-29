from controller import Robot,Receiver,Emitter
import random
import time

# create the Robot instance.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

cruiseVelocity = 5.0
num_dist_sensors = 8

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")


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

emitter = robot.getDevice("emitter")

ACTIONS = ["Forward", "Left", "Right", "MediumLeft", "MediumRight", "Back"]
Q = {}

ALPHA = 0.5
GAMMA = 0.9
EPSILON = 0.1

NEAR_WALL = 100
DANGER_DISTANCE = 300

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
    elif action == "Back":
        leftMotor.setVelocity(-cruiseVelocity)
        rightMotor.setVelocity(-cruiseVelocity)



def get_state():
    Result = []
    dist_sensor_values = [g.getValue() for g in dist_sensors]
    for dist_sensor_value in dist_sensor_values:
        if dist_sensor_value >= NEAR_WALL:
            Result.append(0)
        else:
            Result.append(1)

    return tuple(Result)
    

def Enable_Actions():
    dist_sensor_values = [g.getValue() for g in dist_sensors]
    EnabledActions = []

    if dist_sensor_values[0] > DANGER_DISTANCE or dist_sensor_values[7] > DANGER_DISTANCE:
        EnabledActions += ["Back"]
        return EnabledActions


    if dist_sensor_values[0] > NEAR_WALL \
    or dist_sensor_values[7] > NEAR_WALL:

        if (dist_sensor_values[5] + dist_sensor_values[6] +dist_sensor_values[7]) < (dist_sensor_values[2] + dist_sensor_values[1]+dist_sensor_values[0]):
            EnabledActions += ["MediumLeft"]

        if (dist_sensor_values[5] + dist_sensor_values[6] +dist_sensor_values[7]) > (dist_sensor_values[2] + dist_sensor_values[1]+dist_sensor_values[0]):
            EnabledActions += ["MediumRight"]

    if dist_sensor_values[1] > NEAR_WALL and dist_sensor_values[1] > dist_sensor_values[6]:
        EnabledActions += ["Left"]  

    if dist_sensor_values[6] > NEAR_WALL and dist_sensor_values[6] > dist_sensor_values[1]:
        EnabledActions += ["Right"]
    else:
        EnabledActions += ["Forward"]

    return EnabledActions

def choose_action(state,PossibleActions):
    if random.random() < EPSILON:
        return random.choice(PossibleActions)
    
    qs = [Q.get((state, a), 0.0) for a in PossibleActions]
    max_q = max(qs)
    return random.choice([a for a, q in zip(PossibleActions, qs) if q == max_q])


def update_Q(state, action, reward, next_state):
    old = Q.get((state, action), 0.0)
    next_possible = Enable_Actions()
    next_max = max(Q.get((next_state, a), 0.0) for a in next_possible)
    Q[(state, action)] = old + ALPHA * (reward + GAMMA * next_max - old)




print("Started")

current_state = get_state()
old_reward = 0
ACTION_STEPS = 5
NUM_EPISODES = 20

while robot.step(timeStep) != -1:

    for episode in range(NUM_EPISODES):

        
        print(f"Episode number {episode}\n")
        done = False

        while not done:

            current_state = get_state()
            #print("Current state: ",current_state)


            PossibleActions = Enable_Actions()
            #print("Possible actions: ", PossibleActions)

            current_action = choose_action(current_state,PossibleActions)
            perform_action(current_action)
            #print("Action performed: ", current_action)

            # step simulation for the action to take effect
            for _ in range(ACTION_STEPS):
                robot.step(timeStep)

            # Aqui recebemos um reward ------------------------
            #emitter.send("Action done")
            #print("Emitter sent: Action done")
            
            if receiver.getQueueLength() > 0:

                if receiver.getInts()[0] < 2000:
                    reward = receiver.getInts()[0]
                    final_reward = reward - old_reward
                    #print(final_reward)

                # ---------------------------------------------------

                    next_state = get_state()
                    update_Q(current_state,current_action,final_reward,next_state)

                    current_state = next_state
                    old_reward = reward


                    receiver.nextPacket()
                
                else:
                    message = receiver.getString()
                    print(message)

                    receiver.nextPacket()

                    done = True




    
    

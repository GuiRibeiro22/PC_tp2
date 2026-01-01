from controller import Robot
import random
import time

robot = Robot()

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


leftMotor.setVelocity(cruiseVelocity)
rightMotor.setVelocity(cruiseVelocity)

camera = robot.getDevice("camera")
camera.enable(timeStep)


ACTIONS = ["Forward", "Left", "Right", "MediumLeft", "MediumRight", "Back"]
ALPHA = 0.5
GAMMA = 0.9
EPSILON = 0.0


NEAR_WALL = 90
DANGER_DISTANCE = 250
ACTION_STEPS = 5
NUM_EPISODES = 20



import pickle

with open("q_table.pkl", "rb") as f:
    Q = pickle.load(f)

print("Q-table loaded.")




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



def get_state(lastaction,color_path):
    Result = []
    dist_sensor_values = [g.getValue() for g in dist_sensors]
    for dist_sensor_value in dist_sensor_values:
        if dist_sensor_value < NEAR_WALL:
            Result.append(0)
        elif dist_sensor_value < DANGER_DISTANCE/2:
            Result.append(1)
        elif dist_sensor_value < DANGER_DISTANCE:
            Result.append(2)
        else:
            Result.append(3)

    return (tuple(Result),lastaction,color_path)
    



def Enable_Actions():
    dist_sensor_values = [g.getValue() for g in dist_sensors]
    EnabledActions = []
    #print(dist_sensor_values)

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



def choose_action_greedy(state, PossibleActions):
    best_action = None
    best_q = -1e9

    for a in PossibleActions:
        q = Q.get((state, a), -1e9)
        if q > best_q:
            best_q = q
            best_action = a

    if best_action is None:
        return random.choice(PossibleActions)

    return best_action




def update_Q(state, action, reward, next_state):
    old = Q.get((state, action), 0.0)
    next_possible = Enable_Actions()
    next_max = max(Q.get((next_state, a), 0.0) for a in next_possible)
    Q[(state, action)] = old + ALPHA * (reward + GAMMA * next_max - old)




def next_color(seen_color):
    if seen_color == 'red':
        previous_color = 'yellow'
        expected_color = 'blue'

    elif seen_color == 'blue':
        previous_color = 'red'
        expected_color = 'yellow'

    elif seen_color == 'yellow':
        previous_color = 'blue'
        expected_color = 'red'

    return (expected_color,previous_color)


expected_color = 'blue'
previous_color = 'red'
color_path = previous_color
seen_color = 'green'
lastaction = 'Forward'


while robot.step(timeStep) != -1:

    current_state = get_state(lastaction, color_path)

    PossibleActions = Enable_Actions()
    action = choose_action_greedy(current_state, PossibleActions)

    perform_action(action)
    lastaction = action

    for _ in range(ACTION_STEPS):
        robot.step(timeStep)


    camera_values = camera.getImageArray()
    r, g, b = camera_values[16][16]

    if (r+g) > 500:
        seen_color = 'yellow'
    elif r > 250:
        seen_color = 'red'
    elif b > 250:
        seen_color = 'blue'
    else:
        seen_color = 'green'

    if seen_color == expected_color:
        color_path = seen_color
        (expected_color, previous_color) = next_color(seen_color)

    elif seen_color == previous_color:
        color_path = previous_color
        (expected_color, _) = next_color(previous_color)



    
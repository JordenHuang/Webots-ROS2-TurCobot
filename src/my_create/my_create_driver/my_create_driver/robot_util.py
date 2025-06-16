from pycreate2 import Create2
from . import direction


motor_left_velocity = 0
motor_right_velocity = 0

def wheel_turn(bot: Create2, wheel: str, direction: direction.Direction, cycles: float):
    '''
    wheel:      Which wheel to rotate
    direction:  Forward or backward
    cycles:     How many cycles to turn, 1 cycle equals 2*PI
    '''
    global motor_left_velocity, motor_right_velocity
    MAX_VELOCITY = 500

    v = cycles * MAX_VELOCITY
    if direction == direction.FORWARD:
        c = 1
    elif direction == direction.BACKWARD:
        c = -1
    elif direction == direction.STOP:
        c = 0

    value = int(c * v)
    if value > MAX_VELOCITY:
        value = MAX_VELOCITY
    print(value)

    if wheel == "left":
        bot.drive_direct(value, motor_right_velocity)
        motor_left_velocity = value
    elif wheel == "right":
        bot.drive_direct(motor_left_velocity, value)
        motor_right_velocity = value
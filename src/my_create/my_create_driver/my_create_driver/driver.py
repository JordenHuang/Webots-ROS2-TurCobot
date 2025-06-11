from pycreate2 import Create2
import time

PORT = "/dev/ttyUSB0"

def main():
    # Greeting to see if this function is called
    print("Hello from my_create driver's main function")

    # Create a Create2.
    my_create = Create2(PORT)
    # start the bot
    my_create.start()
    # safe mode
    my_create.safe()
    time.sleep(1)

    sensors = my_create.get_sensors()
    battery_capacity = sensors.battery_capacity
    battery_charge = sensors.battery_charge
    print(f"Battery capacity: {battery_capacity}")
    print(f"Battery charge: {battery_charge}")
    print(f"Battery percentage: {battery_charge / battery_capacity * 100.0:.2f}% (approximatly)")

    # turn in place
    # my_create.drive_direct(100,-100)  # inputs for motors are +/- 500 max
    # time.sleep(2)
    # my_create.drive_direct(-100,100)  # inputs for motors are +/- 500 max
    # time.sleep(2)


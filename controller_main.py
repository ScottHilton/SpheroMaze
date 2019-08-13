"""
Brigham Young University
Sphero Maze Runner
ECEn Department Demo
Camera Main Code

Written in Python 3
August 2019

The purpose of this code is to take position inputs from the maze
solver and give output commands to direct the Sphero as desired
"""

import time
from numpy import *
import cv2

import json

"""
Camera and Maze Parameters
"""
# Height and width of the warped and cropped
PERSPECTIVE_WIDTH = 560  # Pixel Width
PERSPECTIVE_HEIGHT = 240  # Pixel Height
# Maze dimensions
ROWS = 4  # The number of rows in the physical maze
COLS = 7  # The number of columns in the physical maze

# File where the PID gain values are stored.
PID_FILE = 'PID.txt'


class MazeController:
    """Maze Controller: controller for Sphero.

        Checkpoints (in coordinate form) are received from the solver code. The
        Sphero is directed from checkpoint to checkpoint until the maze is
        solved.

        This controller is operated through toggling flags externally. The
        navigate_maze function is designed to operated within an individual
        thread. Navigate_maze terminates once the maze is solved or is forced
        to end when a flag is set.
    """

    def __init__(self, maze_solver):
        """ The maze solver is loaded in for use.  Flags and variables needed
        for maze are initialized.  PID values for the controller are loaded.

        :param maze_solver: Maze solver object (solver.py)
        """
        self.maze_solver = maze_solver

        # PID gain values
        self.__load_pid()

        # Change in time; hardcoded to 1/10th second
        self.dt = 0.1

        # Threshold values for maze
        self.checkpointThreshold = 35
        self.headingOffset = 0

        # Flags
        self.controller_on = False  # Flag is true if the controller is running

    def control_start(self):
        """ Sets controller flag to true. This allows the navigate maze function
        to run.

        :return: nothing
        """
        self.controller_on = True

    def control_stop(self):
        """ Sets controller flag to false. Prevents the navigate maze function
        from running.

        :return: nothing
        """
        self.controller_on = False

    def control_status(self):
        """ Get the current controller status

        :return: Current maze controller status
        """
        return self.controller_on

    def navigate_maze(self, sphero):
        """ Navigates the Sphero through the maze.

        The controller follows these steps to solve the maze.
        1. Collect checkpoints from the Sphero's current position to the
            endpoint.
        2. Direct the Sphero to the nearest checkpoint
            a. Collect Sphero's current location and break into x and y
            coordinates.
            b. Calculate the Sphero's heading
            c. Calculate distance from the Sphero to the checkpoint
            d. Use PID values and error to determine Sphero speed.
            e. Roll the Sphero at the set speed and heading.
            f. Move Sphero until it reaches the checkpoint.
        3. Repeat steps 1 - 2 until there are no remaining checkpoints.
        4. Once finished, the Sphero flashes blue and resetting maze controller
            flag.

        This function is designed to be the target of a thread. It contains two
        loops:
        1. An outer loop which collects checkpoints from the solver and the
            current Sphero coordinates in the maze.
        2. An inner loop that uses the Sphero position and the closest
            checkpoint to physical control and direct the Sphero to said
            checkpoint.



        :param sphero: a connected and oriented sphero object
        :return: nothing
        """
        while self.controller_on:
            try:
                remaining_checkpoints = self.maze_solver.solveMaze()
            except Exception as ex:
                print(ex)
                print("Maze Unsolvable: Adjust Walls of Maze... Trying again")
                time.sleep(5)
                continue

            print("Remaining Checkpoints: " + str(remaining_checkpoints))

            if len(remaining_checkpoints) < 1:
                break

            # Collect X and Y coordinates for checkpoint
            checkpoint_x, checkpoint_y = \
                self.solver_to_image_coordinates(remaining_checkpoints[0])

            # Setup up for PID
            time.sleep(.2)  # Pause a bit
            previous_error = 0  # Initialize error
            integral = 0  # Initialize integrator

            start_time = time.time()

            while self.controller_on:
                # Get Sphero Coordinates
                sphero_coordinates = self.maze_solver.getSpheroCorodinates()

                # Check if there is even a Sphero in the maze
                if sphero_coordinates[0] == 0 and sphero_coordinates[1] == 0:
                    print('Passing: No sphero found')
                    time.sleep(0.5)
                    continue

                # Break Sphero coordinates into discrete X and Y coordinates
                x = sphero_coordinates[0]
                y = sphero_coordinates[1]

                # Calculate heading
                heading = math.atan2(checkpoint_y - y, checkpoint_x - x)
                heading = math.degrees(heading) + self.headingOffset

                # Convert heading into a value in the range of 0 to 360 degrees
                while heading < 0:
                    heading += 360
                while heading > 360:
                    heading -= - 360

                # Calculate the distance between Sphero and checkpoint
                distance = math.sqrt(math.pow(checkpoint_y - y, 2) +
                                     math.pow(checkpoint_x - x, 2))

                error = distance  # This distance is the error

                # PID Controller
                # Derivative Calculations
                derivative = (error - previous_error) / self.dt
                # Integral Calculation
                if derivative < 10:
                    integral += error * self.dt
                # Calculate output speed
                speed = self.KP_gain / 100 * error + self.KI_gain / 100 *\
                    integral - self.KD_gain / 100 * derivative

                previous_error = error  # Save error

                # Saturation limits for speed
                if speed > 255:
                    speed = 255
                if speed < 0:
                    speed = 0

                # Roll the Sphero in the set heading at the calculated speed
                if distance < self.checkpointThreshold:
                    sphero.roll(0, int(heading), 1, False)
                    print('Checkpoint!')
                    break
                else:
                    sphero.roll(int(speed), int(heading), 1, False)

                # If it takes longer than 5 seconds to get to checkpoint,
                #  signal timer overflow and start over
                if time.time() - start_time > 5:
                    print('TIMER OVERFLOW')
                    break

        # Finished Maze
        sphero.roll(0, 0, 0, False)
        sphero.set_rgb_led(0, 0, 255, 0, False)
        time.sleep(1)
        sphero.set_rgb_led(0, 0, 0, 0, False)
        cv2.waitKey(5)
        print("Navigate Maze Finished")
        self.controller_on = False

    def save_pid(self):
        """ This function will write the current PID values to a text file

        :return: nothing
        """
        print("Controller: Saving PID values to file")
        with open(PID_FILE, "w") as f:
            json.dump((self.KP_gain, self.KD_gain, self.KI_gain), f)

    def __load_pid(self):
        """ This function will read in PID values from a text file

        :return: nothing
        """
        try:
            with open(PID_FILE) as f:
                self.KP_gain, self.KD_gain, self.KI_gain = json.load(f)
        except Exception as error:
            print("Maze Camera: Unable to load PID values:", error)
            self.KP_gain = 10
            self.KD_gain = 10
            self.KI_gain = 10

    @staticmethod
    def solver_to_image_coordinates(solver_position_number):
        # refrence:
        # positions = set([0,1,2,3,4,5,6,10,11,12,13,14,15,16,20,21,22,23,24,
        #                  25,26,30,31,32,33,34,35,36])
        # 10's position represents the rows
        # 1's position represents the columns

        mazecheckpoint_x = (solver_position_number % 10) * 2 + 1
        mazecheckpoint_y = (solver_position_number // 10) * 2 + 1
        checkpoint_x = PERSPECTIVE_WIDTH / COLS / 2 + \
            mazecheckpoint_x // 2 * PERSPECTIVE_WIDTH / COLS
        checkpoint_y = PERSPECTIVE_HEIGHT / ROWS / 2 + \
            mazecheckpoint_y // 2 * PERSPECTIVE_HEIGHT / ROWS
        return checkpoint_x, checkpoint_y

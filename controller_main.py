#####################################################################
# Brigham Young University
# Sphero Maze Runner
# ECEn Department Demo
# Camera Main Code
#
#####################################################################

# Written in Python 3
# Version 0.0 (Prototype)
# About this version
# June 15, 2018
# 1.  Prototype for the controller block of the Sphero Maze Runner

#import solver # This is the maze solver that provides reference position information
#import sphero_driver
import time
from numpy import *
import cv2
import timeit
import json

PERSPECTIVE_WIDTH = 560		#Pixel Width
PERSPECTIVE_HEIGHT = 240	#Pixel Height
ROWS = 4				#The Number of rows in the physical maze
COLS = 7				#The number of columns in the physical maze

PID_FILE = 'PID.txt'

#####################################################################
# The purpose of this code is to take position inputs from the maze
# solver and give output commands to direct the Sphero as desired
#####################################################################

def solverToImageCoordinates(solver_Position_Number):
    # refrence:
        # positions = set([0,1,2,3,4,5,6,10,11,12,13,14,15,16,20,21,22,23,24,25,26,30,31,32,33,34,35,36])
        # 10's position represents the rows
        # 1's position represents the columns

    mazeCheckpointX = (solver_Position_Number % 10) * 2 + 1
    mazeCheckpointY = (solver_Position_Number // 10) * 2 + 1
    CheckpointX = PERSPECTIVE_WIDTH / COLS / 2 + mazeCheckpointX//2 * PERSPECTIVE_WIDTH / COLS
    CheckpointY = PERSPECTIVE_HEIGHT / ROWS / 2 + mazeCheckpointY//2 * PERSPECTIVE_HEIGHT / ROWS
    return CheckpointX, CheckpointY

class Maze_Controller:
    def __init__(self, maze_solver):
        self.maze_solver = maze_solver # Takes info from camera and turns it into maze solving instructions for the Controller

        # PID gain values
        self.__load_PID()

        self.dt = 0.1
        #self.dts = []
        # Threshold values for maze
        self.checkpointThreshold = 35
        self.headingOffset = 0

        # Flags
        self.controller_on = False  # Flag is true if the controller is running


    def __del__(self):
        pass

    def control_start(self):
        self.controller_on = True

    def control_stop(self):
        self.controller_on = False

    def control_status(self):
        return self.controller_on

    def navigate_maze(self, sphero): # Must pass in a connected and oriented sphero object

        while self.controller_on:
            print('starting while')
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

            ### Collect X and Y coordinates for checkpoint ###
            CheckpointX, CheckpointY = solverToImageCoordinates(remaining_checkpoints[0])
            print("Checkpoint Coordinates: " + str(CheckpointX) + " " + str(CheckpointY))

            coordinates = self.maze_solver.getSpheroCorodinates()
            print("Sphero Coordinates:" + str(self.maze_solver.coord_to_dik_num(coordinates)) + str(coordinates))

            # Setup up for PID
            time.sleep(.2)  # Pause a bit
            previous_error = 0  # Initialize error
            integral = 0  # Initialize integrator

            start_time = time.time()

            while self.controller_on:
                loop_time = time.time() # Record start time for calculating dt
                ### Get Sphero Coordinates ###
                self.sphero_coordinates = self.maze_solver.getSpheroCorodinates(reset = True)
                #print("Sphero Coordinates" + str(self.sphero_coordinates))

                # Check if there is even a Sphero in the maze
                if (self.sphero_coordinates[0] == 0 and self.sphero_coordinates[1] == 0):
                    print('Passing: No sphero found')
                    time.sleep(0.5)
                    continue

                # Break Sphero coordinates into discrete X and Y coordinates
                x = self.sphero_coordinates[0]
                y = self.sphero_coordinates[1]

                # Calculate heading
                heading = math.atan2(CheckpointY - y, CheckpointX - x)
                heading = math.degrees(heading) + self.headingOffset

                # Convert heading into a value in the range of 0 to 360 degrees if needed
                while heading < 0:
                    heading += 360
                while heading > 360:
                    heading -= - 360
                # Calculate the distance between Sphero and checkpoint
                distance = math.sqrt(math.pow(CheckpointY - y, 2) + math.pow(CheckpointX - x, 2))
                #print("DistanceY:" + str(CheckpointY - y) + "," +  str(distance))
                error = distance  # This distance is the error


                ## PID Controller ##
                # Derivative Calculations
                derivative = (error - previous_error) / 0.1 #self.dt
                # Integral Calculation
                if derivative < 10:
                    integral += error * 0.1 #self.dt
                # Calculate output speed
                speed = self.KP_gain/100 * error + self.KI_gain/100 * integral - self.KD_gain/100 * derivative
                                # Save error
                previous_error = error

                # Saturation limits for speed
                if speed > 255:
                    speed = 255
                if speed < 0:
                    speed = 0

                # Heading Correction
                k = cv2.waitKey(10)
                if k == 32:
                    print('Spacebar!')
                    break
                elif k == 2424832:
                    headingOffset = headingOffset + 45
                elif k == 2555904:
                    headingOffset = headingOffset + 45

                # Roll the Sphero in the set heading at the calculated speed
                if (distance < self.checkpointThreshold):
                    sphero.roll(0, int(heading), 1, False)
                    print('Checkpoint!')
                    #print('Loop Time: ',self.dts)
                    break
                else:
                    sphero.roll(int(speed), int(heading), 1, False)

                # If it takes longer than 5 seconds to get to checkpoint, signal timer overflow and start over
                if time.time() - start_time > 5:
                    timer_overlap = True
                    print('TIMER OVERFLOW')
                    #print('Loop Time: ', self.dts)
                    break
                else:
                    timer_overlap = False

                self.dt = time.time() - loop_time
                #self.dts.append(self.dt)

        # Finished Maze: stop Sphero and make the Sphero flash a different color.
        sphero.roll(0, 0, 0, False)
        sphero.set_rgb_led(0, 0, 255, 0, False)
        time.sleep(1)
        sphero.set_rgb_led(0, 0, 0, 0, False)
        cv2.waitKey(5)
        print("Navigate Maze Finished")
        self.controller_on = False



    # This function will write the current corner values to a text file
    def save_PID(self):
        print("Controller: Saving PID values to file")
        with open(PID_FILE, "w") as f:
            json.dump((self.KP_gain, self.KD_gain, self.KI_gain), f)


    # This function will read in corner values from a text file
    def __load_PID(self):
        try:
            with open(PID_FILE) as f:
                self.KP_gain, self.KD_gain, self.KI_gain = json.load(f)
            print("Controller: Loading PID values from file")
        except:
            print("Maze Camera: Unable to load PID from PID.txt")
            self.KP_gain = 10
            self.KD_gain = 10
            self.KI_gain = 10



def main():
    print(solverToImageCoordinates(24))

if __name__ == '__main__':
    main()

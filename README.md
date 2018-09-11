# Sphero Maze Runner Instructions

About
----
The Sphero Maze Runner is a real time self-solving maze. The maze operates by using images captured by the
webcam above the maze to direct the Sphero to the end point (red tile) of the maze.
Other details about the maze include:
  * The maze can be rearranged in real time. (i.e. walls can be moved while the Sphero is solving the maze)
  * Only one Sphero can operate at a time
  * The red finish point must be accessible to Sphero(aka maze must be solvable)

### Setting up the maze
#### Needed Hardware
The maze includes the following hardware:
  * X2 Sphero’s (and their charging stations)
  * X1 USB Webcam (attached to crossbar PVC)
  * X5 PVC pieces for the camera mount (x2 “T” base pieces, x2 support posts, x1 crossbar)
  * X1 maze board (wooden board with metal posts)
  * X1 end point (red paper tile)
  * Multiple white maze walls with green edges
  * X1 Computer (discussed in the Software section)
  * X1 USB extension cable
  * Power strip and extension cable

#### Hardware Setup
1. Begin setting up the maze by finding a flat, level area large enough to fit the maze board (e.g.table, floor, etc). The area should have enough space to accommodate the computer that you will be using. It is recommended that there is sufficient space for spectators to view and interact with the maze.

2. Set up the camera mount. Connect each support post to a base (it does not matter which one goes where here). Next, connect the crossbar to the supports. Each end of the crossbar should have a color that corresponds to the support that it should connect in to. Once the camera mount is assembled it is recommended to loosely wind the cord to the camera around the crossbar and support closest to the computer (this keeps the cable from dangling in the view of the camera).

3. Place the camera mount over the maze board. The mount’s base pieces and supports should be outside of the maze. Match the colors on the outside of the maze walls to those on the bottom of the support.

4. Use the USB extension cable to connect the webcam to the computer.

5. Other notes:
  a. Colors are currently indicated by electrical tape
  b. It is suggested that you set up the Sphero charging stations nearby, for 
     it is likely that one will operate while the other charges.

### Software

The maze uses a python program to operate the maze. The computer that runs this program needs
the following:
  * Bluetooth capability
  * Good processor. The program does a decent amount of image processing and the program may not run properly on an old or weak processor.
  * Windows or Linux OS (Windows 10 or Ubuntu 16 recommended, no guarantees with other versions).
  * Python (version 3.4 recommended.  OpenCV and PyBluez modules are required)
BE SURE TO TEST YOUR CONFIGURATION BEFOREHAND! The program is currently finicky and may behave differently for different systems. 

The following python files are needed to run the maze:
  1. maze_main.py
  2. GUI_main.py
  3. camera_main.py
  4. controller_main.py
  5. solver.py
  6. sphero_driver.py
  7. dijkstra.py
  8. priodict.py
  And the following are config files to save different settings
  9. camSettings.txt
  10. corners.txt
  11. parameters.txt
  12. PID.txt  
  
To start and set up the program do the following:

1. Set up hardware if that has not been done already.

2. Make sure each Sphero is paired with the computer via Bluetooth. (See Troubleshooting for
more details on connecting the Sphero’s to Bluetooth)

3. Make sure the webcam is connected to the computer

4. In Windows 10 double click on the file named "maze_main.py" to run the program.  For Linux open a terminal in the directory where "main_maze.py" is located and type the following:
   python3 main_maze.py

5. At the start of the program two windows should show up:
   1. A terminal with program operating information (this is where errors and program messages will appear).
   2. A GUI used to run the program.  

6. Click on the live feed button under the Tests Section of the GUI (3rd Row down). Using the live image of the maze, adjust the camera mount (or maze board) until the maze board is fairly centered and square with the window of the image. All the edges of the maze
should be visible.

7. Next click on the Corners button under the Settings Section.  Click on each of the four corners on the image in the window that pops up.  The corners can be selected in any order.  

8. After selecting the corners, click on the Filters button under the Settings Section.  A screenshot of the walls filter and a window of sliders should appear.  Adjust the filters until only the walls are visible.  Press spacebar to continue to endpoint filters.  Adjust the sliders until only the red endpoint appears.  Press spacebar to exit filters.  
   Note: Different lighting will require different filter values. If you are having difficulty getting the filters right, click on the 
         Camera button to adjust the brightness and exposure of the camera, and then try adjusting the filters again.

9. The maze settings should now be complete.  If the current settings are not working properly they can be reconfigured at any time (be sure to stop the maze before doing such.  Failing to do so can cause the program to freak out.)  
     Note: The maze settings are saved to a configuration file. This means 
     that if the program is restarted, then you can skip through steps 6 
     through 8.  Also, note that if the camera exposure and brightness have been 
     changed, then you will need to set the camera to those values again
     since the camera will open with its default factory values.  

10. The last main setup thing to do is connect a Sphero:
    1. Make sure the Sphero(s) is paired with the computer (see troubleshooting 
     for more details).
    2. Select a Sphero and gently tap it against a surface until it lights up. 
     The color pattern will tell you which Sphero it is (e.g. WYO = White 
     Yellow Orange, BPW = Blue Purple White).
    3. Under the Sphero Settings on the menu, there is a drop down menu with a 
     list of Sphero devices. Select the corresponding Sphero that you want to 
     run. Then press “Connect.”
    4. The program will attempt to connect with the Sphero. Be sure that the 
     Sphero is still lit up. The Sphero lights will go out and a blue dot
     will light up on the side.  This is the orientation dot.
    5. Orient the Sphero. When the Sphero is first connected a blue dot will 
     light up on its side. On one side of the maze board perimeter wall there 
     will be a yellow mark. Orient the Sphero so that the blue dot faces that 
     wall (not the yellow mark itself, but the wall that the yellow mark is           
     on). If you need to reorient the Sphero at anytime, press the 
     “Orientation” button under Sphero Settings and orient the Sphero. Note: 
     the “Orientation” button locks the gyro of the Sphero, and you cannot 
     properly orient the Sphero unless the gyro is locked.

11. The program should be good to go! You can always adjust the settings later if they need to be adjusted.

### Running the maze

Once the maze and the program have been set up operating the maze essentially consists of the following:\\ 
1. Arrange/rearrange maze walls. Have spectators participate in this. Make sure there is a path from the Sphero to the endpoint. If the endpoint is missing the Sphero will by default go to the upper left-hand corner of the maze and stop. Also, if the Sphero is walled in the program will go on strike.

2. Press “Start.” Sometimes the program will immediately say that the Sphero has completed the maze even if it has not. Just press “Start” again if this is the case.

3. The Sphero should begin to solve the maze. Some notes about the maze:
   a. It is a real-time maze solver. This means you can rearrange the walls as 
     the Sphero solves the maze. You can also move the endpoint. Feel free to 
     have the spectators rearrange the maze as the Sphero goes along, but you 
     may want to inform them about the two conditions from part 1 (or not).
   b. Sometimes the Sphero may get stuck. Feel free to bump the Sphero to set 
     it on the right course again. Do not go crazy though, as it may mess up 
     the Sphero’s orientation.
   c. If you need to stop the maze, just push the “STOP” button.
   
4. Once the Sphero reaches the endpoint it will flash a blue color.

5. Repeat steps 1 through 4 to run the maze again.

Other notes:
  * If you wish to swap out the Sphero, stop the maze, push “Disconnect” under the Sphero Settings, and then reconnect to the desired Sphero

### Troubleshooting
The following are troubleshooting ideas. This is list not comprehensive.\\ 
  * If the program quits working, the easiest thing to do is quit the program and restart it.
To check if the Sphero is paired with the computer do the following (Windows only):
    1. In the Windows settings, go to “Bluetooth and other device settings”
    2. Under “Other Devices” the Sphero’s that are connected should be visible
    3. To add a Sphero, gently tap the Sphero to turn it on (it should light 
      up). Click “Add Bluetooth or other device,” then select Bluetooth. Wait 
      a moment for the Sphero to show up and then select it. The Sphero should 
      then be paired with the computer
For Linux, go to Bluetooth settings and follow the directions for pairing a Bluetooth device.   

If the Sphero is standing still it is likely due to the following:
    1. No path to either the end endpoint or upper-left corner
    2. Rearrange the maze, or adjust the filters
    3. Low battery (the Sphero will flash red when it is low on power)
Try the following fixes
    1. Move the Sphero to different part of board
    2. Rearrange the maze
    3. Disconnect and reconnect the Sphero
    4. Swap out Sphero with a different one

### Note About Previous Version
The previous version of the Sphero (used from August 2017 to about July 2018) used a different coloring format then the current version (as of August 2018).  Here are some of the basic changes from the old version to the new version:
   1. The new version uses green walls instead of white walls
   2. The new version uses a red endpoint instead of a blue endpoint
   3. The old version had the Sphero lit up (green back light color) as it solved the maze.  The new version has the Sphero unlit.
   
The old version has a similar GUI to the new version and operates in a similar manner.  Detailed instruction will not be provided on how to use the older version, but operation of the older version can be deduced from these instructions.  

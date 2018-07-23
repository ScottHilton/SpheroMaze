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
# July 3, 2018

import time
import tkinter as tk
from tkinter import OptionMenu, Menubutton
import threading

CAM_MAX_EXPOSURE = 20000  # Maximum exposure value
CAM_MAX_BRIGHTNESS = 100.0  # Maximum brightness value


class Main_Window():
    def __init__(self, master, application):
        self.app = application  # Main application class
        self.master = master  # tkinter root
        # Variables and flags
        self.sphero_device = tk.StringVar()
        self.sphero_device_connected = False  # True if a sphero device connected
        self.sphero_connecting = False  #  Used to prevent two threads from trying to connect to sphero
        self.widgets_setup = False  #  True if the widgets have been setup
        #Initialize window, frames, pack main frames
        self.create_main_window()
        self.frames_init()
        self.frames_pack()

        # Create the widgets
        self.create_widgets()

        # Status Flags - Used for updating status indicators on GUI
        self.update_sphero_connection = False
        self.update_maze_solving = False

        #maze.maze_initialize(self.cap)


    # Create the settings for main window
    def create_main_window(self):
        self.master.title("Sphero Maze Runner")
        self.master.minsize(width=500, height=200)
        self.master.maxsize(1000, 1000)

    # Initialize the main frames in the main window
    def frames_init(self):
        self.frame_inner_title = tk.Frame(self.master)
        self.frame_sphero = tk.Frame(self.master, relief="sunken",borderwidth=5)
        self.frame_settings = tk.Frame(self.master, relief="sunken",borderwidth=5)
        self.frame_camera_settings = tk.Frame(self.master, relief="sunken",borderwidth=5)
        self.frame_camera_calibrate = tk.Frame(self.master, relief="sunken",borderwidth=5)
        self.frame_maze_settings = tk.Frame(self.master, relief="sunken",borderwidth=5)
        self.frame_quit = tk.Frame(self.master, relief="sunken",borderwidth=5)

    # Pack the main frames in the main window
    def frames_pack(self):
        self.frame_inner_title.pack(fill="x",expand=True)
        self.frame_sphero.pack(fill="x",expand=True,padx=25)
        self.frame_settings.pack(fill="x",expand=True, padx=25)
        self.frame_maze_settings.pack(fill="x",expand=True, padx=25)
        self.frame_quit.pack(fill="x",expand=True, padx=25)

    def create_widgets(self):
        self.pack_inner_title()
        self.pack_sphero()
        self.pack_settings()
        self.pack_maze()
        self.pack_quit()

        self.widgets_setup = True

    # Contains the main title widget
    def pack_inner_title(self):
        self.inner_title = tk.Label(self.frame_inner_title, font=('Rockwell', 26),borderwidth=2)
        self.inner_title["text"] = "Sphero Maze Runner"
        self.inner_title.pack(side="top")

    # Contains the sphero widgets (Connect, disconnect, select sphero)
    def pack_sphero(self):
        # Create and pack Secondary frames
        sec_frame_sphero_title = tk.Frame(self.frame_sphero)
        sec_frame_sphero_status = tk.Frame(self.frame_sphero)
        sec_frame_sphero_interface = tk.Frame(self.frame_sphero)
        sec_frame_sphero_title.pack()
        sec_frame_sphero_status.pack()
        sec_frame_sphero_interface.pack(expand=True)

        #Create and pack Tertiary frames
        ter_frame_sphero_connect = tk.Frame(sec_frame_sphero_interface)
        ter_frame_sphero_select = tk.Frame(sec_frame_sphero_interface, width=500)
        ter_frame_sphero_disconnect = tk.Frame(sec_frame_sphero_interface)
        ter_frame_sphero_connect.pack(side="left",padx=5,fill="x",expand=True)
        ter_frame_sphero_select.pack(side="left",padx=50,fill="x",expand=True)
        ter_frame_sphero_disconnect.pack(side="left",padx=5,fill="x",expand=True)

        # Create and pack title
        sphero_title = tk.Label(sec_frame_sphero_title, text="Sphero Settings", font=('Arial', 16))
        sphero_title.pack()

        # Create and pack status
        self.sphero_status = tk.StringVar() #  This will hold the sphero status
        self.sphero_status_display = tk.Label(sec_frame_sphero_status, font=('Arial', 12))
        self.sphero_status_display.pack()
        self.sphero_status_display["textvariable"] = self.sphero_status
        self.sphero_status.set("Disconnected")
        self.sphero_status_display.configure(fg="red")

        # Create and pack buttons and selection
        sphero_connect_button = tk.Button(ter_frame_sphero_connect, text="Connect", font=('system', 14)
                                           , fg="green",command=lambda: self.app.sphero_connect(self.sphero_device.get()),
                                          width=9,height=2)
        sphero_disconnect_button = tk.Button(ter_frame_sphero_disconnect, text="Disconnect", font=('system', 14)
                                          , fg="red", command=self.app.sphero_disconnect, width=9, height=2)
        sphero_menu_options = ['Sphero-BPW','Sphero-WYO']
        sphero_select_menu = OptionMenu(ter_frame_sphero_select, self.sphero_device
                                        , *sphero_menu_options)
        sphero_orient_button = tk.Button(ter_frame_sphero_select, text="Orientation", font=('system', 8),
                                         fg="purple", command=self.app.sphero_orient, width=13, height=1)

        sphero_connect_button.pack()
        sphero_select_menu.pack()
        sphero_disconnect_button.pack()
        sphero_orient_button.pack()

    # Contains the setting widgets for the filter, camera, and PID controller
    def pack_settings(self):
        # Create and pack the title label
        title_label = tk.Label(self.frame_settings,
                                          text="Settings", font=("Arial", 14), fg='blue')
        title_label.pack(pady=1)
        # Create and pack secondary frames
        sec_frame_filter = tk.Frame(self.frame_settings)
        sec_frame_camera = tk.Frame(self.frame_settings)
        sec_frame_pid = tk.Frame(self.frame_settings)
        sec_frame_livefeed = tk.Frame(self.frame_settings)
        sec_frame_filter.pack(side="left", padx=5, fill="x", expand=True)
        sec_frame_camera.pack(side="left", padx=5, fill="x", expand=True)
        sec_frame_pid.pack(side="left", padx=5, fill="x", expand=True)
        sec_frame_livefeed.pack(side="left", padx=5, fill="x", expand=True)

        # Filter Button
        filter_button = tk.Button(sec_frame_camera, text="Filters",
                                            command=self.app.calibrate_filters,  font=('system', 14)
                                           , fg="cyan",
                                            width=12, height=2)
        filter_button.pack()

        # Camera Button
        camera_button = tk.Button(sec_frame_filter, text="Camera",
                                            command=self.app.calibrate_camera,  font=('system', 14)
                                           , fg="orange",
                                            width=12, height=2)
        camera_button.pack()

        # PID button
        pid_button = tk.Button(sec_frame_pid, text="PID",
                                            command=self.app.controller_set_PID,  font=('system', 14)
                                           , fg="magenta",
                                            width=12, height=2)
        pid_button.pack()

        # Live Feed Button
        live_feed_button = tk.Button(sec_frame_livefeed, text="Live Feed",
                                            command=self.app.toggle_live_feed,  font=('system', 14)
                                           , fg="green",
                                            width=12, height=2)

        live_feed_button.pack()

    def pack_maze(self):
        # Create and pack secondary frames
        sec_frame_maze_info = tk.Frame(self.frame_maze_settings)
        sec_frame_maze_buttons = tk.Frame(self.frame_maze_settings)

        sec_frame_maze_info.pack()
        sec_frame_maze_buttons.pack()

        # Create and pack the title
        maze_settings_title = tk.Label(sec_frame_maze_info, text="Maze Status", font=('Arial', 16), fg="blue")
        maze_settings_title.pack(pady=1)

        # Create and pack the maze status
        # self.maze_settings_status = tk.StringVar()  # This will hold the sphero status
        self.maze_settings_display = tk.Label(sec_frame_maze_info, font=('Arial', 10), text="Maze Status: STOPPED",fg="red")
        self.maze_settings_display.pack()
        # self.maze_settings_display["textvariable"] = sec_frame_maze_info
        # self.maze_settings_status.set("Maze Status: Stopped")
        # self.maze_settings_display.configure(fg="red", text="Ma SOLVING: STOPPED")

        # Create and pack the start maze button
        maze_start_button = tk.Button(sec_frame_maze_buttons, command=self.app.maze_start,
                                      text="START", font=("Rockwell",14), fg="green",
                                      width=12, height=2)
        maze_stop_button = tk.Button(sec_frame_maze_buttons, command=self.app.maze_stop,
                                      text="STOP", font=("Rockwell", 14), fg="red",
                                      width=12, height=2)
        maze_start_button.pack(pady=1, padx = 10, side="left")
        maze_stop_button.pack(pady=1, padx = 10, side="left")

    def pack_quit(self):
        # Quit Button
        #print("Pack Quit")
        self.quitButton = tk.Button(self.frame_quit, text="QUIT", font=('Arial', 16), fg="red",
                                    command=self.close_window, borderwidth=5)
        self.quitButton.pack(side="top", pady=1, padx=10, expand=True)

    def close_window(self):
        self.master.destroy()
        self.app.quit_program()

    def update_status_indicators(self):
        if self.update_sphero_connection:
            if not self.sphero_device_connected:
                self.sphero_status.set("Connected: %s" % self.sphero_device.get())
                self.sphero_status_display.configure(fg="green")
                self.sphero_device_connected = True
            else:
                self.sphero_status.set("Disconnected")
                self.sphero_status_display.configure(fg="red")
                self.sphero_device_connected = False

            self.update_sphero_connection = False

    def sphero_connection_changed(self):
        self.update_sphero_connection = True

# This class creates a window used to calibrate the camera settings.
class Calibrate_Window():
    def __init__(self, master, cam):
        self.master = master
        self.maze_camera = cam

        # Initialize sixe of window
        self.master.title("Camera Settings")
        self.master.minsize(width=500, height=250)
        self.master.maxsize(800, 800)

        # Camera Settings, these values
        self.setting_exposure = self.maze_camera.cam_exposure_value
        self.setting_brightness = self.maze_camera.cam_brightness_value
        print('Brightness' , self.setting_brightness, self.maze_camera.cam_brightness_value)
        print('Exposure', self.setting_exposure, self.maze_camera.cam_exposure_value)

        # Flag used for live stream
        self.adjusting = True

        # Initialize and pack the frames
        self.frames_init()
        self.frames_pack()

        # Pack each of the setting interfaces
        self.exposure_pack()
        self.brightness_pack()
        # Pack the quit button
        self.quit_button_pack()

    # Frame for each part
    def frames_init(self):
        #print("Init Frames")
        self.frame_exposure = tk.Frame(self.master)
        self.frame_brightness = tk.Frame(self.master)
        self.frame_quit = tk.Frame(self.master)

    # Pack each frame
    def frames_pack(self):
        #print("Pack Frames")
        self.frame_exposure.pack(side="top",fill="x", expand=True)
        self.frame_brightness.pack(side="top",fill="x", expand=True)
        self.frame_quit.pack(side="bottom",fill="x", expand=True)

    # Exposure frames, buttons, and variables
    def exposure_pack(self):
        # Create and pack sub frames
        self.frame_exposure_add = tk.Frame(self.frame_exposure,borderwidth=3)
        self.frame_exposure_info = tk.Frame(self.frame_exposure,relief="sunken",borderwidth=3)
        self.frame_exposure_subtract = tk.Frame(self.frame_exposure,borderwidth=3)

        self.frame_exposure_subtract.pack(side="left", fill="x",expand=True,padx=3)
        self.frame_exposure_info.pack(side="left", fill="both", expand=True)
        self.frame_exposure_add.pack(side="left",fill="x",expand=True,padx=3)

        # Exposure frame variable (frame_exposure_value) stores the exposure as collected from the camera object
        self.exposure_value = tk.StringVar()
        self.exposure_value_display = tk.Label(self.frame_exposure_info, font=('Arial', 10),pady=5)
        self.exposure_value_display["textvariable"] = self.exposure_value

        self.exposure_add_button = tk.Button(self.frame_exposure_add, text="+", font=('Arial', 16), fg="green",
                                             command= lambda: self.maze_camera.set_exposure(self.__set_exposure(True)))
        self.exposure_label = tk.Label(self.frame_exposure_info, text="Exposure", font=('Arial', 16), fg="blue", width=20)
        self.exposure_sub_button = tk.Button(self.frame_exposure_subtract, text="-", font=('Arial', 16), fg="red",
                                             command= lambda: self.maze_camera.set_exposure(self.__set_exposure(False)))

        self.exposure_sub_button.pack(side="top",fill="both",expand=True)
        self.exposure_label.pack(side="top")
        self.exposure_value_display.pack(side="top")
        self.exposure_add_button.pack(side="top",fill="both",expand=True)

        #Initialize current exposure value
        self.exposure_value.set(self.maze_camera.cam_exposure_value)

    # Brightness frames, variables, and buttons
    def brightness_pack(self):
        # Create and pack sub frames
        self.frame_brightness_add = tk.Frame(self.frame_brightness, borderwidth=3)
        self.frame_brightness_info = tk.Frame(self.frame_brightness, relief="sunken", borderwidth=3)
        self.frame_brightness_subtract = tk.Frame(self.frame_brightness, borderwidth=3)

        self.frame_brightness_subtract.pack(side="left", fill="x", expand=True, padx=3)
        self.frame_brightness_info.pack(side="left", fill="both", expand=True)
        self.frame_brightness_add.pack(side="left", fill="x", expand=True, padx=3)

        # Exposure frame variable (frame_exposure_value) stores the exposure as collected from the camera object
        self.brightness_value = tk.StringVar()
        self.brightness_value_display = tk.Label(self.frame_brightness_info, font=('Arial', 10), pady=5)
        self.brightness_value_display["textvariable"] = self.brightness_value

        self.brightness_add_button = tk.Button(self.frame_brightness_add, text="+", font=('Arial', 16), fg="green", command= lambda :self.maze_camera.set_brightness(self.__set_brightness(True)))
        self.brightness_label = tk.Label(self.frame_brightness_info, text="Brightness",font=('Arial', 16), fg="blue", width=20)
        self.brightness_sub_button = tk.Button(self.frame_brightness_subtract, text="-", font=('Arial', 16), fg="red",command= lambda :self.maze_camera.set_brightness(self.__set_brightness(False)))

        self.brightness_sub_button.pack(side="top",fill="both",expand=True)
        self.brightness_label.pack(side="top")
        self.brightness_value_display.pack(side="top")
        self.brightness_add_button.pack(side="top",fill="both",expand=True)

        #Initialize Brightness Value
        self.brightness_value.set(self.setting_brightness)

    def quit_button_pack(self):
        # Quit Button
        #print("Pack Quit")
        self.quitButton = tk.Button(self.frame_quit, text="DONE", font=('Arial', 16), fg="red",
                                    command=self.close_windows,borderwidth=5)
        self.quitButton.pack(side="top",fill="x",expand=True)

    # This function interacts with the camera class to set the exposure of the camera
    # 'increment' is a boolean, where if increment == True, the exposure is increased,
    # otherwise, it is decremented
    def __set_exposure(self, increment):
        if increment:
            self.maze_camera.cam_exposure_value += 10
            # print("Increment exposure: ", exposure)
            self.exposure_value.set(self.maze_camera.cam_exposure_value)
        elif not increment:
            self.maze_camera.cam_exposure_value -= 10
            # print("Decrement exposure: ", exposure)
            self.exposure_value.set(self.maze_camera.cam_exposure_value)
        return self.maze_camera.cam_exposure_value
        # print("Set exposure: ", self.setting_exposure)
    # This function interacts with the camera class to set the brightness of the camera
    # 'increment' is a boolean, where if increment == True, the brightness is increased,
    # otherwise it is decremented
    def __set_brightness(self,increment):
        if increment:
            if self.maze_camera.cam_brightness_value >= 100:
                self.maze_camera.cam_brightness_value = 100
            else:
                self.maze_camera.cam_brightness_value += 5
                self.brightness_value.set(self.maze_camera.cam_brightness_value)
        elif not increment:
            if self.maze_camera.cam_brightness_value <= 0:
                self.maze_camera.cam_brightness_value = 0
            else:
                self.maze_camera.cam_brightness_value -= 5
                self.brightness_value.set(self.maze_camera.cam_brightness_value)
        return self.maze_camera.cam_brightness_value

    def close_windows(self):
        self.maze_camera.save_cam_settings()
        print("Saving Camera Settings")
        self.adjusting = False
        time.sleep(0.1)
        self.master.destroy()

# This class creates a window used to change the PID controller values
class PID_Window():
    def __init__(self, master, controller):
        self.master = master
        self.controller = controller

        # Initialize sixe of window
        self.master.title("PID Settings")
        self.master.minsize(width=500, height=250)
        self.master.maxsize(800, 800)

        # Initialize and pack the frames
        self.frames_init()
        self.frames_pack()

        # Pack each of the setting interfaces
        # self.dt_pack()    Don't use or modify dt value
        self.kp_pack()
        self.ki_pack()
        self.kd_pack()
        self.dt_pack()
        # Pack the quit button
        self.quit_button_pack()

    # Frame for each part
    def frames_init(self):
        #print("Init Frames")
        self.frame_dt = tk.Frame(self.master)
        self.frame_kp = tk.Frame(self.master)
        self.frame_ki = tk.Frame(self.master)
        self.frame_kd = tk.Frame(self.master)
        self.frame_quit = tk.Frame(self.master)


    # Pack each frame
    def frames_pack(self):
        #print("Pack Frames")
        self.frame_dt.pack(side="top",fill="x", expand=True)
        self.frame_kp.pack(side="top",fill="x", expand=True)
        self.frame_ki.pack(side="top",fill="x", expand=True)
        self.frame_kd.pack(side="top", fill="x", expand=True)
        self.frame_quit.pack(side="bottom", fill="x", expand=True)

    # dt frames, buttons, and variables
    def dt_pack(self):
        #print("dt pack")
        # Create and pack sub frames
        frame_dt_add = tk.Frame(self.frame_dt,borderwidth=3)
        frame_dt_info = tk.Frame(self.frame_dt,relief="sunken",borderwidth=3)
        frame_dt_subtract = tk.Frame(self.frame_dt,borderwidth=3)

        frame_dt_subtract.pack(side="left", fill="x", expand=True, padx=3)
        frame_dt_info.pack(side="left", fill="both", expand=True)
        frame_dt_add.pack(side="left", fill="x", expand=True, padx=3)

        # dt frame variable (frame_dt_value) stores the dt from the maze
        self.dt_value = tk.StringVar()
        self.dt_value_display = tk.Label(frame_dt_info, font=('Arial', 10), pady=5)
        self.dt_value_display["textvariable"] = self.dt_value

        dt_add_button = tk.Button(frame_dt_add, text="+", font=('Arial', 16), fg="green",
                                             command= lambda: self.__set_dt(True))
        dt_label = tk.Label(frame_dt_info, text="dt", font=('Arial', 16), fg="blue", width=20)
        dt_sub_button = tk.Button(frame_dt_subtract, text="-", font=('Arial', 16), fg="red",
                                             command= lambda: self.__set_dt(False))

        dt_sub_button.pack(side="top",fill="both",expand=True)
        dt_label.pack(side="top")
        self.dt_value_display.pack(side="top")
        dt_add_button.pack(side="top",fill="both",expand=True)

        #Initialize current dt value
        self.dt_value.set(6)

    # Kp frames, buttons, and variables
    def kp_pack(self):
        #print("kp pack")
        # Create and pack sub frames
        frame_kp_add = tk.Frame(self.frame_kp, borderwidth=3)
        frame_kp_info = tk.Frame(self.frame_kp, relief="sunken", borderwidth=3)
        frame_kp_subtract = tk.Frame(self.frame_kp, borderwidth=3)

        frame_kp_subtract.pack(side="left", fill="x", expand=True, padx=3)
        frame_kp_info.pack(side="left", fill="both", expand=True)
        frame_kp_add.pack(side="left", fill="x", expand=True, padx=3)

        # Kp frame variable (frame_kp_value) stores the Kp from the maze
        self.kp_value = tk.StringVar()
        self.kp_value_display = tk.Label(frame_kp_info, font=('Arial', 10), pady=5)
        self.kp_value_display["textvariable"] = self.kp_value

        kp_add_button = tk.Button(frame_kp_add, text="+", font=('Arial', 16), fg="green",
                                  command=lambda: self.__set_kp(True))
        kp_label = tk.Label(frame_kp_info, text="Kp", font=('Arial', 16), fg="blue", width=20)
        kp_sub_button = tk.Button(frame_kp_subtract, text="-", font=('Arial', 16), fg="red",
                                  command=lambda: self.__set_kp(False))

        kp_sub_button.pack(side="top", fill="both", expand=True)
        kp_label.pack(side="top")
        self.kp_value_display.pack(side="top")
        kp_add_button.pack(side="top", fill="both", expand=True)

        #Initialize current Kp value
        self.kp_value.set(self.controller.KP_gain)

    # Ki frames, buttons, and variables
    def ki_pack(self):
        #print("ki pack")
        # Create and pack sub frames
        frame_ki_add = tk.Frame(self.frame_ki, borderwidth=3)
        frame_ki_info = tk.Frame(self.frame_ki, relief="sunken", borderwidth=3)
        frame_ki_subtract = tk.Frame(self.frame_ki, borderwidth=3)

        frame_ki_subtract.pack(side="left", fill="x", expand=True, padx=3)
        frame_ki_info.pack(side="left", fill="both", expand=True)
        frame_ki_add.pack(side="left", fill="x", expand=True, padx=3)

        # Ki frame variable (frame_ki_value) stores the ki from the maze
        self.ki_value = tk.StringVar()
        self.ki_value_display = tk.Label(frame_ki_info, font=('Arial', 10), pady=5)
        self.ki_value_display["textvariable"] = self.ki_value
        ki_add_button = tk.Button(frame_ki_add, text="+", font=('Arial', 16), fg="green",
                                  command=lambda: self.__set_ki(True))
        ki_label = tk.Label(frame_ki_info, text="Ki", font=('Arial', 16), fg="blue", width=20)
        ki_sub_button = tk.Button(frame_ki_subtract, text="-", font=('Arial', 16), fg="red",
                                  command=lambda: self.__set_ki(False))

        ki_sub_button.pack(side="top", fill="both", expand=True)
        ki_label.pack(side="top")
        self.ki_value_display.pack(side="top")
        ki_add_button.pack(side="top", fill="both", expand=True)

        # Initialize current ki value
        self.ki_value.set(self.controller.KI_gain)

    # Kd frames, buttons, and variables
    def kd_pack(self):
        #print("kd pack")
        # Create and pack sub frames
        frame_kd_add = tk.Frame(self.frame_kd, borderwidth=3)
        frame_kd_info = tk.Frame(self.frame_kd, relief="sunken", borderwidth=3)
        frame_kd_subtract = tk.Frame(self.frame_kd, borderwidth=3)

        frame_kd_subtract.pack(side="left", fill="x", expand=True, padx=3)
        frame_kd_info.pack(side="left", fill="both", expand=True)
        frame_kd_add.pack(side="left", fill="x", expand=True, padx=3)

        # Kd frame variable (frame_kd_value) stores the Kd from the maze
        self.kd_value = tk.StringVar()
        self.kd_value_display = tk.Label(frame_kd_info, font=('Arial', 10), pady=5)
        self.kd_value_display["textvariable"] = self.kd_value

        kd_add_button = tk.Button(frame_kd_add, text="+", font=('Arial', 16), fg="green",
                                  command=lambda: self.__set_kd(True))
        kd_label = tk.Label(frame_kd_info, text="Kd", font=('Arial', 16), fg="blue", width=20)
        kd_sub_button = tk.Button(frame_kd_subtract, text="-", font=('Arial', 16), fg="red",
                                  command=lambda: self.__set_kd(False))

        kd_sub_button.pack(side="top", fill="both", expand=True)
        kd_label.pack(side="top")
        self.kd_value_display.pack(side="top")
        kd_add_button.pack(side="top", fill="both", expand=True)

        # Initialize current Kd value
        self.kd_value.set(self.controller.KD_gain)


    def quit_button_pack(self):
        # Quit Button
        #print("Pack Quit")
        self.quitButton = tk.Button(self.frame_quit, text="DONE", font=('Arial', 16), fg="red",
                                    command=self.close_windows,borderwidth=5)
        self.quitButton.pack(side="top",fill="x",expand=True)

    # These functions set the value for the given PID setting
    # 'increment' is a boolean, where if increment == True, the setting is increased,
    # otherwise, it is decremented
    def __set_dt(self, increment):
        if increment:
            self.controller.dt += 0.1
        elif not increment:
            self.controller.dt -= 0.1
            # Do not drop below zero
            if self.controller.dt <= 0:
                self.controller.dt = 0

        self.dt_value.set(self.controller.dt)
        # print("dt:",maze.dt)

    def __set_kp(self, increment):
        if increment:
            self.controller.KP_gain += 0.1
        elif not increment:
            self.controller.KP_gain -= 0.1
            # Do not drop below zero
            if self.controller.KP_gain <= 0:
                self.controller.KP_gain = 0

        self.kp_value.set(self.controller.KP_gain)
        # print("Kp:",maze.Kp)

    def __set_ki(self, increment):
        if increment:
            self.controller.KI_gain += 0.1
        elif not increment:
            self.controller.KI_gain -= 0.1
            # Do not drop below zero
            if self.controller.KI_gain <= 0:
                self.controller.KI_gain = 0

        # print("Set exposure: ", self.setting_exposure)
        self.ki_value.set(self.controller.KI_gain)
        # print("Ki:",maze.Ki)

    def __set_kd(self, increment):
        if increment:
            self.controller.KD_gain += 0.1
        elif not increment:
            self.controller.KD_gain -= 0.1
            # Do not drop below zero
            if self.controller.KD_gain <= 0:
                self.controller.KD_gain = 0
                return

        # print("Set exposure: ", self.setting_exposure)
        self.kd_value.set(self.controller.KD_gain )
        # print("Kd:",maze.Kd)


    def close_windows(self):
        self.adjusting = False
        time.sleep(0.1)
        self.master.destroy()

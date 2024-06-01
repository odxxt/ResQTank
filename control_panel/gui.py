import os
from pathlib import Path
import serial.tools.list_ports
from tkinter import Scrollbar, Tk, Canvas, Entry, Text, Button, PhotoImage, Toplevel
import threading
import time
import sys
import subprocess
import serial
import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar
import traceback
import math

# to send the arduino a specific command or char
auto = 0
auto_running = False
prev= None
def toArd(command,does):
    global auto
    auto = 0
    try:
        arduino.write(command.encode('utf-8'))
        print(does)
    except:
        print(f"Couldnt send '{does}' to Arduino.....Check if its connected")


# the class to redirect the terminal output
class TextRedirector:
    def __init__(self, widget):
        self.widget = widget

    def write(self, string):
        self.widget.configure(state='normal')
        self.widget.insert('end', string)
        self.widget.see('end')
        self.widget.configure(state='disabled')

    def flush(self):
        pass

#########################################################
#                       GUI Options                     #
#########################################################

# path options for images
OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH = OUTPUT_PATH / "assets" 

def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)

window = Tk()

window.geometry("890x881")
window.configure(bg = "#1F1F1F")
window.title("ResQTank Control Panel")
#window.iconbitmap(relative_to_assets("resqtank-logo.ico").as_posix())

# Bind arrow keys and other relevant keys for keyboard control
window.bind('<Up>', lambda event: toArd("f", "Forward Clicked"))
window.bind('<Down>', lambda event: toArd("b", "Backwards Clicked"))
window.bind('<Left>', lambda event: toArd("q", "Left Clicked"))
window.bind('<Right>', lambda event: toArd("e", "Right Clicked"))

# Bindings for lowercase
window.bind('<KeyPress-w>', lambda event: toArd("f", "Forward Clicked"))
window.bind('<KeyPress-s>', lambda event: toArd("b", "Backward Clicked"))
window.bind('<KeyPress-a>', lambda event: toArd("l", "Left Clicked"))
window.bind('<KeyPress-d>', lambda event: toArd("r", "Right Clicked"))
window.bind('<KeyPress-q>', lambda event: toArd("q", "Spin Left Clicked"))
window.bind('<KeyPress-e>', lambda event: toArd("e", "Spin Right Clicked"))
window.bind('<KeyPress-z>', lambda event: toArd("z", "z Clicked"))
window.bind('<KeyPress-c>', lambda event: toArd("c", "c Clicked"))

# Bindings for uppercase
window.bind('<KeyPress-W>', lambda event: toArd("f", "Forward Clicked"))
window.bind('<KeyPress-S>', lambda event: toArd("b", "Backward Clicked"))
window.bind('<KeyPress-A>', lambda event: toArd("l", "Left Clicked"))
window.bind('<KeyPress-D>', lambda event: toArd("r", "Right Clicked"))
window.bind('<KeyPress-Q>', lambda event: toArd("q", "Spin Left Clicked"))
window.bind('<KeyPress-E>', lambda event: toArd("e", "Spin Right Clicked"))
window.bind('<KeyPress-Z>', lambda event: toArd("z", "Z Clicked"))
window.bind('<KeyPress-C>', lambda event: toArd("c", "C Clicked"))

# Bind spacebar for stopping
window.bind('<space>', lambda event: toArd("s", "Stop Clicked"))

canvas = Canvas(
    window,
    bg = "#1F1F1F",
    height = 881,
    width = 890,
    bd = 0,
    highlightthickness = 0,
    relief = "ridge"
)

canvas.place(x = 0, y = 0)
active = canvas.create_text(
    365,
    775,
    anchor="nw",
    text="hello!",
    fill="#FFFFFF",
    font=("Ariel", 16 * -1, "bold")
)

image_image_2 = PhotoImage(
    file=relative_to_assets("image_2.png"))
image_2 = canvas.create_image(
    720,
    293,
    image=image_image_2
)

image_image_3 = PhotoImage(
    file=relative_to_assets("image_3.png"))
image_3 = canvas.create_image(
    720,
    582,
    image=image_image_3
)

canvas.create_text(
    638,
    354,
    anchor="nw",
    text="Motor Drivers",
    fill="#FFFFFF",
    font=("Poppins Medium", 16 * -1)
)

arduino = canvas.create_text(
    638,
    330,
    anchor="nw",
    text="Arduino",
    fill="#FFFFFF",
    font=("Poppins Medium", 16 * -1)
)

camera = canvas.create_text(
    638,
    307,
    anchor="nw",
    text="Camera",
    fill="#FFFFFF",
    font=("Poppins Medium", 16 * -1)
)

canvas.create_text(
    638,
    282,
    anchor="nw",
    text="2D Lidar",
    fill="#FFFFFF",
    font=("Poppins Medium", 16 * -1)
)

canvas.create_text(
    638,
    256,
    anchor="nw",
    text="3D Lidar",
    fill="#FFFFFF",
    font=("Poppins Medium", 16 * -1)
)

canvas.create_text(
    700,
    180,
    anchor="nw",
    text="Tank Status",
    fill="#FFFFFF",
    font=("Poppins SemiBold", 20 * -1)
)

image_image_4 = PhotoImage(
    file=relative_to_assets("image_4.png"))
image_4 = canvas.create_image(
    650,
    190,
    image=image_image_4
)

green_led = PhotoImage(
    file=relative_to_assets("on.png"))
red_led = PhotoImage(
    file=relative_to_assets("off.png"))

lidar_3d_led = canvas.create_image(
    797,
    264,
    image=red_led
)

lidar_2d_led = canvas.create_image(
    797,
    290,
    image=red_led
)


camera_led = canvas.create_image(
    797,
    316,
    image=red_led
)


arduino_led = canvas.create_image(
    797,
    340,
    image=red_led
)


motors_led = canvas.create_image(
    797,
    365,
    image=green_led
)

button_image_1 = PhotoImage(
    file=relative_to_assets("button_1.png"))
forward = Button(
    image=button_image_1,
    borderwidth=0,
    activebackground="#820000",
    highlightthickness=0,
    command=lambda: toArd("f","Forward Clicked"),
    relief="flat",
    bd=0
    )

forward.place(
    x=241.7,
    y=222,
    width=135.4,
    height=97.8
)

button_image_2 = PhotoImage(
    file=relative_to_assets("button_2.png"))
stop = Button(
    image=button_image_2,
    borderwidth=0,
    activebackground="#820000",
    highlightthickness=0,
    command=lambda: toArd("s", "Stop Clicked"),
    relief="flat",
    bd=0
)
stop.place(
    x=241.7,
    y=326,
    width=135.4,
    height=97.9
)

button_image_3 = PhotoImage(
    file=relative_to_assets("button_3.png"))
backward = Button(
    image=button_image_3,
    borderwidth=0,
    activebackground="#820000",
    highlightthickness=0,
    command=lambda: toArd("b", "Backwards Clicked"),
    relief="flat",
    bd=0
)
backward.place(
    x=241.7,
    y=430,
    width=135.4,
    height=98
)

def set_auto():
    global auto
    auto = 1
    print(auto)
    if auto_running==False:
        autonomous_thread = threading.Thread(target=autonomous)
        autonomous_thread.daemon = True
        autonomous_thread.start()
    else:
        pass

button_image_4 = PhotoImage(
    file=relative_to_assets("guidance.png"))
auto_mode = Button(
    image=button_image_4,
    borderwidth=0,
    activebackground="#820000",
    highlightthickness=0,
    command=lambda: set_auto(),
    relief="flat",
    bd=0
)
auto_mode.place(
    x=319,
    y=578,
    width=236,
    height=138
)

button_image_5 = PhotoImage(
    file=relative_to_assets("button_5.png"))
manual_mode = Button(
    image=button_image_5,
    borderwidth=0,
    activebackground="#820000",
    highlightthickness=0,
    command=lambda: toArd("manual", " Manual Mode Activated!"),
    relief="flat",
    bd=0
)
manual_mode.place(
    x=63,
    y=578,
    width=236,
    height=138
)

button_image_6 = PhotoImage(
    file=relative_to_assets("button_6.png"))
right = Button(
    image=button_image_6,
    borderwidth=0,
    activebackground="#820000",
    highlightthickness=0,
    command=lambda: toArd("r", "Right Clicked"),
    relief="flat",
    bd=0
)
right.place(
    x=383.4,
    y=326,
    width=135.7,
    height=97.9
)
################################
button_image_11 = PhotoImage(
    file=relative_to_assets("button_12.png"))
spin_right = Button(
    image=button_image_11,
    borderwidth=0,
    activebackground="#820000",
    highlightthickness=0,
    command=lambda: toArd("e", "Spin Right Clicked"),
    relief="flat",
    bd=0
)
spin_right.place(
    x=383.4,
    y=280,
    width=135,
    height=40
)

button_image_12 = PhotoImage(
    file=relative_to_assets("button_11.png"))
spin_left = Button(
    image=button_image_12,
    borderwidth=0,
    activebackground="#820000",
    highlightthickness=0,
    command=lambda: toArd("q", "Spin Left Clicked"),
    relief="flat",
    bd=0
)
spin_left.place(
    x=100,
    y=280,
    width=135,
    height=40
)

button_image_7 = PhotoImage(
    file=relative_to_assets("button_7.png"))
left = Button(
    image=button_image_7,
    borderwidth=0,
    highlightthickness=0,
    activebackground="#820000",
    command=lambda: toArd("l", "Left Clicked"),
    relief="flat",
    bd=0
)
left.place(
    x=99.7,
    y=326,
    width=135.5,
    height=98
)

canvas.create_text(
    75,
    127,
    anchor="nw",
    text="Controls",
    fill="#FFFFFF",
    font=("Poppins SemiBold", 36 * -1)
)

image_image_10 = PhotoImage(
    file=relative_to_assets("image_10.png"))
image_10 = canvas.create_image(
    445,
    67,
    image=image_image_10
)

canvas.create_text(
    670,
    465,
    anchor="nw",
    text="Features",
    fill="#FFFFFF",
    font=("Poppins SemiBold", 23 * -1)
)
button_image_13 = PhotoImage(
    file=relative_to_assets("process.png"))
start_detection = Button(
    image=button_image_13,
    borderwidth=0,
    highlightthickness=0,
    activebackground="#820000",
    command=lambda: run_image_detection(),
    relief="flat",
    bd=0
)
start_detection.place(
    x=610,
    y=550,
    width=225,
    height=40
)

button_image_14 = PhotoImage(
    file=relative_to_assets("3D-mapping.png"))
start_mapping = Button(
    image=button_image_14,
    #text="START 3D Mapping",
    borderwidth=0,
    highlightthickness=0,
    activebackground="#820000",
    command=lambda: launch_rtabmap(),
    relief="sunken",
    bd=0
)
start_mapping.place(
    x=610,
    y=500,
    width=225,
    height=40
)

button_image_15 = PhotoImage(
    file=relative_to_assets("IR.png"))
start_ir = Button(
    image=button_image_15,
    #text="START IR",
    borderwidth=0,
    highlightthickness=0,
    activebackground="#820000",
    command=lambda: ir_video_test(),
    relief="sunken",
    bd=0
)
start_ir.place(
    x=610,
    y=600,
    width=225,
    height=40
)

# the about Button and Canvas
def create_popup():
    # Create the Toplevel window
    popup = Toplevel(window)
    popup.title("About ResQTank")
    popup.geometry("500x500")  # Adjust the size as needed
    popup.configure(bg="#1F1F1F")

    # Create a canvas within the popup window
    popup_canvas = Canvas(popup, 
    bg="#333333", 
    width=500, 
    height=430, 
    bd=0, 
    highlightthickness=0, 
    relief="ridge"
    )
    
    popup_canvas.pack(pady=10, padx=10)

    about_text = (
    "ResQTank\n\n"
    "Version 1.0\n\n"
    "Developed by:\n"
    "Abdulaziz AlOdat\n"
    "Adnan Kazi\n"
    "Alexandre Meulien\n"
    "Amer Ammar\n\n"
    "ResQTank is a Lidar-based emergency response system designed\n" 
    "to quickly and efficiently respond to indoor disasters such as\n"
    "earthquakes and fires. Using advanced Lidar technology and an\n"
    "Explorer Tank, it assists first responders in mapping the \n"
    "unknown environment and in locating trapped victims.\n\n"
    "Key Technologies:\n"
    "- SBC:         NVIDIA Jetson Orin Nano\n"
    "- 3D Lidar:    Intel RealSense Lidar L515\n"
    "- 2D Lidar:    RPLidar A1M8\n"
    " \n\n"

)
    popup_canvas.create_text(10, 10, anchor="nw", text=about_text, fill="#FFFFFF", font=("Helvetica", 14 * -1))

    # Button on the main window to open the popup
    close_image = PhotoImage(
        file=relative_to_assets("close.png"))
    
    
    # Button to close the popup 
    close_button = Button(popup, image=close_image, command=popup.destroy, 
                          borderwidth=0, highlightthickness=0, relief="flat", bd=0,width=117,height=43)
    close_button.image = close_image  # Keep a reference, prevent garbage collection
    close_button.pack(pady=1, padx=1)

# Button on the main window to open the popup
about_image = PhotoImage(
    file=relative_to_assets("about.png"))
about_button = Button(
    image=about_image,
    borderwidth=0,
    highlightthickness=0,
    activebackground="#820000",
    command=create_popup,
    relief="flat",
    bd=0
)
about_button.place(
    x=10, 
    y=10, 
    width=116, 
    height=43
)  

# Image dimensions and position
top_left_x = 55  
top_left_y = 738  

# Create a Frame-like behavior using Canvas for Text widget with a Scrollbar
text_frame = Canvas(window, width=600, height=100, bd=0, highlightthickness=0)
text_scroll = Scrollbar(text_frame, orient="vertical")
text_box = Text(text_frame, yscrollcommand=text_scroll.set, wrap="word", bg="#1F1F1F", fg="white", insertbackground="white", state='disabled')

text_scroll.pack(side="right", fill="y")
text_box.pack(side="left", expand=True, fill="both")
text_scroll.config(command=text_box.yview)

# Create a window on the main canvas to hold the Text widget
canvas.create_window(top_left_x, top_left_y, anchor="nw", window=text_frame, width=800, height=120)


#########################################################
#               Arduino Setup & Checking                #
#########################################################
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Arduino" in port.description or "ACM" in port.device:
            #print("Available ports ->", port.device)
            return port.device
    return None

arduino = None

def setup_arduino_connection():
    global arduino
    port = find_arduino_port()
    if port:
        try:
            arduino = serial.Serial(port, 9600, timeout=1)
            canvas.itemconfig(arduino_led, image=green_led)
            return arduino
        except serial.SerialException:
            canvas.itemconfig(arduino_led, image=red_led)
            arduino = None
            
    else:
        canvas.itemconfig(arduino_led, image=red_led)
        arduino = None
    return None

def check_arduino_connection():
    port = find_arduino_port()
    if port:
        try:
            # Try opening the port
            with serial.Serial(port, 9600, timeout=1) as test_serial:
                return True
        except serial.SerialException:
            pass
    return False


def monitor_arduino():
    global arduino
    while True:
        if arduino is None or not check_arduino_connection():
            print("Attempting to connect to Arduino...")
            setup_arduino_connection()
        time.sleep(5)  # Check every 5 seconds
    


#########################################################
#           3D Lidar - Camera Setup & Checking          #
#########################################################

def check_device_connection(device_path):
    command = f'ls {device_path}'
    try:
        # Execute the command and redirect stderr to subprocess.PIPE so it's captured
        subprocess.check_output(command, shell=True, stderr=subprocess.PIPE, text=True)
        return True
    except subprocess.CalledProcessError:
        return False


def update_device_status(device_path, led_item, type):
    connected = check_device_connection(device_path)
    if connected:
        canvas.itemconfig(led_item, image=green_led)
    else:
        canvas.itemconfig(led_item, image=red_led)
        print(f"Device {type} is not connected.")
    window.after(5000, lambda: update_device_status(device_path, led_item,type))


#########################################################
#                Terminal Output on GUI                 #
#########################################################

# Redirect stdout and stderr to the text widget
sys.stdout = TextRedirector(text_box)
sys.stderr = TextRedirector(text_box) 

#########################################################
#                      Run RTAB-Map                     #
#########################################################

def launch_rtabmap():
    try:
        # Start RTAB-Map without waiting for it to complete. This keeps the GUI responsive.
        subprocess.Popen(["rtabmap"], shell=True)
    except Exception as e:
        print("Failed to start RTAB-Map:", str(e))

#########################################################
#                      Start Detection                  #
#########################################################

def run_image_detection():
    try:
        print("                    Starting People Detction!....press q to stop"),
        subprocess.Popen(["python3 detection.py"], shell=True)
    except subprocess.CalledProcessError as e:
        print("Image detection script failed:", str(e))

def ir_video_test():
    print('No IR Camera Connected Right Now.....')

#########################################################
#                      Guidance Mode                    #
#########################################################

lidar_port = '/dev/ttyUSB0'
#lidar_port = 'COM4'
BAUDRATE: int = 115200
TIMEOUT: int = 1
safest_angle=None
right_dist=None
left_dist=None
"""
************************************************** Arduino Segment ********************************************************************************
"""

def auto_ard(command,does):
    global arduino
    try:
        arduino.write(command.encode('utf-8'))
        print(does)
        pass
    except:
        print(f"Couldnt send '{does}' to Arduino.....Check if its connected")
"""
************************************************** Eqution Segment ********************************************************************************
"""
def polar_to_cartesian(angle_deg, distance):
    angle_rad = math.radians(angle_deg)
    x = distance * math.cos(angle_rad)
    y = distance * math.sin(angle_rad)
    return x,y

def cart_to_polar(x, y):
    r = math.sqrt(x**2 + y**2)
    theta = math.atan2(y, x)
    theta_degrees = math.degrees(theta)
    return r, theta_degrees

def line2(a1,d1,a2,d2):
    x1,y1=polar_to_cartesian(a1,d1)
    x2,y2=polar_to_cartesian(a2,d2)
    m = (x2-x1)/(y2-y1)
    b = y1-m*x1
    return m,b

def intersection(m1,b1,m2,b2):
    if(m1-m2==0):
        exit()
    x = (b2-b1)/(m1-m2)   
    y = m1*x+b1
    return x, y

def polar_line(angle,dist):
    angle_rad = math.radians(angle)
    
    if math.isclose(math.sin(angle_rad), 0):
        return float('inf')  # Vertical line, slope is infinity
    else:
        return math.tan(angle_rad)
    
def distance_to_line(a1,d1,a2,d2):
    m1 = polar_line(a1,d1)
    x,y=polar_to_cartesian(a2,d2)
    distance= (abs((-m1*x)+y))/math.sqrt(m1**2+1)
    return distance
"""
************************************************** Safe angle Segment ********************************************************************************
"""

def low_hi_angles(angle, angle_list):
    high_angle = angle_list[-1]
    high_diff=angle-angle_list[-1]
    for a in angle_list:
        diff=angle-a
        if diff<0 and diff>high_diff:
            high_angle=a
            high_diff=diff
    if(high_angle!=angle_list[-1]):
        low_angle= angle_list[angle_list.index(high_angle)-1]
    else:
        low_angle=angle_list[-1]
    return high_angle,low_angle



def check_collision(robot_width, angles,distances, angle, max_distance):
    for a in angles:  
        if abs(angle-a)<=math.degrees(math.asin(robot_width/max_distance)):
            distance=distance_to_line(angle,max_distance,a,distances[angles.index(a)])
            if distance<robot_width/2:
                return True,distances[angles.index(a)],angle
    return False,None,None  # No Collision detected


def check_sector(a1,a2,dir,robot_width, angles, distances, max_distance):
    safest_distance =0
    safest_angle = None
    for angle in range(a1, a2, dir):
            try:
                con, d, a = check_collision(robot_width, angles, distances, angle, max_distance)
                if not con:
                    safest_distance= max_distance
                    l,h=low_hi_angles(angle,angles)
                    safest_angle=(l+h)/2
                    return safest_distance,safest_angle
                    
                    
                if con:
                    if d > safest_distance:
                       
                        safest_distance = d
                        safest_angle = angle
                        if d>max_distance:
                            safest_distance=max_distance

            except Exception as e:
                traceback.print_exc()
                continue
    return safest_distance, safest_angle


def find_safest_travel_distance(robot_width, angles, distances, max_distance):
    safest_distance = 0
    safest_angle = 180
    right_dist=None
    left_dist=None
    front_dist=None
    try:
        conr,right_dist,ar = check_collision(robot_width, angles, distances, 270 , max_distance)
        conl,left_dist,al = check_collision(robot_width, angles, distances, 90 , max_distance)
        front_dist=check_collision(robot_width, angles, distances, 180 , max_distance)
        
        safest_distance,safest_angle,= check_sector(270,90,-1,robot_width, angles, distances, max_distance)
        if safest_distance<1000:
            #check left segment
            safest_distance,safest_angle,= check_sector(210,241,1,robot_width, angles, distances, max_distance)
            
        if safest_distance<600:
            #check right segment
            safest_distance,safest_angle,= check_sector(150,121,-1,robot_width, angles, distances, max_distance)
        if safest_distance<600:
             #check left segment 2 
            safest_distance,safest_angle,= check_sector(241,271,1,robot_width, angles, distances, max_distance)
            #check right segment 2
        if safest_distance<600:
            safest_distance,safest_angle,= check_sector(120,91,-1,robot_width, angles, distances, max_distance)  

    except Exception as ex:
        traceback.print_exc()
        # Handle the error as needed, such as logging or returning default values
    print(safest_distance,safest_angle)
    return safest_distance, safest_angle, right_dist,left_dist

 
def find_angle(ax,angles,distances):
    global right_dist,left_dist
    robot_width = 400  # in mm
    max_distance = 2500 # maximum distance the robot can travel
    safest_distance, safest_angle,right_dist,left_dist = find_safest_travel_distance(robot_width,angles,distances, max_distance)
    ax.clear()
    ax.scatter(np.deg2rad(angles), distances, c='b',s=5, label="LiDAR Data")
    # Plot safest angle line in polar coordinates
    if safest_angle != None:
        safest_angle_rad = math.radians(safest_angle)
        ax.plot([safest_angle_rad, safest_angle_rad], [0, safest_distance], c='r', linestyle='--', label="Safest Travel Distance")
        ax.plot(safest_angle_rad, safest_distance, marker='o', markersize=8, color='g')  # Mark end point
        ax.annotate(f"Safest Angle: {safest_angle} degrees", (safest_angle_rad, safest_distance), textcoords="offset points", xytext=(10,10), ha='center', color='g')
    ax.set_theta_direction(-1)  # Set the direction of the theta axis (counterclockwise)
    ax.set_theta_zero_location("S")  # Set the zero angle at South (top of the plot)
    ax.set_ylim(0, 5000)
    #print(safest_distance,safest_angle)
    return safest_angle
"""
************************************************** Realign Segment ********************************************************************************
"""
def realign():
    global safest_angle,right_dist,left_dist,arduino
    global prev
    try:
        if safest_angle!=None:

            if right_dist!=None and right_dist<=600 and prev!= "rf" and prev !="lf":
                    auto_ard("l","left")
                    auto_ard("f","forward")
                    print( prev)
                    prev = "rf"
                    

            if left_dist!=None and left_dist<=600 and prev != "rf" and prev != "lf":

                    auto_ard("r","right")
                    print("going forward")
                    auto_ard("f","forward")
                    
                    prev = "lf"
                    print( prev)
            
            
    except Exception as e:
        traceback.print_exc()
    #print(safest_angle)
       
"""
************************************************** Main ********************************************************************************
"""
def autonomous():
    global safest_angle,right_dist,left_dist
    lidar = RPLidar(lidar_port)
    try:
        print("Starting LIDAR...")
        lidar.start_motor()
        plt.ion()  # Turn on interactive mode for live plotting
        fig = plt.figure()
        ax = fig.add_subplot(111,polar =True)
        i=0       
        for scan in lidar.iter_scans():
                scan = [tup for tup in scan if tup[0] >= 14]
                angles = [angle for (_, angle, _) in scan]
                distances = [distance for (_, _, distance) in scan]
                safest_angle=find_angle(ax,angles,distances)
                plt.pause(0.0001)
                realign()
                if not(plt.fignum_exists(fig.number)):
                    break
                    
                #print("right",right_dist,left_dist)
    except Exception as e:
        print("Stopping.")
        traceback.print_exc()

    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        quit()

#########################################################
#                Running Different Threads              #
#########################################################

# Start checking the connections in separate threads
arduino_thread = threading.Thread(target=monitor_arduino)
arduino_thread.daemon = True  # this makes sure the thread will close when the main program closes
arduino_thread.start()

lidar_3d_thread = threading.Thread(target=update_device_status, args=("/dev/video6", lidar_3d_led,"3D Lidar"))
lidar_3d_thread.daemon = True
lidar_3d_thread.start()

camera_thread = threading.Thread(target=update_device_status, args=("/dev/video0", camera_led,"Camera"))
camera_thread.daemon = True
camera_thread.start()

lidar_2d_thread = threading.Thread(target=update_device_status, args=("/dev/ttyUSB0", lidar_2d_led ,"2D Lidar"))
lidar_2d_thread.daemon = True
lidar_2d_thread.start()

window.resizable(False, False)
window.mainloop()




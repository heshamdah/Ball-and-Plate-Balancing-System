"""
Ball and Plate System - Vision Processing and GUI Control
========================================================

This module implements a computer vision-based ball tracking system with PID control
for a ball and plate balancing mechanism. It includes:

- Real-time ball detection using HSV color filtering
- PID control algorithm for position control
- Tkinter GUI for parameter adjustment and system monitoring
- Serial communication with Arduino for servo control
- Multiple camera view windows (main, mask, camera)

Author: UPM Hesham&Tanzim
"""

import cv2
import numpy as np
import time
import imutils
import tkinter as tk
import tkinter.messagebox
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
from math import *
import threading

# Load calibration data from file
# Format: max_alpha|max_theta (first line contains maximum servo angles)
lines = open("data.txt").read().splitlines()
max_alpha, max_theta = lines[1].split("|")
max_alpha = - float(max_alpha)  # Invert alpha for correct direction
max_theta = float(max_theta)

# Initialize serial communication with Arduino
# Change COM port and baud rate according to your setup
ser = serial.Serial('COM8', 9600)

# Dictionary to store calibration data (alpha -> theta mapping)
dataDict = {}

# Camera configuration
Height = 480
width = 640
cap = cv2.VideoCapture(0)  # Initialize camera 
cap.set(3, width)   # Set camera width
cap.set(4, Height)  # Set camera height

# Load calibration data into dictionary
for i in range(1, len(lines)):
    alpha, theta = lines[i].split("|")
    dataDict[float(alpha)] = float(theta)

# ============================================================================
# GUI WINDOW SETUP
# ============================================================================

# Main operator control window
OperatorWindow = tk.Tk()
OperatorWindow.title("PID control Ball and Plate System")
OperatorWindow["bg"] = "white"
OperatorWindow.resizable(0, 0)  # Fixed window size
OperatorWindow.geometry("430x600")

# Camera view window (initially hidden)
CameraWindow = tk.Toplevel(OperatorWindow)
CameraWindow.title("Camera")
CameraWindow.resizable(0, 0)
lmain = tk.Label(CameraWindow)
lmain.pack()
CameraWindow.withdraw()  # Hide window initially

# Mask view window for debugging (initially hidden)
showMaskWindow = False
maskWindow = tk.Toplevel(OperatorWindow)
maskWindow.title("Mask View")
maskWindow.resizable(0, 0)
maskLabel = tk.Label(maskWindow)
maskLabel.pack()
maskWindow.withdraw()

# Video window visibility flag
showVideoWindow = False

# ============================================================================
# GUI CONTROL FUNCTIONS
# ============================================================================

def showCameraFrameWindow():
    """Toggle camera window visibility"""
    global showVideoWindow
    if showVideoWindow == False:
        CameraWindow.deiconify()  # Show window
        showVideoWindow = True
        BShowVideo["text"] = "Hide Camera"
    else:
        CameraWindow.withdraw()   # Hide window
        showVideoWindow = False
        BShowVideo["text"] = "Open Camera"

def endProgam():
    """Clean shutdown of the application"""
    OperatorWindow.destroy()

def toggleMaskWindow():
    """Toggle mask window visibility for debugging"""
    global showMaskWindow
    if showMaskWindow == False:
        maskWindow.deiconify()  # Show the mask window
        showMaskWindow = True
        BShowMask["text"] = "Hide Mask"
    else:
        maskWindow.withdraw()   # Hide the mask window
        showMaskWindow = False
        BShowMask["text"] = "Show Mask"

def donothing():
    """Placeholder function for window close events"""
    pass

# ============================================================================
# PID CONTROL PARAMETERS AND VARIABLES
# ============================================================================

# Default PID coefficients
sliderCoefPDefault = 0.006
sliderCoefIDefault = 0.001
sliderCoefDDefault = 0.005
sliderCoefmgDefault = 0.4

# PID control variables
totalErrorX = 0      # Cumulative error for integral term (X-axis)
totalErrorY = 0      # Cumulative error for integral term (Y-axis)
Time_lenght = 1      # Time tracking variable
alpha, beta, prevAlpha, prevBeta = 0, 0, 0, 0

# Filtering and control variables
N = 20              # Filter window size
prevDerivX = 0      # Previous derivative value (X-axis)
prevDerivY = 0      # Previous derivative value (Y-axis)
prevIntegX = 0      # Previous integral value (X-axis)
prevIntegY = 0      # Previous integral value (Y-axis)
delivery_time = 0   # Time tracking for sampling
prevErrorX = 0      # Previous error (X-axis)
prevErrorY = 0      # Previous error (Y-axis)
x, y = 0, 0         # Current ball position

# Previous position tracking
prevX, prevY = 0, 0
prevRefX, prevRefY = 0, 0
start_time = 0
prevFilteredIx, prevFilteredIy = 0, 0

# ============================================================================
# PID CONTROL ALGORITHM
# ============================================================================

def smooth_deadzone(error, deadzone):
    """
    Apply deadzone filtering to reduce noise around zero
    
    Args:
        error: Input error value
        deadzone: Deadzone threshold
    
    Returns:
        Filtered error value
    """
    if abs(error) < deadzone:
        return 0
    elif error > 0:
        return error - deadzone
    else:
        return error + deadzone

def PIDcontrol(ballPosX, ballPosY, prevBallPosX, prevBallPosY, refX, refY, deadZone):
    """
    PID control algorithm for ball position control
    
    Args:
        ballPosX, ballPosY: Current ball position
        prevBallPosX, prevBallPosY: Previous ball position
        refX, refY: Reference/target position
        deadZone: Deadzone threshold for noise filtering
    
    Returns:
        filteredIx, filteredIy: Control outputs for X and Y axes
        errorX, errorY: Current error values
    """
    global totalErrorX, totalErrorY
    global Ts, delivery_time
    global prevDerivX, prevDerivY, prevIntegX, prevIntegY
    global prevErrorX, prevErrorY
    global prevFilteredIx, prevFilteredIy

    # Calculate sampling time
    Ts = time.time() - delivery_time
    delivery_time = time.time()
    
    # Calculate error with deadzone filtering
    errorX = smooth_deadzone(refX - ballPosX, deadZone)
    errorY = smooth_deadzone(refY - ballPosY, deadZone)
    
    # Get PID coefficients from GUI sliders
    Kp = slider_P.get()    # Proportional gain
    Ki = slider_I.get()    # Integral gain
    Kd = slider_D.get()    # Derivative gain
    Fc = slider_MG.get()   # Filter coefficient

    # Calculate derivative terms (rate of change of error)
    try:
        derivX = (errorX - prevErrorX) / Ts
    except ZeroDivisionError:
        derivX = 0

    try:
        derivY = (errorY - prevErrorY) / Ts
    except ZeroDivisionError:
        derivY = 0
    
    # Calculate integral terms (cumulative error)
    totalErrorX += errorX * Ts
    totalErrorY += errorY * Ts
    
    # PID control law: Output = Kp*error + Ki*integral + Kd*derivative
    Cx = Kp * errorX + Ki * totalErrorX + Kd * derivX
    Cy = Kp * errorY + Ki * totalErrorY + Kd * derivY
    
    # Apply low-pass filter to smooth control outputs
    filteredIx = Fc * Cx + (1 - Fc) * prevFilteredIx
    filteredIy = Fc * Cy + (1 - Fc) * prevFilteredIy
    
    # Update previous values for next iteration
    prevErrorX = errorX
    prevErrorY = errorY
    prevFilteredIx = filteredIx
    prevFilteredIy = filteredIy
    
    return round(filteredIx, 1), round(filteredIy, 1), errorX, errorY

# ============================================================================
# MAIN VISION PROCESSING AND CONTROL LOOP
# ============================================================================

def main():
    """
    Main processing loop for ball detection, tracking, and control
    This function runs continuously to process camera frames and control the system
    """
    start_timeFPS = time.time()
    global x, y, alpha, beta
    global prevX, prevY, prevAlpha, prevBeta, prevRefX, prevRefY
    global X_referance, Y_referance, totalErrorX, totalErrorY
    global width, Height
    global Time_lenght, start_time
    global showVideoWindow

    # Capture frame from camera
    _, O_frame = cap.read()
    
    # Crop frame to square aspect ratio for consistent processing
    O_frame = O_frame[0:int(Height),
          int((width - Height) / 2):int(width - ((width - Height) / 2))]
    
    # Convert to HSV color space for better color detection
    HSV_frame = cv2.cvtColor(O_frame, cv2.COLOR_BGR2HSV)

    # Color detection parameters for different ball colors
    # Uncomment the appropriate color range for your ball
    
    # Yellow ball (currently active)
    loweryellow = np.array([0, 20, 205])
    upperyellow = np.array([56, 255, 255])

    # Green ball
    #lowerblue = np.array([44, 90, 25])
    #appergreen = np.array([179, 255, 255])

    # Red ball
    #lowerblue = np.array([0, 89, 146])
    #appergreen = np.array([179, 255, 255])

    # Create color mask for ball detection
    mask = cv2.inRange(HSV_frame, loweryellow, upperyellow)
    
    # Apply morphological operations to clean up the mask
    mask = cv2.blur(mask, (6, 6))           # Blur to reduce noise
    mask = cv2.erode(mask, None, iterations=2)  # Remove small objects
    mask = cv2.dilate(mask, None, iterations=2) # Fill gaps

    # Find contours in the mask
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[0]
    center = None

    # Draw reference point (target position)
    cv2.circle(O_frame, (int(X_referance), int(Y_referance)), int(4), (255, 0, 0), 2)

    # Process ball detection if contours are found
    if len(cnts) > 0:
        # Find the largest contour (assumed to be the ball)
        c = max(cnts, key=cv2.contourArea)
        Time_lenght = time.time() - start_time
        
        # Calculate ball center and radius
        (x, y), radius = cv2.minEnclosingCircle(c)
        
        if radius > 0:
            # Display ball coordinates on frame
            cv2.putText(O_frame, str(int(x)) + ";" + str(int(y)).format(0, 0), 
                       (int(x) - 50, int(y) - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Draw circle around detected ball
            cv2.circle(O_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            
            # Apply PID control to calculate servo angles
            filteredIx, filteredIy, errorX, errorY = PIDcontrol(
                int(x), int(y), prevX, prevY, X_referance, Y_referance, 30)
            start_time = time.time()

            # Convert control outputs to servo angles
            # Invert and offset to center position (75 degrees)
            filteredIx = 75 - filteredIx
            filteredIy = 75 - filteredIy

            # Debug output
            print(f" {filteredIy}  {filteredIx} {  errorY}  {errorX}  {y}  {x} ")
            
            # Send servo angles to Arduino via serial
            center_data = f'{filteredIx},{filteredIy}\n'.encode()
            ser.write(center_data)
    else:
        # Reset integral terms if ball is not detected
        totalErrorX, totalErrorY = 0, 0

    # Update mask window if visible
    if showMaskWindow == True:
        maskImage = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        maskImage = Image.fromarray(maskImage)
        maskImgtk = ImageTk.PhotoImage(image=maskImage)
        maskLabel.imgtk = maskImgtk
        maskLabel.configure(image=maskImgtk)

    # Update camera window if visible
    if showVideoWindow == True:
        O_frame = cv2.cvtColor(O_frame, cv2.COLOR_BGR2RGB)
        O_frame = Image.fromarray(O_frame)
        imgtk = ImageTk.PhotoImage(image=O_frame)
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)
    
    # Schedule next frame processing (5ms delay)
    lmain.after(5, main)

    # Update previous position values
    prevX, prevY = int(x), int(y)
    prevRefX, prevRefY = X_referance, Y_referance
    prevAlpha = alpha
    prevBeta = beta

# ============================================================================
# GUI ELEMENTS CREATION
# ============================================================================

# PID Control Variables Frame
PIDCoef_variable = tk.LabelFrame(OperatorWindow, text="PID Variables")
PIDCoef_variable.place(x=0, y=20, width=430)

# Proportional gain slider
slider_P = tk.Scale(PIDCoef_variable, from_=0, to=0.1, orient="horizontal", 
                   label="P (Proportional)", length=350, tickinterval=0.01,
                   resolution=0.001)
slider_P.set(sliderCoefPDefault)
slider_P.pack()

# Integral gain slider
slider_I = tk.Scale(PIDCoef_variable, from_=0, to=0.1, orient="horizontal", 
                   label="I (Integral)", length=350, tickinterval=0.01,
                   resolution=0.001)
slider_I.set(sliderCoefIDefault)
slider_I.pack()

# Derivative gain slider
slider_D = tk.Scale(PIDCoef_variable, from_=0, to=0.1, orient="horizontal", 
                   label="D (Derivative)", length=350, tickinterval=0.01,
                   resolution=0.001)
slider_D.set(sliderCoefDDefault)
slider_D.pack()

# Filter coefficient slider
slider_MG = tk.Scale(PIDCoef_variable, from_=0, to=1, orient="horizontal", 
                    label="FC (Filter Coefficient)", length=350, tickinterval=0.1,
                    resolution=0.01)
slider_MG.set(sliderCoefmgDefault)
slider_MG.pack()

# Control buttons
BShowVideo = tk.Button(OperatorWindow, text="Camera", command=showCameraFrameWindow)
BShowVideo.place(x=110, y=400)

BShowMask = tk.Button(OperatorWindow, text="Show Mask", command=toggleMaskWindow)
BShowMask.place(x=230, y=400) 

BQuit = tk.Button(OperatorWindow, text="Leave", command=endProgam)
BQuit.place(x=320, y=400)

# Display project image
try:
    image = Image.open("image.jpeg")
    image = image.resize((430, 100), Image.ANTIALIAS)
    photo = ImageTk.PhotoImage(image)
    photo_label = tk.Label(OperatorWindow, image=photo)
    photo_label.image = photo  # Keep reference to avoid garbage collection
    photo_label.pack()
    photo_label.place(x=0, y=450)
except:
    # Create placeholder if image not found
    pass

# Credits text
T = tk.Text(OperatorWindow, height=4, width=80)
T.pack()
T.place(x=0, y=550)
T.insert(tk.END, "                  Created By\n  UPM  Hesham")

# Prevent camera window from closing when X is clicked
CameraWindow.protocol("WM_DELETE_WINDOW", donothing)

# ============================================================================
# START THE APPLICATION
# ============================================================================

# Start the main processing loop
main()

# Start the GUI event loop
tk.mainloop() 
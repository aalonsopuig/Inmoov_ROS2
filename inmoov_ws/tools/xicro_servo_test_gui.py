#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# File: xicro_servo_test_gui.py
# Author: Alejandro Alonso Puig
# Date: 2025-05-30
# License: Apache License, Version 2.0
#
# Description:
#   Simple graphical interface in Python (Tkinter) for sending commands to servos
#   of the InMoov robot controlled by ROS 2 nodes generated with XICRO.
#   - Lets the user select a subsystem (1 = right arm, 2 = left arm + head).
#   - Lets the user select a servo from a drop-down list (updated per subsystem).
#   - Lets the user set an angle between 0 and 180 with a slider.
#   - On clicking "Send", publishes the value to the corresponding ROS 2 topic.
#   - "Close" button ends the program.
#
# Usage:
#   source ~/inmoov_ws/install/setup.bash
#   python3 ~/inmoov_ws/tools/xicro_servo_test_gui.py
#
# Requirements:
#   ROS 2 and a running XICRO node for the relevant subsystem.
# -----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import tkinter as tk
from tkinter import ttk

# Servo topic names for each subsystem
SERVOS_SUBSYS1 = [
    "thumb_finger_R",
    "index_finger_R",
    "middle_finger_R",
    "ring_finger_R",
    "pinky_finger_R",
    "bicep_R",
    "rotate_R",
    "shoulder_R",
    "omoplate_R"
]

SERVOS_SUBSYS2 = [
    "thumb_finger_L",
    "index_finger_L",
    "middle_finger_L",
    "ring_finger_L",
    "pinky_finger_L",
    "bicep_L",
    "rotate_L",
    "shoulder_L",
    "omoplate_L",
    "neck",
    "rothead",
    "jaw",
    "eye_x",
    "eye_y"
]

# Map subsystem selection to their servo list and label
SUBSYSTEMS = {
    "Subsystem 1 (Right Arm)": SERVOS_SUBSYS1,
    "Subsystem 2 (Left Arm + Head)": SERVOS_SUBSYS2,
}

class ServoTestNode(Node):
    """
    ROS 2 Node that publishes an Int16 message to the topic for the selected servo.
    """
    def __init__(self):
        super().__init__('servo_test_gui_node')
        # Create publishers for all possible topics in both subsystems
        self.servo_publishers = {}
        for servo_list in SUBSYSTEMS.values():
            for name in servo_list:
                if name not in self.servo_publishers:
                    self.servo_publishers[name] = self.create_publisher(Int16, f'/{name}', 10)

    def publish_angle(self, servo_name, angle):
        """
        Publish the selected angle to the ROS 2 topic for the given servo.
        """
        msg = Int16()
        msg.data = angle
        self.servo_publishers[servo_name].publish(msg)
        self.get_logger().info(f'Published {angle} to /{servo_name}')

def start_gui(node):
    """
    Main GUI loop.
    """
    root = tk.Tk()
    root.title("Test InMoov Servos - Subsystems 1 & 2")
    root.geometry("370x180")

    # GUI variables
    subsystem_names = list(SUBSYSTEMS.keys())
    selected_subsystem = tk.StringVar(value=subsystem_names[0])
    selected_servo = tk.StringVar()
    angle_var = tk.IntVar(value=90)

    # Function to update the servo list when the subsystem changes
    def update_servo_menu(*args):
        servos = SUBSYSTEMS[selected_subsystem.get()]
        menu = servo_menu['menu']
        menu.delete(0, 'end')
        for servo in servos:
            menu.add_command(label=servo, command=lambda s=servo: selected_servo.set(s))
        # Set default servo
        selected_servo.set(servos[0])

    # Subsystem selector
    ttk.Label(root, text="Select subsystem:").pack(pady=(8,0))
    ttk.OptionMenu(root, selected_subsystem, subsystem_names[0], *subsystem_names, command=update_servo_menu).pack()

    # Servo selector (will be updated by update_servo_menu)
    ttk.Label(root, text="Select servo:").pack(pady=(7,0))
    servo_menu = ttk.OptionMenu(root, selected_servo, '', '')  # Placeholder, will be filled by update_servo_menu
    servo_menu.pack()
    update_servo_menu()

    # Angle slider
    ttk.Label(root, text="Angle:").pack(pady=(7,0))
    slider = ttk.Scale(root, from_=0, to=180, orient='horizontal', variable=angle_var)
    slider.pack(fill="x", padx=20)

    angle_label = ttk.Label(root, text="90°")
    angle_label.pack()

    # Update angle label when slider is moved
    def update_label(*args):
        angle_label.config(text=f"{angle_var.get()}°")
    angle_var.trace_add("write", update_label)

    # Send button
    def send():
        angle = int(angle_var.get())
        servo = selected_servo.get()
        node.publish_angle(servo, angle)
    ttk.Button(root, text="Send", command=send).pack(pady=(6,0))

    # Close button
    ttk.Button(root, text="Close", command=root.destroy).pack(pady=(2,7))

    root.mainloop()

def main():
    rclpy.init()
    node = ServoTestNode()
    try:
        start_gui(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

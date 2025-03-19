#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import tkinter as tk
import time
import os
import threading
import queue
import pymycobot
from packaging import version

# Minimum required version of pymycobot
MAX_REQUIRE_VERSION = '3.5.3'

current_version = pymycobot.__version__
print('Current pymycobot library version: {}'.format(current_version))
if version.parse(current_version) > version.parse(MAX_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of the pymycobot library must be less than {}. '
        'The current version is {}. Please downgrade the library version.'.format(MAX_REQUIRE_VERSION, current_version)
    )
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot.mycobot import MyCobot

# Global command queue for asynchronous command buffering
cmd_queue = queue.Queue()

def command_worker(mc):
    """Worker thread that continuously processes commands from the queue."""
    while True:
        try:
            cmd_name, args = cmd_queue.get(timeout=0.1)
            method = getattr(mc, cmd_name)
            method(*args)
            cmd_queue.task_done()
        except queue.Empty:
            continue
        except Exception as e:
            print("Error in command_worker:", e)

class AsyncCobotGUI:
    def __init__(self, handle):
        # Determine available serial port
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline().strip()
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline().strip()
        if self.robot_m5:
            port = self.robot_m5
        else:
            port = self.robot_wio
        print("Port: %s, baud: %d" % (port, 115200))
        self.mc = MyCobot(port, 115200)
        time.sleep(0.05)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)

        # Send an initial pose (modify if a nonzero pose is desired)
        initial_pose = [0, 0, 0, 0, 0, 0]
        self.mc.send_angles(initial_pose, 75)
        time.sleep(1.0)  # Allow the arm to move and settle

        self.win = handle
        self.win.resizable(True, True)  # Fixed window size

        self.model = 0
        self.speed = 50
        self.speed_d = tk.StringVar()
        self.speed_d.set(str(self.speed))

        # Initialize state with default values
        self.record_coords = [[0, 0, 0, 0, 0, 0], self.speed, self.model]
        self.res_angles = [[0, 0, 0, 0, 0, 0], self.speed, self.model]
        self.get_date()  # Retrieve initial state

        # Set window geometry (centered, fixed size 440x440)
        ws = self.win.winfo_screenwidth()
        hs = self.win.winfo_screenheight()
        x = (ws // 2) - 190
        y = (hs // 2) - 250
        self.win.geometry("440x440+{}+{}".format(int(x), int(y)))

        self.set_layout()
        self.need_input()
        self.show_init()

        # Buttons for sending commands (which enqueue commands)
        tk.Button(self.frmLT, text="Set Joints", width=5, command=self.buffer_joint_command).grid(
            row=6, column=1, sticky="w", padx=3, pady=2)
        tk.Button(self.frmRT, text="Set Coords", width=5, command=self.buffer_coord_command).grid(
            row=6, column=1, sticky="w", padx=3, pady=2)

        # Gripper control buttons (widened)
        tk.Button(self.frmLB, text="Gripper (Open)", command=lambda: cmd_queue.put(("set_gripper_state", (0, 80))), width=10).grid(
            row=1, column=0, sticky="w", padx=3, pady=20)
        tk.Button(self.frmLB, text="Gripper (Close)", command=lambda: cmd_queue.put(("set_gripper_state", (1, 80))), width=10).grid(
            row=1, column=1, sticky="w", padx=3, pady=2)

        # Gripper mm control: convert mm to percentage (assuming 200mm = 100%)
        tk.Label(self.frmLB, text="Gripper mm").grid(row=3, column=0, sticky="w", padx=3, pady=2)
        self.gripper_entry = tk.Entry(self.frmLB, width=10)
        self.gripper_entry.grid(row=3, column=1, sticky="w", padx=3, pady=2)
        tk.Button(self.frmLB, text="Set Gripper mm", command=self.buffer_gripper_mm, width=10).grid(
            row=3, column=2, sticky="w", padx=3, pady=2)

        # Start the command worker thread
        self.worker_thread = threading.Thread(target=command_worker, args=(self.mc,), daemon=True)
        self.worker_thread.start()

        # Start a thread to update state and refresh the display continuously
        self.state_thread = threading.Thread(target=self.state_updater, daemon=True)
        self.state_thread.start()

    def set_layout(self):
        self.frmLT = tk.Frame(width=200, height=200)
        self.frmLC = tk.Frame(width=200, height=200)
        self.frmLB = tk.Frame(width=200, height=200)
        self.frmRT = tk.Frame(width=200, height=200)
        self.frmLT.grid(row=0, column=0, padx=1, pady=3)
        self.frmLC.grid(row=1, column=0, padx=1, pady=3)
        self.frmLB.grid(row=1, column=1, padx=2, pady=3)
        self.frmRT.grid(row=0, column=1, padx=2, pady=3)

    def need_input(self):
        # Joint input labels
        tk.Label(self.frmLT, text="Joint 1").grid(row=0)
        tk.Label(self.frmLT, text="Joint 2").grid(row=1)
        tk.Label(self.frmLT, text="Joint 3").grid(row=2)
        tk.Label(self.frmLT, text="Joint 4").grid(row=3)
        tk.Label(self.frmLT, text="Joint 5").grid(row=4)
        tk.Label(self.frmLT, text="Joint 6").grid(row=5)
        # Coordinate input labels
        tk.Label(self.frmRT, text="X").grid(row=0)
        tk.Label(self.frmRT, text="Y").grid(row=1)
        tk.Label(self.frmRT, text="Z").grid(row=2)
        tk.Label(self.frmRT, text="RX").grid(row=3)
        tk.Label(self.frmRT, text="RY").grid(row=4)
        tk.Label(self.frmRT, text="RZ").grid(row=5)
        # Default values for joint inputs (from current state)
        self.j1_default = tk.StringVar(); self.j1_default.set(self.res_angles[0][0])
        self.j2_default = tk.StringVar(); self.j2_default.set(self.res_angles[0][1])
        self.j3_default = tk.StringVar(); self.j3_default.set(self.res_angles[0][2])
        self.j4_default = tk.StringVar(); self.j4_default.set(self.res_angles[0][3])
        self.j5_default = tk.StringVar(); self.j5_default.set(self.res_angles[0][4])
        self.j6_default = tk.StringVar(); self.j6_default.set(self.res_angles[0][5])
        # Default values for coordinate inputs (from current state)
        self.x_default = tk.StringVar(); self.x_default.set(self.record_coords[0][0])
        self.y_default = tk.StringVar(); self.y_default.set(self.record_coords[0][1])
        self.z_default = tk.StringVar(); self.z_default.set(self.record_coords[0][2])
        self.rx_default = tk.StringVar(); self.rx_default.set(self.record_coords[0][3])
        self.ry_default = tk.StringVar(); self.ry_default.set(self.record_coords[0][4])
        self.rz_default = tk.StringVar(); self.rz_default.set(self.record_coords[0][5])
        # Joint entry fields
        self.J_1 = tk.Entry(self.frmLT, textvariable=self.j1_default); self.J_1.grid(row=0, column=1, pady=3)
        self.J_2 = tk.Entry(self.frmLT, textvariable=self.j2_default); self.J_2.grid(row=1, column=1, pady=3)
        self.J_3 = tk.Entry(self.frmLT, textvariable=self.j3_default); self.J_3.grid(row=2, column=1, pady=3)
        self.J_4 = tk.Entry(self.frmLT, textvariable=self.j4_default); self.J_4.grid(row=3, column=1, pady=3)
        self.J_5 = tk.Entry(self.frmLT, textvariable=self.j5_default); self.J_5.grid(row=4, column=1, pady=3)
        self.J_6 = tk.Entry(self.frmLT, textvariable=self.j6_default); self.J_6.grid(row=5, column=1, pady=3)
        # Coordinate entry fields
        self.x = tk.Entry(self.frmRT, textvariable=self.x_default); self.x.grid(row=0, column=1, pady=3, padx=0)
        self.y = tk.Entry(self.frmRT, textvariable=self.y_default); self.y.grid(row=1, column=1, pady=3)
        self.z = tk.Entry(self.frmRT, textvariable=self.z_default); self.z.grid(row=2, column=1, pady=3)
        self.rx = tk.Entry(self.frmRT, textvariable=self.rx_default); self.rx.grid(row=3, column=1, pady=3)
        self.ry = tk.Entry(self.frmRT, textvariable=self.ry_default); self.ry.grid(row=4, column=1, pady=3)
        self.rz = tk.Entry(self.frmRT, textvariable=self.rz_default); self.rz.grid(row=5, column=1, pady=3)
        self.all_j = [self.J_1, self.J_2, self.J_3, self.J_4, self.J_5, self.J_6]
        self.all_c = [self.x, self.y, self.z, self.rx, self.ry, self.rz]
        # Speed entry field
        tk.Label(self.frmLB, text="Speed").grid(row=0, column=0)
        self.get_speed = tk.Entry(self.frmLB, textvariable=self.speed_d, width=10)
        self.get_speed.grid(row=0, column=1)

    def show_init(self):
        # Display joint labels
        tk.Label(self.frmLC, text="Joint 1").grid(row=0)
        tk.Label(self.frmLC, text="Joint 2").grid(row=1)
        tk.Label(self.frmLC, text="Joint 3").grid(row=2)
        tk.Label(self.frmLC, text="Joint 4").grid(row=3)
        tk.Label(self.frmLC, text="Joint 5").grid(row=4)
        tk.Label(self.frmLC, text="Joint 6").grid(row=5)
        # Setup display variables for joints
        self.cont_1 = tk.StringVar(self.frmLC); self.cont_1.set(str(self.res_angles[0][0]) + "°")
        self.cont_2 = tk.StringVar(self.frmLC); self.cont_2.set(str(self.res_angles[0][1]) + "°")
        self.cont_3 = tk.StringVar(self.frmLC); self.cont_3.set(str(self.res_angles[0][2]) + "°")
        self.cont_4 = tk.StringVar(self.frmLC); self.cont_4.set(str(self.res_angles[0][3]) + "°")
        self.cont_5 = tk.StringVar(self.frmLC); self.cont_5.set(str(self.res_angles[0][4]) + "°")
        self.cont_6 = tk.StringVar(self.frmLC); self.cont_6.set(str(self.res_angles[0][5]) + "°")
        self.cont_all = [self.cont_1, self.cont_2, self.cont_3, self.cont_4, self.cont_5, self.cont_6, self.speed, self.model]

        tk.Label(self.frmLC, textvariable=self.cont_1, font=("Arial", 9), width=7, height=1, bg="white").grid(row=0, column=1, padx=0, pady=5)
        tk.Label(self.frmLC, textvariable=self.cont_2, font=("Arial", 9), width=7, height=1, bg="white").grid(row=1, column=1, padx=0, pady=5)
        tk.Label(self.frmLC, textvariable=self.cont_3, font=("Arial", 9), width=7, height=1, bg="white").grid(row=2, column=1, padx=0, pady=5)
        tk.Label(self.frmLC, textvariable=self.cont_4, font=("Arial", 9), width=7, height=1, bg="white").grid(row=3, column=1, padx=0, pady=5)
        tk.Label(self.frmLC, textvariable=self.cont_5, font=("Arial", 9), width=7, height=1, bg="white").grid(row=4, column=1, padx=0, pady=5)
        tk.Label(self.frmLC, textvariable=self.cont_6, font=("Arial", 9), width=7, height=1, bg="white").grid(row=5, column=1, padx=5, pady=5)
        self.all_jo = [self.cont_1, self.cont_2, self.cont_3, self.cont_4, self.cont_5, self.cont_6]

        # Display coordinate labels
        tk.Label(self.frmLC, text="  x ").grid(row=0, column=3)
        tk.Label(self.frmLC, text="  y ").grid(row=1, column=3)
        tk.Label(self.frmLC, text="  z ").grid(row=2, column=3)
        tk.Label(self.frmLC, text="  rx ").grid(row=3, column=3)
        tk.Label(self.frmLC, text="  ry ").grid(row=4, column=3)
        tk.Label(self.frmLC, text="  rz ").grid(row=5, column=3)
        self.coord_x = tk.StringVar(); self.coord_x.set(str(self.record_coords[0][0]))
        self.coord_y = tk.StringVar(); self.coord_y.set(str(self.record_coords[0][1]))
        self.coord_z = tk.StringVar(); self.coord_z.set(str(self.record_coords[0][2]))
        self.coord_rx = tk.StringVar(); self.coord_rx.set(str(self.record_coords[0][3]))
        self.coord_ry = tk.StringVar(); self.coord_ry.set(str(self.record_coords[0][4]))
        self.coord_rz = tk.StringVar(); self.coord_rz.set(str(self.record_coords[0][5]))
        self.coord_all = [self.coord_x, self.coord_y, self.coord_z, self.coord_rx, self.coord_ry, self.coord_rz, self.speed, self.model]

        tk.Label(self.frmLC, textvariable=self.coord_x, font=("Arial", 9), width=7, height=1, bg="white").grid(row=0, column=4, padx=5, pady=5)
        tk.Label(self.frmLC, textvariable=self.coord_y, font=("Arial", 9), width=7, height=1, bg="white").grid(row=1, column=4, padx=5, pady=5)
        tk.Label(self.frmLC, textvariable=self.coord_z, font=("Arial", 9), width=7, height=1, bg="white").grid(row=2, column=4, padx=5, pady=5)
        tk.Label(self.frmLC, textvariable=self.coord_rx, font=("Arial", 9), width=7, height=1, bg="white").grid(row=3, column=4, padx=5, pady=5)
        tk.Label(self.frmLC, textvariable=self.coord_ry, font=("Arial", 9), width=7, height=1, bg="white").grid(row=4, column=4, padx=5, pady=5)
        tk.Label(self.frmLC, textvariable=self.coord_rz, font=("Arial", 9), width=7, height=1, bg="white").grid(row=5, column=4, padx=5, pady=5)

        # Display unit (mm)
        self.unit = tk.StringVar()
        self.unit.set("mm")
        for i in range(6):
            tk.Label(self.frmLC, textvariable=self.unit, font=("Arial", 9)).grid(row=i, column=5)

    def gripper_open(self):
        try:
            self.mc.set_gripper_state(0, 80)
        except Exception as e:
            pass

    def gripper_close(self):
        try:
            self.mc.set_gripper_state(1, 80)
        except Exception as e:
            pass

    def get_date(self):
        # Retrieve robot data for display; use defaults if data is not returned
        t = time.time()
        try:
            coords = self.mc.get_coords()
        except Exception as e:
            print("Error getting coords:", e)
            coords = None
        while time.time() - t < 2 and (coords is None or coords == []):
            time.sleep(0.1)
            try:
                coords = self.mc.get_coords()
            except Exception as e:
                coords = None
        if not coords or coords == []:
            coords = [0, 0, 0, 0, 0, 0]

        t = time.time()
        try:
            angles = self.mc.get_angles()
        except Exception as e:
            print("Error getting angles:", e)
            angles = None
        while time.time() - t < 2 and (angles is None or angles == []):
            time.sleep(0.1)
            try:
                angles = self.mc.get_angles()
            except Exception as e:
                angles = None
        if not angles or angles == []:
            angles = [0, 0, 0, 0, 0, 0]

        self.record_coords[0] = coords
        self.res_angles[0] = angles

    def show_j_date(self, data, way=""):
        # Update the GUI display with the provided data
        if way == "coord":
            for i, var in zip(data, self.coord_all):
                var.set(str(i))
        else:
            for i, var in zip(data, self.cont_all):
                var.set(str(i) + "°")

    def buffer_joint_command(self):
        self.get_date()  # Update state before sending command
        j_value = []
        for entry in self.all_j:
            try:
                j_value.append(float(entry.get()))
            except Exception:
                j_value.append(0)
        speed = int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        cmd_queue.put(("send_angles", (j_value, speed)))
        self.show_j_date(j_value)

    def buffer_coord_command(self):
        self.get_date()  # Update state before sending command
        c_value = []
        for entry in self.all_c:
            try:
                c_value.append(float(entry.get()))
            except Exception:
                c_value.append(0)
        speed = int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        cmd_queue.put(("send_coords", (c_value, speed, self.model)))
        self.show_j_date(c_value, "coord")

    def buffer_gripper_mm(self):
        try:
            mm_value = float(self.gripper_entry.get())
            speed = 75
            max_mm = 200.0
            percentage = (mm_value / max_mm) * 100
            if percentage < 0:
                percentage = 0
            if percentage > 100:
                percentage = 100
            cmd_queue.put(("set_gripper_value", (percentage, speed, 1)))
        except Exception as e:
            print("Error buffering gripper mm command:", e)

    def state_updater(self):
        """Continuously update robot state and refresh the display."""
        while True:
            try:
                self.get_date()
                self.update_display()
                time.sleep(0.1)
            except Exception as e:
                print("State updater error:", e)
                time.sleep(0.1)

    def update_display(self):
        try:
            for idx, val in enumerate(self.res_angles[0]):
                self.cont_all[idx].set(str(val) + "°")
            for idx, val in enumerate(self.record_coords[0]):
                self.coord_all[idx].set(str(val))
        except Exception as e:
            pass

    def run(self):
        while True:
            try:
                self.win.update()
                time.sleep(0.001)
            except tk.TclError as e:
                if "application has been destroyed" in str(e):
                    break
                else:
                    raise

def main():
    root = tk.Tk()
    root.title("MyCobot ROS GUI")
    gui = AsyncCobotGUI(root)
    gui.run()

if __name__ == "__main__":
    main()

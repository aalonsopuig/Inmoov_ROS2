#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Programa: xicro_servo_test_gui.py
# Fecha: 27 de mayo de 2025
# Autor: Alejandro Alonso Puig
# Descripción:
#   Interfaz gráfica simple en Python (Tkinter) para enviar comandos a servos
#   del subsistema 1 (Arduino Uno) del robot InMoov. Permite:
#     - Seleccionar un servo de una lista desplegable.
#     - Ajustar un ángulo entre 0 y 180 con un slider.
#     - Publicar el valor al topic ROS 2 correspondiente al pulsar "Enviar".
#     - Cerrar el programa con un botón "Cerrar".
#
#   Requiere ROS 2 activo y el nodo Python generado por XICRO ejecutándose.
#
# Uso:
#   source ~/inmoov_ws/install/setup.bash
#   python3 ~/inmoov_ws/tools/xicro_servo_test_gui.py
# -----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import tkinter as tk
from tkinter import ttk

# Lista de nombres de los servos del subsistema 1 (brazo derecho)
SERVOS = [
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

# Nodo ROS 2 que publica un mensaje std_msgs/Int16 al topic del servo seleccionado
class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test_gui_node')
        # Usar un nombre que no colisione con propiedades internas de rclpy
        self.servo_publishers = {
            name: self.create_publisher(Int16, f'/{name}', 10)
            for name in SERVOS
        }

    def publish_angle(self, servo_name, angle):
        # Publicar el valor de ángulo al topic del servo seleccionado
        msg = Int16()
        msg.data = angle
        self.servo_publishers[servo_name].publish(msg)
        self.get_logger().info(f'Publicado {angle} en /{servo_name}')

# Función principal de la interfaz gráfica
def start_gui(node):
    root = tk.Tk()
    root.title("Test Servos InMoov - Subsistema 1")
    root.geometry("350x150")

    selected_servo = tk.StringVar(value=SERVOS[0])  # Servo seleccionado
    angle_var = tk.IntVar(value=90)                 # Ángulo del slider

    # Menú desplegable para seleccionar servo
    ttk.Label(root, text="Selecciona un servo:").pack(pady=5)
    ttk.OptionMenu(root, selected_servo, SERVOS[0], *SERVOS).pack()

    # Slider de ángulo
    ttk.Label(root, text="Ángulo:").pack()
    slider = ttk.Scale(root, from_=0, to=180, orient='horizontal', variable=angle_var)
    slider.pack(fill="x", padx=20)

    # Etiqueta que muestra el ángulo actual
    angle_label = ttk.Label(root, text="90°")
    angle_label.pack()

    # Actualizar etiqueta cuando se mueve el slider
    def update_label(*args):
        angle_label.config(text=f"{angle_var.get()}°")

    angle_var.trace_add("write", update_label)

    # Botón para enviar comando al topic ROS
    def enviar():
        angle = int(angle_var.get())
        servo = selected_servo.get()
        node.publish_angle(servo, angle)

    ttk.Button(root, text="Enviar", command=enviar).pack(pady=5)

    # Botón para cerrar ventana
    ttk.Button(root, text="Cerrar", command=root.destroy).pack()

    root.mainloop()

# Función principal del nodo ROS 2 + GUI
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

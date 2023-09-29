# JoyLab: A PyQt5-based Servo-Actuated Joystick Control Interface

## Overview

JoyLab is a Python application that provides a graphical interface for controlling a servo-actuated joystick. The application is built using PyQt5 and pyqtgraph for the GUI, and it communicates with the joystick via full-duplex serial connection. The interface allows users to control the joystick mode, servo control mode, and other settings while displaying real-time data plots.

## Features

- Real-time plotting of joystick and servo positions
- Multiple joystick modes (Center, Follow, Walls, Maze)
- Multiple servo control modes (Position Control, Current Based Position Control, Current PID Position Control)
- Torque enable/disable via a checkbox
- Current limit control via a slider
- Mouse control override
- Serial communication with servos

## Dependencies

- PyQt5
- pyqtgraph
- Python's `serial` library
- Python's `threading` library
- NumPy

## Installation

1. Install PyQt5 and pyqtgraph:
    ```bash
    pip install PyQt5 pyqtgraph
    ```
2. Install Python's `serial` library:
    ```bash
    pip install pyserial
    ```
3. Clone the repository and navigate to the project directory.

## Usage

1. Run the application:
    ```bash
    python main.py
    ```
2. The GUI will appear, allowing you to control the joystick and servos.

## Classes

### `CustomPlotWidget`

- Inherits from `PlotWidget` and adds mouse tracking functionality.
- Emits a signal containing the mouse position when the mouse is clicked and dragged on the plot.

### `JoyLab`

- Main application class that initializes the GUI and handles events.
- Manages the `SerialManager` for serial communication.

### `SerialManager`

- Manages the serial communication with the servos.
- Reads servo positions and sends control messages.

## Signals and Slots

- `new_position_signal`: Emitted by `SerialManager` when new servo positions are read.
- `mouse_pos_signal`: Emitted by `CustomPlotWidget` when the mouse is clicked and dragged.

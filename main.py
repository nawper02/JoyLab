from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QComboBox, QSlider, QLabel, QCheckBox, QPushButton
from PyQt5.QtCore import pyqtSignal, QObject, Qt, QTimer, QSize
from pyqtgraph import PlotWidget
import sys
import serial
import threading
from collections import deque
import time
import numpy as np


class CustomPlotWidget(PlotWidget):
    """
    This small class adds the ability to control the robot by clicking on the plot.
    """
    mouse_pos_signal = pyqtSignal(list)

    def __init__(self, parent, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.parent = parent
        self.setMouseTracking(True)
        self.mouse_dragging = False
        self.plotItem.vb.setMouseEnabled(x=False, y=False)
        self.plotItem.vb.setXRange(2000, 3200)
        self.plotItem.vb.setYRange(2000, 3350)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_mouse_pos)
        self.timer.setInterval(16)

    def mousePressEvent(self, event):
        self.mouse_dragging = True
        self.parent.user_is_dragging = True
        self.timer.start()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self.mouse_dragging = False
        self.parent.user_is_dragging = False
        self.timer.stop()
        super().mouseReleaseEvent(event)

    #def mouseMoveEvent(self, event):
    #    if self.mouse_dragging:
    #        pos = event.pos()
    #        if self.plotItem.vb.sceneBoundingRect().contains(pos):
    #            mouse_point = self.plotItem.vb.mapSceneToView(pos)
    #            pos = [mouse_point.x(), mouse_point.y()]
    #            self.mouse_pos_signal.emit(pos)  # Emit the signal
    #    super().mouseMoveEvent(event)

    def update_mouse_pos(self):
        # This function will be called periodically when the mouse is held down
        pos = self.mapFromGlobal(self.cursor().pos())
        if self.plotItem.vb.sceneBoundingRect().contains(pos):
            mouse_point = self.plotItem.vb.mapSceneToView(pos)
            pos = [mouse_point.x(), mouse_point.y()]
            self.mouse_pos_signal.emit(pos)  # Emit the signal


class JoyLab(QMainWindow):
    def __init__(self):
        super().__init__()
        self.servo_mode = 'Current Based Position Control'
        self.joystick_mode = 'Center'
        self.user_is_dragging = False  # For overriding controls with user input
        self.is_test_sequence_running = False  # For overriding controls with test sequence
        self.user_mouse_pos = None  # For storing the user's mouse position
        self.center_pos = [2589, 2667]

        # Data buffers
        self.present_positions = deque(maxlen=100)
        self.goal_positions = deque(maxlen=100)
        self.timestamps = deque(maxlen=100)
        self.errors = deque(maxlen=100)

        self.serial_manager = SerialManager(self, port="/dev/cu.usbmodem14101")  # Replace with your port name
        self.serial_manager.connect()

        # Plot for joystick position
        self.plot = CustomPlotWidget(self)
        self.plot.setFixedSize(QSize(400, 400))

        # Plot for error
        self.error_plot = CustomPlotWidget(self)
        self.error_plot.setFixedSize(QSize(800, 200))
        self.error_plot.plotItem.setLabel('left', 'Control Magnitudes')
        self.error_plot.plotItem.setLabel('bottom', 'Time')

        # Create the dropdowns
        self.joystick_mode_dropdown = QComboBox()
        self.joystick_mode_dropdown.addItem("Center")
        self.joystick_mode_dropdown.addItem("Follow")
        self.joystick_mode_dropdown.addItem("Walls")
        self.joystick_mode_dropdown.addItem("Maze")
        self.joystick_mode_dropdown.setCurrentText(self.joystick_mode)

        self.servo_mode_dropdown = QComboBox()
        self.servo_mode_dropdown.addItem('Position Control')
        self.servo_mode_dropdown.addItem('Current Based Position Control')
        self.servo_mode_dropdown.addItem('Current PID Position Control')
        self.servo_mode_dropdown.setCurrentText(self.servo_mode)

        # Create the current_limit_slider
        self.current_limit_slider = QSlider(Qt.Horizontal)
        self.current_limit_slider.setMinimum(0)
        self.current_limit_slider.setMaximum(1500)
        self.current_limit_slider.setValue(1500)

        # Create checkbox for torque enable
        self.torque_checkbox = QCheckBox("Enable Torque")
        self.torque_checkbox.setChecked(True)

        # Create a pushbutton for test sequence
        # Create the test sequence button
        self.test_sequence_button = QPushButton("Run Test Sequence")
        self.test_sequence_button.clicked.connect(self.start_test_sequence_thread)

        # Create labels for the dropdowns and current_limit_slider
        joystick_mode_label = QLabel('Joystick Mode:')
        servo_mode_label = QLabel('Servo Control Mode:')
        slider_label = QLabel('Servo Current Limit (%):')
        self.current_limit_slider.setMinimumHeight(50)  # Adjust the height to stop handle icon clipping

        # Create the layout and add widgets
        layout = QVBoxLayout()
        layout.addWidget(self.torque_checkbox)
        layout.addWidget(self.test_sequence_button)
        layout.addWidget(joystick_mode_label)
        layout.addWidget(self.joystick_mode_dropdown)
        layout.addWidget(servo_mode_label)
        layout.addWidget(self.servo_mode_dropdown)
        layout.addWidget(slider_label)
        layout.addWidget(self.current_limit_slider)
        layout.addWidget(self.plot)
        layout.addWidget(self.error_plot)

        container = QWidget()
        container.setLayout(layout)

        self.setCentralWidget(container)

        # Connect signals to slots
        self.serial_manager.new_position_signal.connect(self.on_new_position_signal)
        self.plot.mouse_pos_signal.connect(self.mouse_control)
        self.joystick_mode_dropdown.currentIndexChanged.connect(self.on_joystick_mode_changed)
        self.servo_mode_dropdown.currentIndexChanged.connect(self.on_servo_mode_changed)
        self.current_limit_slider.valueChanged.connect(self.on_current_limit_changed)
        self.torque_checkbox.stateChanged.connect(self.on_toggle_torque)

        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(16)  # ~60 Hz

        self.thread = threading.Thread(target=self.serial_manager.main_loop)
        self.thread.start()

    def on_joystick_mode_changed(self, index):
        selected_option = self.joystick_mode_dropdown.itemText(index)
        self.joystick_mode = selected_option

    def on_servo_mode_changed(self, index):
        selected_option = self.servo_mode_dropdown.itemText(index)
        self.servo_mode = selected_option
        servo_modes_map = {'Position Control': 3, 'Current Based Position Control': 5, 'Current PID Position Control': 0}
        self.serial_manager.send_message('set_servo_mode', [servo_modes_map[selected_option]])

    def on_current_limit_changed(self, value):
        self.serial_manager.send_message('set_current_limit', [value])
        # this is a weird workaround to get current limit changes to save.
        # if in current based position mode, switch to position mode and back.
        if self.servo_mode == 'Current Based Position Control':
            self.serial_manager.send_message('set_servo_mode', [3])
            self.serial_manager.send_message('set_servo_mode', [5])

    def on_toggle_torque(self, state):
        if state == Qt.Checked:
            self.serial_manager.send_message('toggle_torque', [1])
        else:
            self.serial_manager.send_message('toggle_torque', [0])

    def update_plot(self):
        # Update joystick plot
        if len(self.present_positions) > 0:  # Check if we have any data to plot
            last_pos = self.present_positions[-1]  # Get the most recent position
            self.plot.clear()
            self.plot.plot([last_pos[0]], [last_pos[1]], pen=None,
                           symbol='o', symbolPen=None, symbolSize=10, symbolBrush=(255, 0, 0, 255))

        # Update error plot
        if len(self.present_positions) > 0 and len(self.goal_positions) > 0:
            # Make sure the lengths are the same
            min_length = min(len(self.timestamps), len(self.present_positions), len(self.goal_positions))

            # Calculate the magnitudes of present and goal positions
            present_magnitudes = [np.linalg.norm(np.array(pos)) for pos in list(self.present_positions)[:min_length]]
            goal_magnitudes = [np.linalg.norm(np.array(pos)) for pos in list(self.goal_positions)[:min_length]]
            trimmed_timestamps = list(self.timestamps)[:min_length]

            # Plot the magnitudes
            self.error_plot.plot(trimmed_timestamps, present_magnitudes, pen='r', name="Present Position", clear=True)
            self.error_plot.plot(trimmed_timestamps, goal_magnitudes, pen='b', name="Goal Position")

            # Set Y-axis range (optional)
            self.error_plot.plotItem.vb.setYRange(3000, 5000)

            # Set X-axis to only show the past 10 seconds
            if len(trimmed_timestamps) > 1:
                self.error_plot.plotItem.vb.setXRange(max(trimmed_timestamps) - 1.5, max(trimmed_timestamps))

    def on_new_position_signal(self, pos):
        self.present_positions.append(pos)
        try:
            error = np.linalg.norm(np.array(self.goal_positions[-1]) - np.array(self.present_positions[-1]))
            self.errors.append(error)
        except TypeError:
            pass
        self.timestamps.append(time.time())
        #self.update_plot(pos)

    def mouse_control(self, pos):
        self.user_mouse_pos = [int(pos[0]), int(pos[1])]

    def calculate_standard_position(self):
        goal_pos = None
        match self.joystick_mode:
            case 'Center':
                goal_pos = self.user_mouse_pos if self.user_is_dragging else self.center_pos
            case 'Follow':
                goal_pos = self.user_mouse_pos if self.user_is_dragging else self.present_positions[-1]
            case 'Walls':
                pass
            case 'Maze':
                pass
        if goal_pos is not None:
            self.goal_positions.append(goal_pos)
        return goal_pos

    def calculate_pid_currents(self):
        if self.user_mouse_pos is None:
            return None  # in future, should match self.joystick_mode
        else:
            return None

    def start_test_sequence_thread(self):
        """
        Start the test sequence in its own thread.
        """
        test_sequence_thread = threading.Thread(target=self.run_test_sequence)
        test_sequence_thread.start()

    def run_test_sequence(self):
        """
        Function to command the robotic joystick to move in a circle.

        """
        self.is_test_sequence_running = True
        radius_in_encoder_ticks = 300

        # Circle in one direction
        for angle in np.linspace(0, 2 * np.pi, 250):
            x = int(self.center_pos[0] + radius_in_encoder_ticks * np.cos(angle))
            y = int(self.center_pos[1] + radius_in_encoder_ticks * np.sin(angle))
            self.goal_positions.append([x, y])
            self.serial_manager.send_message('goal_positions', [x, y])

        # Circle in the opposite direction
        for angle in np.linspace(2 * np.pi, 0, 250):
            x = int(self.center_pos[0] + radius_in_encoder_ticks * np.cos(angle))
            y = int(self.center_pos[1] + radius_in_encoder_ticks * np.sin(angle))
            self.goal_positions.append([x, y])
            self.serial_manager.send_message('goal_positions', [x, y])

        self.serial_manager.send_message('goal_positions', self.center_pos)
        time.sleep(0.3)

        for i in range(2):
            # Trace out a '+' pattern
            for offset in np.linspace(-radius_in_encoder_ticks, radius_in_encoder_ticks, 300):
                self.goal_positions.append([self.center_pos[0] + offset, self.center_pos[1]])
                self.serial_manager.send_message('goal_positions', [self.center_pos[0] + offset, self.center_pos[1]])

            self.serial_manager.send_message('goal_positions', self.center_pos)

            for offset in np.linspace(-radius_in_encoder_ticks, radius_in_encoder_ticks, 300):
                self.goal_positions.append([self.center_pos[0], self.center_pos[1] + offset])
                self.serial_manager.send_message('goal_positions', [self.center_pos[0], self.center_pos[1] + offset])

            self.serial_manager.send_message('goal_positions', self.center_pos)

        # Circle in one direction
        for angle in np.linspace(0, 2 * np.pi, 300):
            x = int(self.center_pos[0] + radius_in_encoder_ticks * np.cos(angle))
            y = int(self.center_pos[1] + radius_in_encoder_ticks * np.sin(angle))
            self.goal_positions.append([x, y])
            self.serial_manager.send_message('goal_positions', [x, y])

        time.sleep(0.8)
        self.serial_manager.send_message('goal_positions', self.center_pos)

        """
        # Spiral outwards
        for radius in np.linspace(50, radius_in_encoder_ticks, 25):  # Reduced to 25 points
            for angle in np.linspace(0, 2 * np.pi, 50):  # Reduced to 50 points
                x = int(self.center_pos[0] + radius * np.cos(angle))
                y = int(self.center_pos[1] + radius * np.sin(angle))
                self.serial_manager.send_message('goal_positions', [x, y])

        # Spiral back to the center
        for radius in np.linspace(radius_in_encoder_ticks, 50, 25):  # Reduced to 25 points
            for angle in np.linspace(0, 2 * np.pi, 50):  # Reduced to 50 points
                x = int(self.center_pos[0] + radius * np.cos(angle))
                y = int(self.center_pos[1] + radius * np.sin(angle))
                self.serial_manager.send_message('goal_positions', [x, y])
        """

        self.is_test_sequence_running = False


class SerialManager(QObject):

    new_position_signal = pyqtSignal(list)

    def __init__(self, joy, port, baudrate=1000000):
        super().__init__()
        self.joylab = joy
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate)
        self.ser.flushInput()  # Flush the input buffer

    def disconnect(self):
        if self.ser:
            self.ser.close()

    def read_servo_positions(self):
        if self.ser.inWaiting() > 0:
            data_str = self.ser.readline().decode('utf-8').rstrip()
            self.ser.flushInput()

            tokens = data_str.split(',')
            if len(tokens) != 2:
                return None
            return [int(tokens[0]), int(tokens[1])]  # Positions
            # ValueError Here

    def send_message(self, topic, values):
        if self.ser:
            try:
                data_str = f"{topic},{','.join(map(str, values))}\n"
                self.ser.write(data_str.encode('utf-8'))
                self.ser.flush()
            except TypeError:
                pass

    def main_loop(self):
        while True:
            positions = self.read_servo_positions()
            if positions:
                self.new_position_signal.emit(positions)

            if not self.joylab.is_test_sequence_running:
                match self.joylab.servo_mode:
                    case 'Current Based Position Control':
                        positions = self.joylab.calculate_standard_position()
                        self.send_message('goal_positions', positions)
                    case 'Position Control':
                        goal_positions = self.joylab.calculate_standard_position()
                        self.send_message('goal_positions', goal_positions)
                    case 'Current PID Position Control':
                        goal_currents = self.joylab.calculate_pid_currents()
                        self.send_message('goal_currents', goal_currents)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    joylab = JoyLab()
    joylab.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        joylab.serial_manager.disconnect()

"""
                 @@@@@@@@@@@@@%                        @@@@@@@.
            ,@@@@@@@@@@@@      @@@@.                         .,@@@,
          @@@@@@@@@@@@@@@          @@,                           .@@@.
        @@@@@@@@@@@@@@@@@            @@,                            @@@.
       @@@@@@@@@@@@@@@@@@             @@*         @@@@@@@@@@/         @@,
      @@@@@@@@@@@@@@@@@@@              @@,      @@@@@@@@*. .@@@        @@.
      @@@@@@@@@@@@@@@@@@@              (@#.    @@@@@@@@@*.   @@*.      @@,
      @@@@@@@@@@@@@@@@@@@              *@%.@@, @@@@@@@@@*.   @@*.      @@*
      @@@@@@@@@@@@@@@@@@@              @@*.@@, @@@@@@@@@*.  @@@,       @@*
      @@@@@@@@@@@@@@@@@@@             @@#,@@*. @@*@@@@@@@@@@@*.        @@*
      @@@@@@@@@@@@@@@@@@@            @@/,@@*,  @@*   .@@/,.            @@*
      @@,.@@@@@@@@@@@@@@@          @@@*.@@*.   @@*    @@*.             @@*
      @@,  .@@@@@@@@@@@@@       @@@*,(@@*,/@@@/*,..@@@@@@@@&           @@*
      @@,      ,@@@@@@@@@@@@@@@**.#@@@*,@@@*.@@@@/    @@@@@@@@@@@.     @@*
      @@,            ..@@,..  @@@@**. @@*,@@@         @@@@@@@@@@@@@@.  @@*
      @@,              @@,    @@,   @@/,@@@           @@@@@@@@@@@@@@@@,@@*
      @@,           @@@@@@@,  @@,  @@*.@@             @@@@@@@@@@@@@@@@@@@*
      @@,        @@@   @@@@@@@@@, @@*.@@              @@@@@@@@@@@@@@@@@@@*
      @@,       @@     @@@@@@@@@, @@-/@#              @@@@@@@@@@@@@@@@@@@*
      @@,      &@      @@@@@@@@@, .*.@@               @@@@@@@@@@@@@@@@@@@*
      @@,       @@     @@@@@@@@/.     @@              @@@@@@@@@@@@@@@@@@@,
       @@.       @@@   @@@@@@@*.      @@              @@@@@@@@@@@@@@@@@@*.
       .@@.        .,@@@@@(*,          @@.            @@@@@@@@@@@@@@@@@/,
         @@@.                           .@@           @@@@@@@@@@@@@@@@*.
           @@@,                           .@@@        @@@@@@@@@@@@@@*.
             .*@@@@@                         .@@@@@.  @@@@@@@@@@/*.
                  .,,*/(,                         .,**/(/***,..
"""

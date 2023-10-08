from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QComboBox, QSlider, QLabel, QCheckBox, \
    QPushButton, QLineEdit, QHBoxLayout
from PyQt5.QtCore import pyqtSignal, QObject, Qt, QTimer, QSize
from pyqtgraph import PlotWidget, ImageItem
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
        self.plotItem.vb.setXRange(0, 1)
        self.plotItem.vb.setYRange(0, 1)
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

    # def mouseMoveEvent(self, event):
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
        self.center_pos = [2589, 2667]  # Center position of joystick
        self.range = 600  # Range of joystick
        self.in_bounds = True  # Whether the joystick is in bounds defined for walls mode
        self.prev_in_bounds = True  # Whether the joystick was in bounds in the previous iteration
        self.servo_modes_map = {'Position Control': 3, 'Current Based Position Control': 5,
                                'Current PID Position Control': 0}

        # Data buffers
        self.present_positions = deque(maxlen=100)
        self.goal_positions = deque(maxlen=100)
        self.timestamps = deque(maxlen=100)
        self.errors = deque(maxlen=100)

        # PID
        self.Kp = 0.3  # Proportional gain
        self.Ki = 0.000  # Integral gain
        self.Kd = 2.0  # Derivative gain
        self.prev_errors = deque([[0, 0]], maxlen=100)  # Initialize with zero error
        self.integral = [0, 0]  # Initialize integral term

        self.serial_manager = SerialManager(self, port="/dev/cu.usbmodem14101")  # Replace with your port name
        self.serial_manager.connect()

        # Plot for joystick position
        self.plot = CustomPlotWidget(self)
        self.plot.setFixedSize(QSize(300, 300))

        # Walls mode boundary image
        self.boundary_image = np.zeros((1000, 1000))
        self.boundary_image[250:750, 250:750] = 255

        # Create an ImageItem
        self.img_item = ImageItem(image=self.boundary_image)

        # Set the position and scale to fit within the normalized range
        self.img_item.setPos(0, 0)
        self.img_item.setScale(0.001)  # Scale down to fit within 0 to 1

        # Plot for error
        self.error_plot = CustomPlotWidget(self)
        self.error_plot.setFixedSize(QSize(500, 200))
        self.error_plot.plotItem.setLabel('left', 'Control Magnitudes')
        self.error_plot.plotItem.setLabel('bottom', 'Time')

        # Create the dropdowns
        self.joystick_mode_dropdown = QComboBox()
        self.joystick_mode_dropdown.addItem("Center")
        self.joystick_mode_dropdown.addItem("Follow")
        self.joystick_mode_dropdown.addItem("Walls")
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

        # Create QLineEdit widgets for PID gains
        self.Kp_textbox = QLineEdit(str(self.Kp))
        self.Ki_textbox = QLineEdit(str(self.Ki))
        self.Kd_textbox = QLineEdit(str(self.Kd))

        # Create labels for the PID gain textboxes
        Kp_label = QLabel('Kp:')
        Ki_label = QLabel('Ki:')
        Kd_label = QLabel('Kd:')

        # Add a button to update the PID gains
        update_pid_button = QPushButton("Update PID Gains")
        update_pid_button.clicked.connect(self.update_pid_gains)

        horizontal_layout = QHBoxLayout()
        horizontal_layout.addWidget(Kp_label)
        horizontal_layout.addWidget(self.Kp_textbox)
        horizontal_layout.addWidget(Ki_label)
        horizontal_layout.addWidget(self.Ki_textbox)
        horizontal_layout.addWidget(Kd_label)
        horizontal_layout.addWidget(self.Kd_textbox)
        horizontal_layout.addWidget(update_pid_button)
        # Create a container widget for the horizontal layout
        horizontal_container = QWidget()

        # Set the horizontal layout as the layout for the container widget
        horizontal_container.setLayout(horizontal_layout)

        # Create the layout and add widgets
        layout = QVBoxLayout()
        layout.addWidget(self.torque_checkbox)
        layout.addWidget(self.test_sequence_button)
        layout.addWidget(joystick_mode_label)
        layout.addWidget(self.joystick_mode_dropdown)
        layout.addWidget(servo_mode_label)
        layout.addWidget(self.servo_mode_dropdown)
        layout.addWidget(horizontal_container)
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
        self.serial_manager.send_message('set_servo_mode', [self.servo_modes_map[selected_option]])

    def on_current_limit_changed(self, value):
        self.serial_manager.send_message('set_current_limit', [value])
        # this is a weird workaround to get current limit changes to save.
        # if in current based position mode, switch to position mode and back.
        if self.servo_mode == 'Current Based Position Control':
            self.serial_manager.send_message('set_servo_mode', [3])
            self.serial_manager.send_message('set_servo_mode', [5])

    def update_pid_gains(self):
        self.Kp = float(self.Kp_textbox.text())
        self.Ki = float(self.Ki_textbox.text())
        self.Kd = float(self.Kd_textbox.text())

    def on_toggle_torque(self, state):
        if state == Qt.Checked:
            self.serial_manager.send_message('toggle_torque', [1])
        else:
            self.serial_manager.send_message('toggle_torque', [0])

    def update_plot(self):

        # Update joystick plot
        if len(self.present_positions) > 0:  # Check if we have any data to plot
            last_pos_norm = self.encoder_to_norm(self.present_positions[-1])  # Get the most recent position
            self.plot.clear()

            # Add the ImageItem to the PlotWidget
            if self.joystick_mode == 'Walls':
                self.plot.addItem(self.img_item)
            self.plot.plot([last_pos_norm[0]], [last_pos_norm[1]], pen=None,
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

    def mouse_control(self, pos):
        encoder = self.norm_to_encoder([pos[0], pos[1]])
        self.user_mouse_pos = [int(encoder[0]), int(encoder[1])]

    def find_nearest_boundary_point(self, img_x, img_y):
        # Initialize variables to hold the nearest point
        nearest_x, nearest_y = img_x, img_y
        min_distance = float('inf')  # Initialize to a large value

        # Define a search radius (this can be adjusted)
        search_radius = 100

        # Search along each axis
        for d in range(1, search_radius + 1):
            for dx, dy in [(d, 0), (-d, 0), (0, d), (0, -d)]:  # Right, Left, Up, Down
                # Calculate the coordinates of the pixel to check
                check_x, check_y = img_x + dx, img_y + dy

                # Make sure the coordinates are within the image boundaries
                if 0 <= check_x < self.boundary_image.shape[1] and 0 <= check_y < self.boundary_image.shape[0]:
                    # Check if the pixel is white
                    if self.boundary_image[check_y, check_x] == 255:
                        distance = abs(dx) + abs(dy)  # Manhattan distance
                        if distance < min_distance:
                            min_distance = distance
                            nearest_x, nearest_y = check_x, check_y

        return nearest_x, nearest_y  # Return the nearest point found

    def calculate_standard_position(self):
        goal_pos = None
        match self.joystick_mode:
            case 'Center':
                goal_pos = self.user_mouse_pos if self.user_is_dragging else self.center_pos
            case 'Follow':
                goal_pos = self.user_mouse_pos if self.user_is_dragging else self.present_positions[-1]
            case 'Walls':
                # Proposed next position
                proposed_goal_pos = self.user_mouse_pos if self.user_is_dragging else self.present_positions[-1]
                norm_proposed_goal_pos = self.encoder_to_norm(proposed_goal_pos)

                # Convert the normalized proposed position to image coordinates
                img_x, img_y = int(norm_proposed_goal_pos[0] * self.boundary_image.shape[1]), int(
                    norm_proposed_goal_pos[1] * self.boundary_image.shape[0])

                # Check if the proposed position is within the white region of the boundary image
                if self.boundary_image[img_y, img_x] == 255:
                    self.in_bounds = True
                    goal_pos = proposed_goal_pos
                else:
                    self.in_bounds = False
                    # Find the nearest point within the boundary
                    nearest_x, nearest_y = self.find_nearest_boundary_point(img_x, img_y)

                    # Convert the image coordinates back to encoder coordinates
                    goal_pos = self.norm_to_encoder([nearest_x / self.boundary_image.shape[1], nearest_y / self.boundary_image.shape[0]])

        if goal_pos is not None:
            self.goal_positions.append(goal_pos)
        return goal_pos

    def calculate_pid_currents(self):
        if len(self.goal_positions) == 0 or len(self.present_positions) == 0:
            return [0, 0]  # Return zero currents if there's no data

        # Determine the reference position based on joystick mode and user dragging
        reference_pos = None
        match self.joystick_mode:
            case 'Center':
                reference_pos = self.user_mouse_pos if self.user_is_dragging else self.center_pos
            case 'Follow':
                reference_pos = self.user_mouse_pos if self.user_is_dragging else self.present_positions[-1]
            case 'Walls':
                pass  # Implement your logic for 'Walls' mode

        if reference_pos is None:
            return [0, 0]  # Return zero currents if there's no reference position

        # Calculate the error for each servo
        error = np.array(reference_pos) - np.array(self.present_positions[-1])

        # Calculate the integral of the error for each servo
        self.integral += error

        # Calculate the derivative of the error for each servo
        derivative = error - np.array(self.prev_errors[-1])

        # Calculate PID output for each servo
        output = self.Kp * np.array(error, dtype=float) + self.Ki * np.array(self.integral, dtype=float) + \
                 self.Kd * np.array(derivative, dtype=float)

        # Clip the output to be within the range [-100, 100]
        output = np.clip(output, -100, 100)

        # Update the previous errors deque
        self.prev_errors.append(error)

        return output.tolist()

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
        servo_mode_before_test = self.servo_mode
        self.serial_manager.send_message('set_servo_mode', [3])
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
        self.servo_mode = servo_mode_before_test
        self.serial_manager.send_message('set_servo_mode', self.servo_modes_map[servo_mode_before_test])
        self.is_test_sequence_running = False

    def norm_to_encoder(self, norm_coord):
        """
        Convert a normalized position to an encoder position.
        :param norm_coord: Normalized position in the range [0, 1] for both axes
        :return: Encoder position in the range self.center_pos +- 600 for both axes
        """
        encoder_coord = [
            int(norm * self.range * 2 - self.range + center)
            for norm, center in zip(norm_coord, self.center_pos)
        ]
        return encoder_coord

    def encoder_to_norm(self, encoder_coord):
        """
        Convert an encoder position to a normalized position.
        :param encoder_coord: Encoder position in the range self.center_pos +- 600 for both axes
        :return: Normalized position in the range [0, 1] for both axes
        """
        norm_coord = [
            (encoder - center + self.range) / (self.range * 2)
            for encoder, center in zip(encoder_coord, self.center_pos)
        ]
        return norm_coord


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
            try:
                positions = self.read_servo_positions()
            except ValueError:
                continue
            if positions:
                self.new_position_signal.emit(positions)

            if not self.joylab.is_test_sequence_running:

                # If in walls mode, toggle the torque when the joystick passes a boundary
                if self.joylab.joystick_mode == 'Walls':
                    if self.joylab.in_bounds != self.joylab.prev_in_bounds:
                        print("Change detected: ", self.joylab.prev_in_bounds, " -> ", self.joylab.in_bounds, "")
                        # Update the previous state
                        self.joylab.prev_in_bounds = self.joylab.in_bounds

                        # Toggle the torque based on the new state
                        if self.joylab.in_bounds:
                            self.send_message('toggle_torque', [0])  # Torque off
                        else:
                            self.send_message('toggle_torque', [1])  # Torque on

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

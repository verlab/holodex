import sys 
import zenoh
import time
import struct
from PySide6.QtCore import Slot, Qt, QObject, Signal, QThread
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QSlider, QVBoxLayout, QWidget, QLabel

from joystick_widget import JoystickWidget
from zenoh_widget import ZenohWidget
from vector_input import Vetor2DWidget
from cinematic import Cinematics


MAX_SPEED = 25000

class MainWindow(QMainWindow):    
    def __init__(self):
        super().__init__()

        # Configure the main window properties
        self.setWindowTitle("Robot interface")
        self.resize(500, 300)


        # Sliders
        self.slider = []
        for i in range(4):
            self.slider.append(QSlider(Qt.Orientation.Horizontal))
            self.slider[-1].setMinimum(-MAX_SPEED)
            self.slider[-1].setMaximum(MAX_SPEED)
            self.slider[-1].setValue(0)

            # Slider to function
            self.slider[-1].valueChanged.connect(
                    lambda val, id=i: self.update_slider(id+1, val)
                    )

        

        # Adding elements
        layout = QVBoxLayout()


        # Zenoh widget
        self.zenoh_widget = ZenohWidget()
        layout.addWidget(self.zenoh_widget)

        # Joystick widget
        self.joy_widget = JoystickWidget()
        self.joy_widget.joy_data.connect(self.on_joy_event)
        layout.addWidget(self.joy_widget)

        # Vector system
        self.vector_input = Vetor2DWidget()
        self.vector_input.vector.connect(self.vector_onChange)
        layout.addWidget(self.vector_input)

        # Reset button
        self.reset_button = QPushButton("Zerar motores")
        self.reset_button.clicked.connect(self.reset_motors)
        layout.addWidget(self.reset_button)


        # Sliders
        for slider, i in zip(self.slider, range(4)):
            layout.addWidget(QLabel(f"Motor {i}"))
            layout.addWidget(slider)

        # Assign layout contents
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Cinematic calculator
        self.cinCalc = Cinematics()
        self.cinCalc.addMotor(-1,-1, 23.5,-14.7)
        self.cinCalc.addMotor(-1, 1,-23.5,-14.7)
        self.cinCalc.addMotor( 1,-1, 23.5, 14.7)
        self.cinCalc.addMotor( 1, 1,-23.5, 14.7)
        
        self.control_rotation = 0.0
        
        self.slider_block = False
    
    
    def closeEvent(self, event):
        print("FECHADNOOO")
        self.joy_widget.close()
        self.zenoh_widget.close()

    def reset_motors(self):
        self.vector_input.set_value(0,0)


    def update_slider(self, slider_id, value):
        print(f"Novo valor para motor {slider_id}: {value}")
        if not self.slider_block:
            self.zenoh_widget.send_values(
                self.slider[0].value(),
                self.slider[1].value(),
                self.slider[2].value(),
                self.slider[3].value()
            )
    
    def on_joy_event(self, x, y, rot):
        print(x, y, rot)
        self.control_rotation = rot
        self.vector_input.set_value(x, -y)

    # Vector system callback function
    def vector_onChange(self, x, y):
        vec = [i*MAX_SPEED for i in self.cinCalc.getDirection(y, x, self.control_rotation)]
        print(*vec)
        self.slider_block = True
        for i in zip(self.slider, vec):
            i[0].setValue(i[1])
        self.slider_block = False
        self.zenoh_widget.send_values(
            *vec
        )

    @Slot()
    def on_connect_button(self):
        """Slot handler triggered when the user clicks the button."""
        print("Connecting")
        if(self.zenoh_thread):
            # Signal the loop inside the worker to stop
            self.zenoh_thread.requestInterruption() 
            
            # Tell the Qt event loop to quit
            self.zenoh_thread.quit()                
            
            # CRITICAL: Wait for the OS to completely close the thread
            self.zenoh_thread.wait()
        self.setup_zenoh_thread()

if __name__ == "__main__":
    # Initialize application and its underlying event loop execution
    app = QApplication(sys.argv)
    
    window = MainWindow()
    window.show() # Windows remain hidden by default until explicitly called
    
    sys.exit(app.exec())


import pygame
import time
from zenoh_worker import ZenohWorker
from PySide6.QtCore import Slot, Qt, QObject, Signal, QThread
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton


pygame.init()
pygame.joystick.init()

class JoystickWidget(QWidget):
    joy_data = Signal(float, float, float)

    def __init__(self):
        super().__init__()

        # Creating layout
        layout_horizontal = QHBoxLayout()

        self.status_label = QLabel("Controller disconnected")
        self.connect_button = QPushButton("Connect controller")
        self.connect_button.clicked.connect(self._on_connect_button)

        layout_horizontal.addWidget(self.status_label)
        layout_horizontal.addWidget(self.connect_button)

        self.setLayout(layout_horizontal)
        
        # Start zenoh attributes
        self.joy_thread = None
        self.worker = None

    def close(self):
        print("Closing joy")
        if self.worker:
            self.worker.running = False
        
        if self.joy_thread:
            # Stops and waits
            self.joy_thread.requestInterruption() 
            self.joy_thread.quit()
            self.joy_thread.wait()

    def get_signal(self) -> Signal:
        return self.worker.axis

    def _setup_zenoh_thread(self):
        # Create persistent thread and worker
        self.joy_thread = QThread()
        self.worker = JoyWorker()

        # Move the worker object to run inside the background thread
        self.worker.moveToThread(self.joy_thread)

        # Connect thread lifecycle signals
        self.joy_thread.started.connect(self.worker.start)
        
        # Connecting cleanup
        self.joy_thread.finished.connect(self.worker.cleanup)
        
        # Connect Worker signals back to UI slots
        self.worker.connection_status.connect(self.status_label.setText)
        
        # Start the background thread execution
        self.joy_thread.start()

        self.worker.axis.connect(self._on_axis_change)


    def _on_axis_change(self, v1, v2, v3):
        self.joy_data.emit(v1, v2, v3)


    @Slot()
    def _on_connect_button(self):
        """Slot handler triggered when the user clicks the button."""
        print("Connecting")
        if(self.joy_thread):
            # Signal the loop inside the worker to stop
            self.joy_thread.requestInterruption() 
            
            # Tell the Qt event loop to quit
            self.joy_thread.quit()                
            
            # CRITICAL: Wait for the OS to completely close the thread
            self.joy_thread.wait()
        self._setup_zenoh_thread()


class JoyWorker(QObject):
    axis = Signal(float, float, float)
    connection_status = Signal(str)

    def __init__(self):
        super().__init__()
        self.running = False
    
    @Slot()
    def start(self):
        try:
            joy = pygame.joystick.Joystick(0)
            joy.init()
            self.connection_status.emit(f"Connected: {joy.get_name()}")
            self.running = True

            a1 = 0.0
            a2 = 0.0
            a3 = 0.0

            while self.running:
                self.axis.emit(a1, a2, a3)
                for evento in pygame.event.get():
                    if evento.type == pygame.QUIT:
                        self.running = False
                    
                    # Detecta botões sendo pressionados
                    if evento.type == pygame.JOYBUTTONDOWN:
                        print(f"Botão {evento.button} pressionado.")

                    # Detecta movimentação dos eixos (Analógicos)
                    if evento.type == pygame.JOYAXISMOTION:
                        # 0 -> X on left
                        # 1 -> Y on left
                        # 2 -> X on right
                        if evento.axis == 0:
                            a1 = evento.value
                        if evento.axis == 1:
                            a2 = evento.value
                        if evento.axis == 2:
                            a3 = evento.value
                time.sleep(0.01)

        except:
            self.running = False
            self.connection_status.emit("Controller disconnected due to error")
        finally:
            self.running = False
            self.connection_status.emit("Controller disconnected")
    
    
    def cleanup(self):
        self.connection_status.emit("Controller disconnected")
        self.running = False

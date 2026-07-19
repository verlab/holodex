from zenoh_worker import ZenohWorker
from PySide6.QtCore import Slot, Qt, QObject, Signal, QThread
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton


class ZenohWidget(QWidget):
    # QT Signal
    motor_values = Signal(int, int, int, int)

    def __init__(self):
        super().__init__()

        # Creating layout
        layout_horizontal = QHBoxLayout()

        self.status_label = QLabel("Zenoh Disconnected")
        self.connect_button = QPushButton("Connect Zenoh")
        self.connect_button.clicked.connect(self._on_connect_button)

        layout_horizontal.addWidget(self.status_label)
        layout_horizontal.addWidget(self.connect_button)

        self.setLayout(layout_horizontal)
        
        # Start zenoh attributes
        self.zenoh_thread = None
        self.worker = None

    def send_values(self, val1, val2, val3, val4):
        print("tentando enviar valores")
        self.motor_values.emit(val1, val2, val3, val4)

    def close(self):
        print("Closing Zenoh")
        if(self.zenoh_thread):
            # Stops and waits
            self.zenoh_thread.requestInterruption() 
            self.zenoh_thread.quit()
            self.zenoh_thread.wait()


    def _setup_zenoh_thread(self):
        # Create persistent thread and worker
        self.zenoh_thread = QThread()
        self.worker = ZenohWorker()

        # Move the worker object to run inside the background thread
        self.worker.moveToThread(self.zenoh_thread)

        # Connect thread lifecycle signals
        self.zenoh_thread.started.connect(self.worker.start_zenoh)
        
        # Connecting cleanup
        self.zenoh_thread.finished.connect(self.worker.cleanup)
        
        # Connect Worker signals back to UI slots
        self.worker.connection_status.connect(self.status_label.setText)

        # Connect UI custom signal to Worker publishing slot
        self.motor_values.connect(
            self.worker.publish_slider_values, 
            Qt.ConnectionType.QueuedConnection
        )        
        
        # Start the background thread execution
        self.zenoh_thread.start()

    @Slot()
    def _on_connect_button(self):
        """Slot handler triggered when the user clicks the button."""
        print("Connecting")
        if(self.zenoh_thread):
            # Signal the loop inside the worker to stop
            self.zenoh_thread.requestInterruption() 
            
            # Tell the Qt event loop to quit
            self.zenoh_thread.quit()                
            
            # CRITICAL: Wait for the OS to completely close the thread
            self.zenoh_thread.wait()
        self._setup_zenoh_thread()
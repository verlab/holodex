import zenoh
import time
import struct
from PySide6.QtCore import Slot, Qt, QObject, Signal, QThread

class ZenohWorker(QObject):
    # QT signals
    data_received = Signal(str)
    connection_status = Signal(str)
    
    def __init__(self):
        super().__init__()
        self.session = None
        self.running = False

    
    @Slot()
    def start_zenoh(self):
        """Initializes Zenoh and starts the persistent listening loop."""
        try:
            print("Connecting to Zenoh...")
            self.connection_status.emit("Connecting to Zenoh...")

            conf = zenoh.Config.from_json5(
                '{"mode": "client", "connect": {"endpoints": ["tcp/127.0.0.1:7447"]}}'
            )
            self.session = zenoh.open(conf)
            
            print("Zenoh connected!")
            self.connection_status.emit("Zenoh Connected!")
            
            # Set state as running
            self.running = True

            

            self.subscriber = self.session.declare_subscriber("demo/rt/data", self.on_data)

            
        except Exception as e:
            self.connection_status.emit(f"Zenoh Error: {str(e)}")

    def on_data(self, sample):
        """Message handling callback."""
        payload = sample.payload.decode('utf-8')
        
        # Forward data for main window
        self.data_received.emit(payload)

    @Slot(int, int, int, int)
    def publish_slider_values(self, val1, val2, val3, val4):
        """Sends 4 signed 32 bit integers to Zenoh"""
        print("Trying to publish")
        if self.session:
            key_expr = "demo/pc_to_esp"

            # 4 int32_t values
            payload = struct.pack("<4i", val1, val2, val3, val4)

            print(f"Publishing binary struct: {val1}, {val2}, {val3}, {val4}")
            self.session.put(key_expr, payload)

    def cleanup(self):
        """Closes resources when application terminates."""
        print("Disconnecting zenoh. Doing cleanup")
        if self.session:
            self.session.close()
            self.connection_status.emit("Zenoh Disconnected.")
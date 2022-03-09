import serial
import json


class Driver:

    def __init__(self, port, update_cb):
        self._port = port
        self._update_cb = update_cb

    def set_joints(self, angles):
        tx = json.dumps({
            "uuid": ""
            
        })

    def get_joints(self):
        pass

    def push_update(self):
        pass

    def get_metadata(self):
        pass

    def loop(self):
        pass
import socketio
import threading
import queue
import time
import json
import uuid
import numpy as np

class NetworkTransmitter:
    """
    Handles network communication for the AMI Ogma agent.
    Implements the "Split-Phase" protocol:
    1. Graph Sync (Heavy)
    2. Reality Stream (Lightweight)
    """
    def __init__(self, server_url="http://localhost:5000", agent_id="ami-ogma-1", modality="AUDIO"):
        self.server_url = server_url
        self.agent_id = agent_id
        self.modality = modality
        
        # Async-safe Queue for outgoing messages
        self.queue = queue.Queue(maxsize=1000)
        self.running = False
        self.enabled = True # v35: Runtime toggle
        self.app_state = "PAUSED" # v35: RUNNING, PAUSED, BOOTSTRAPPING, ERROR
        self.throttle_ms = 40 # v35: Match brain processing interval
        
        # Socket.IO Client
        self.sio = socketio.Client()
        self.thread = None
        
        self.connected = False
        
        # Setup Callbacks
        self.sio.on('connect', self._on_connect)
        self.sio.on('disconnect', self._on_disconnect)
        
    @staticmethod
    def get_local_ip():
        """Returns the local IP address for multi-agent coordination."""
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0)
            # Use a dummy address to get local interface IP
            s.connect(('10.254.254.254', 1))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"
            
    def set_server_url(self, new_url):
        """Update the target server and trigger a reconnect."""
        if new_url != self.server_url:
            print(f"Transmitter: Switching server to {new_url}")
            self.server_url = new_url
            if self.connected:
                try:
                    self.sio.disconnect()
                except: pass
                # _worker_loop will automatically try to reconnect to new_url

    def get_config(self):
        """Returns current persistable configuration."""
        return {
            "server_url": self.server_url,
            "agent_id": self.agent_id,
            "enabled": self.enabled,
            "throttle_ms": self.throttle_ms,
            "modality": self.modality
        }

    def set_config(self, config):
        """Applies a configuration dictionary."""
        if "server_url" in config:
            self.set_server_url(config["server_url"])
        if "agent_id" in config:
            self.agent_id = config["agent_id"]
        if "enabled" in config:
            self.enabled = config["enabled"]
        if "throttle_ms" in config:
            self.throttle_ms = config["throttle_ms"]
        if "modality" in config:
            self.modality = config["modality"]

    def start(self):
        """Start the background worker thread."""
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()
        print(f"Transmitter: Started background thread (Target: {self.server_url})")

    def stop(self):
        """Stop the worker thread."""
        self.running = False
        if self.thread: self.thread.join(timeout=1.0)
        if self.connected: 
            try: self.sio.disconnect()
            except: pass

    def emit_sync(self, nodes_data):
        """
        Emit a GRAPH_SYNC event (Heavy).
        nodes_data: list of dicts {id, label, embedding, ...}
        """
        self._enqueue("GRAPH_SYNC", {
            "nodes": nodes_data
        })

    def emit_reality(self, current_id, trajectory, neurotransmitters, current_dt=0.0, text_label=""):
        """
        Emit a TEMPORAL_CONTEXT event (Lightweight).
        """
        self._enqueue("TEMPORAL_CONTEXT", {
            "current_id": current_id,
            "current_dt": current_dt,
            "neurotransmitters": neurotransmitters,
            "trajectory": trajectory,
            "text_label": text_label
        })

    def emit_status(self, state):
        """
        Emit a STATUS event to update agent mode on the server.
        state: RUNNING, PAUSED, BOOTSTRAPPING, ERROR
        """
        self._enqueue("STATUS", {
            "state": state
        })

    def _enqueue(self, msg_type, payload):
        if not self.running: return
        
        envelope = {
            "token_uuid": str(uuid.uuid4()),
            "type": msg_type,
            "header": {
                "agent_id": self.agent_id,
                "modality": self.modality,
                "timestamp": time.time(),
                "app_state": self.app_state # v35: Current agent state
            },
            "payload": payload
        }
        
        try:
            self.queue.put_nowait(envelope)
        except queue.Full:
            try:
                self.queue.get_nowait() # Drop oldest
                self.queue.put_nowait(envelope)
            except: pass

    def _worker_loop(self):
        last_connect_attempt = 0
        connect_retry = 5.0
        
        while self.running:
            if not self.connected and self.enabled:
                now = time.time()
                if now - last_connect_attempt > connect_retry:
                    try:
                        self.sio.connect(self.server_url, wait_timeout=1)
                    except: pass
                    last_connect_attempt = now
            
            # If we become disabled while connected, disconnect
            if not self.enabled and self.connected:
                try: self.sio.disconnect()
                except: pass
            
            if self.connected:
                try:
                    # Batch processing (Burst Limited to 20 to avoid payload overflow)
                    count = 0
                    while not self.queue.empty() and count < 20:
                        msg = self.queue.get_nowait()
                        self.sio.emit('reality_token', msg)
                        self.queue.task_done()
                        count += 1
                except:
                    self.connected = False
            
            time.sleep(self.throttle_ms / 1000.0)

    def _on_connect(self):
        print(f"Transmitter: Connected to {self.server_url}")
        self.connected = True
        self.sio.emit('register', {'agent_id': self.agent_id, 'modality': self.modality})

    def _on_disconnect(self):
        print("Transmitter: Disconnected")
        self.connected = False

import socketio
import threading
import queue
import time

class NetworkReceiver:
    """
    Experimental Receiver for 'Reality Tokens' from other AMI Ogma instances.
    Listens for TEMPORAL_CONTEXT events (via monitor_feed) and queues them for the Brain.
    """
    def __init__(self, server_url="http://localhost:5000", agent_id="ami-ogma-receiver"):
        self.server_url = server_url
        self.agent_id = agent_id
        
        # Async-safe Queue for incoming messages
        self.queue = queue.Queue(maxsize=1000)
        self.running = False
        self.enabled = False # Default to OFF
        
        # Socket.IO Client
        self.sio = socketio.Client()
        self.thread = None
        self.connected = False
        
        # Setup Callbacks
        self.sio.on('connect', self._on_connect)
        self.sio.on('disconnect', self._on_disconnect)
        self.sio.on('monitor_feed', self._handle_token)
        
    def set_server_url(self, new_url):
        if new_url != self.server_url:
            print(f"Receiver: Switching server to {new_url}")
            self.server_url = new_url
            if self.connected:
                try: self.sio.disconnect()
                except: pass

    def get_config(self):
        """Returns current persistable configuration."""
        return {
            "server_url": self.server_url,
            "agent_id": self.agent_id,
            "enabled": self.enabled
        }

    def set_config(self, config):
        """Applies a configuration dictionary."""
        if "server_url" in config:
            self.set_server_url(config["server_url"])
        if "agent_id" in config:
            self.agent_id = config["agent_id"]
        if "enabled" in config:
            self.enabled = config["enabled"]

    def start(self):
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()
        print(f"Receiver: Started background thread (Target: {self.server_url})")

    def stop(self):
        self.running = False
        if self.thread: self.thread.join(timeout=1.0)
        if self.connected:
            try: self.sio.disconnect()
            except: pass

    def _worker_loop(self):
        last_connect_attempt = 0
        connect_retry = 5.0
        
        while self.running:
            if not self.connected and self.enabled:
                now = time.time()
                if now - last_connect_attempt > connect_retry:
                    try:
                        # Wait 1s for connection
                        self.sio.connect(self.server_url, wait_timeout=1)
                    except: pass
                    last_connect_attempt = now
            
            if not self.enabled and self.connected:
                try: self.sio.disconnect()
                except: pass
            
            time.sleep(1.0) # Slow check

    def _on_connect(self):
        print(f"Receiver: Connected to {self.server_url}")
        self.connected = True

    def _on_disconnect(self):
        print("Receiver: Disconnected")
        self.connected = False

    def _handle_token(self, data):
        """Callback for monitor_feed events from the server."""
        if not self.enabled: return
        
        msg_type = data.get('type')
        if msg_type == "TEMPORAL_CONTEXT":
            payload = data.get('payload', {})
            # We want current_id, traj, neurotransmitters, current_dt
            token_data = {
                "current_id": payload.get('current_id'),
                "trajectory": payload.get('trajectory', []),
                "neurotransmitters": payload.get('neurotransmitters', {}),
                "current_dt": payload.get('current_dt', 0.0)
            }
            try:
                self.queue.put_nowait(token_data)
            except queue.Full:
                try:
                    self.queue.get_nowait()
                    self.queue.put_nowait(token_data)
                except: pass

    def read(self):
        """Pull the latest token from the queue."""
        try:
            return self.queue.get_nowait()
        except queue.Empty:
            return None

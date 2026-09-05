"""Control-socket client + ZMQ diag subscriber.

Both sides are deliberately thin and synchronous; the inspector wraps
them in QThreads / QTimers so the main GUI stays responsive.
"""
from __future__ import annotations

import json
import socket
import threading
from dataclasses import dataclass
from typing import Callable, Optional

import zmq


# ---------------------------------------------------------------------------
# Control socket: newline-JSON request/reply over TCP
# ---------------------------------------------------------------------------

class ControlClient:
    """Persistent connection to OgmaBrain's ControlServer.

    Each call() blocks until a reply arrives.  Callers should run this
    on a worker thread when used from a GUI.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 7400, timeout: float = 5.0):
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._timeout = timeout
        self._buf = b""

    # -- lifecycle --

    def connect(self) -> None:
        with self._lock:
            if self._sock is not None:
                return
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self._timeout)
            s.connect((self.host, self.port))
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock = s
            self._buf = b""

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
                self._buf = b""

    def set_endpoint(self, host: str, port: int) -> None:
        """Point at a different brain.  Drops any live socket so the next
        connect() dials the new address; no-op when nothing changed, so the
        caller can call it unconditionally on every Refresh."""
        if host == self.host and port == self.port:
            return
        self.close()
        with self._lock:
            self.host = host
            self.port = port

    def reconnect(self) -> None:
        """Force-close any existing socket and open a fresh one.

        Used by the inspector's Refresh button to recover from a
        Godot relaunch: the stale TCP socket would otherwise be
        held in self._sock and the next call() would fail.  Calling
        reconnect() drops the old socket and reopens to whatever's
        currently listening on (host, port).
        """
        self.close()
        self.connect()

    # -- rpc --

    def call(self, verb: str, **kwargs) -> dict:
        if self._sock is None:
            self.connect()
        msg = json.dumps({"verb": verb, **kwargs}) + "\n"
        with self._lock:
            assert self._sock is not None
            self._sock.sendall(msg.encode())
            while b"\n" not in self._buf:
                chunk = self._sock.recv(4096)
                if not chunk:
                    raise ConnectionError("control socket closed by peer")
                self._buf += chunk
            line, _, rest = self._buf.partition(b"\n")
            self._buf = rest
        return json.loads(line.decode())


# ---------------------------------------------------------------------------
# Diag stream: ZMQ SUB
# ---------------------------------------------------------------------------

@dataclass
class DiagPayload:
    sub_id: int
    module_id: str
    topic: str
    tick_id: int
    snapshot: dict


class DiagSubscriber:
    """ZMQ SUB wrapper — one socket, multiple subscription prefixes.

    Caller adds topic prefixes (typically returned by `module_subscribe_diag`)
    via add_prefix; the receive loop runs on a daemon thread and invokes
    the on_payload callback for each two-frame message.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 7401):
        self.host = host
        self.port = port
        self._ctx: Optional[zmq.Context] = None
        self._sock: Optional[zmq.Socket] = None
        self._on_payload: Optional[Callable[[DiagPayload], None]] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self, on_payload: Callable[[DiagPayload], None]) -> None:
        if self._thread is not None:
            return
        self._on_payload = on_payload
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.RCVHWM, 64)
        self._sock.setsockopt(zmq.RCVTIMEO, 200)  # ms; allows clean stop
        self._sock.connect(f"tcp://{self.host}:{self.port}")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="DiagSubscriber",
                                         daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def set_endpoint(self, host: str, port: int) -> None:
        """Point the live SUB socket at a different brain.

        ZMQ disconnect/connect on the SAME socket, deliberately: subscriptions
        are a property of the socket rather than of the connection, so every
        active topic prefix survives the move and the receive thread never
        stops.  Tearing the subscriber down and rebuilding it would drop them
        and leave the UI subscribed to a module it no longer receives.
        """
        if host == self.host and port == self.port:
            return
        old = f"tcp://{self.host}:{self.port}"
        self.host = host
        self.port = port
        if self._sock is not None:
            try:
                self._sock.disconnect(old)
            except zmq.ZMQError:
                pass                      # never connected, or already gone
            self._sock.connect(f"tcp://{host}:{port}")

    def add_prefix(self, prefix: str) -> None:
        if self._sock is not None:
            self._sock.setsockopt_string(zmq.SUBSCRIBE, prefix)

    def remove_prefix(self, prefix: str) -> None:
        if self._sock is not None:
            self._sock.setsockopt_string(zmq.UNSUBSCRIBE, prefix)

    def _run(self) -> None:
        # Two-frame messages: topic, then JSON body.
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                topic = self._sock.recv_string()
                body = self._sock.recv_string()
            except zmq.Again:
                continue
            except zmq.ZMQError:
                if self._stop.is_set():
                    break
                continue
            try:
                payload = json.loads(body)
                p = DiagPayload(
                    sub_id=int(payload.get("sub_id", -1)),
                    module_id=str(payload.get("module_id", "")),
                    topic=str(payload.get("topic", "")),
                    tick_id=int(payload.get("tick_id", 0)),
                    snapshot=payload.get("snapshot") or {},
                )
            except (json.JSONDecodeError, ValueError, TypeError):
                continue
            if self._on_payload is not None:
                self._on_payload(p)

"""
zmq_bus.py — ZeroMQ subscriber utilities for the AMI-Ogma multi-EPM bus.

Each C++ ami-ogma-v3 process publishes RealityToken JSON on a ZMQ PUB socket
(port = control_port + 1) after every tick.  Message format:

    "<modality> <json_payload>"

RealityTokenSubscriber connects a single ZMQ SUB socket to one or more PUB
endpoints, drains messages in a background thread, and exposes the latest
token per modality via get_latest() / get_all_latest().
"""

import json
import threading
import time
from typing import Dict, Optional

import zmq


class RealityTokenSubscriber:
    """
    Non-blocking ZMQ SUB that aggregates RealityToken JSON from multiple
    C++ EPM PUB sockets.

    Usage:
        sub = RealityTokenSubscriber()
        sub.connect("tcp://127.0.0.1:7201", topic="retinal")
        sub.connect("tcp://127.0.0.1:7203", topic="cochlear")
        sub.start()
        ...
        token = sub.get_latest("retinal")  # None if no messages yet
        all_tokens = sub.get_all_latest()  # {modality: token_dict}
        sub.stop()
    """

    def __init__(self, poll_timeout_ms: int = 5):
        self._ctx  = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.RCVHWM, 64)
        # RCVTIMEO not used — we use NOBLOCK recv + Event.wait() to avoid
        # zmq.Poller / select() which overflows FD_SETSIZE when Qt is running.
        self._sock.setsockopt(zmq.RCVTIMEO, -1)  # blocking disabled via NOBLOCK flag

        self._lock:   threading.Lock             = threading.Lock()
        self._latest: Dict[str, dict]            = {}
        self._stop:   threading.Event            = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sleep_s = poll_timeout_ms / 1000.0  # seconds to wait when no message

    def connect(self, endpoint: str, topic: str = "") -> None:
        """Connect to a PUB socket and optionally filter by topic prefix."""
        self._sock.connect(endpoint)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, topic)

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._drain, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            self._sock.close(linger=0)
        except Exception:
            pass

    def get_latest(self, modality: str) -> Optional[dict]:
        with self._lock:
            return self._latest.get(modality)

    def get_all_latest(self) -> Dict[str, dict]:
        with self._lock:
            return dict(self._latest)

    def _drain(self) -> None:
        """
        Drain loop using NOBLOCK recv + Event.wait() instead of zmq.Poller.
        Avoids zmq.Poller.poll() which internally calls select() and overflows
        FD_SETSIZE (1024) when running alongside Qt/X11.
        """
        while not self._stop.is_set():
            try:
                raw = self._sock.recv_string(zmq.NOBLOCK)
                topic, _, payload = raw.partition(" ")
                if payload:
                    token = json.loads(payload)
                    with self._lock:
                        self._latest[topic] = token
            except zmq.Again:
                # No message available — sleep briefly, waking immediately on stop
                self._stop.wait(self._sleep_s)
            except zmq.ZMQError:
                break
            except Exception:
                pass

"""Talking to the xaq_voice engine.

Two sockets, mirroring the brain's own inspector interface: request/reply for commands,
publish/subscribe for the stream of live numbers.

The subscriber half is ``tools/xaq_inspector/transport.py``'s ``DiagSubscriber``, used
unmodified — the engine wraps its meter frame in the same ``{topic, tick_id, snapshot}``
envelope precisely so that class fits.  The SUB thread, the 200 ms receive timeout that
makes ``stop()`` clean and the bounded join are the fiddly half, and they are already
written and in service.

The command half cannot reuse ``ControlClient``: that speaks newline-JSON over a raw TCP
socket to the brain, while the engine speaks ZMQ REQ/REP.  What is here instead is a
lazy-pirate REQ — a REQ socket is strictly alternating, so a single lost reply wedges it
forever, and the only cure is to discard the socket and open a new one.

**The studio never connects to the brain.**  The engine is already subscribed to every
module, so a second subscriber would double the sim's per-tick serialisation cost for
nothing, and a leaked subscription costs the sim on every tick for the life of the
process.  Routing everything through the engine makes that leak structurally impossible
from here.
"""
from __future__ import annotations

import json
import threading
from typing import Any, Callable

import zmq

from xaq_inspector.transport import DiagPayload, DiagSubscriber   # noqa: F401  (re-exported)


class EngineError(RuntimeError):
    pass


class EngineClient:
    """Synchronous REQ client with recovery.

    Call from a worker thread, not the GUI thread: every call blocks for up to
    ``timeout_ms``, and a dead engine would otherwise freeze the window.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 7460, timeout_ms: int = 1500):
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self._ctx = zmq.Context.instance()
        self._sock: zmq.Socket | None = None
        self._lock = threading.Lock()

    # -- lifecycle --

    def _open(self) -> zmq.Socket:
        s = self._ctx.socket(zmq.REQ)
        s.setsockopt(zmq.LINGER, 0)
        s.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        s.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        s.connect(f"tcp://{self.host}:{self.port}")
        return s

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                self._sock.close(0)
                self._sock = None

    def reconnect(self) -> None:
        self.close()

    # -- rpc --

    def call(self, verb: str, **kwargs: Any) -> dict:
        """One round trip.  Raises EngineError on timeout or an error reply."""
        with self._lock:
            if self._sock is None:
                self._sock = self._open()
            try:
                self._sock.send_string(json.dumps({"verb": verb, **kwargs}))
                reply = json.loads(self._sock.recv_string())
            except (zmq.Again, zmq.ZMQError, json.JSONDecodeError) as exc:
                # A REQ that did not get its reply is unusable from here on; the socket
                # has to go, or every later call inherits the failure.
                self._sock.close(0)
                self._sock = None
                raise EngineError(f"{verb}: {type(exc).__name__}") from exc
        if reply.get("status") != "ok":
            raise EngineError(reply.get("message", f"{verb} failed"))
        return reply

    def try_call(self, verb: str, **kwargs: Any) -> dict | None:
        try:
            return self.call(verb, **kwargs)
        except EngineError:
            return None

    # -- the verbs, named --

    def hello(self) -> dict:            return self.call("hello")
    def get_patch(self) -> dict:        return self.call("get_patch")["patch"]
    def set_patch(self, p: dict) -> None: self.call("set_patch", patch=p)
    def get_sources(self) -> list:      return self.call("get_sources")["modules"]
    def auto_patch(self, vary: bool = False) -> dict:
        return self.call("auto_patch", vary=vary)["patch"]
    def save(self, path: str) -> str:   return self.call("save", path=path)["path"]
    def load(self, path: str) -> dict:  return self.call("load", path=path)["patch"]

    def apply_ops(self, ops: list[dict]) -> None:
        if ops:
            self.call("patch", ops=ops)


class StateStream:
    """The engine's meter stream, bridged onto a callback.

    Thin wrapper so the studio never has to know that the frame is a diag envelope.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 7461):
        self._sub = DiagSubscriber(host=host, port=port)
        self._started = False

    def start(self, on_state: Callable[[dict], None]) -> None:
        if self._started:
            return
        self._sub.start(lambda p: on_state(p.snapshot))
        self._sub.add_prefix("state")
        self._started = True

    def stop(self) -> None:
        if self._started:
            self._sub.stop()
            self._started = False

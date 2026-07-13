import asyncio
import socketio
from aiohttp import web
import logging
import time
import json
import socket
import os
from .lateral_voter import LateralVoterNode
from .benchmarks.breakout_harness import BreakoutHarness
from .global_workspace import GlobalWorkspaceHarness
from .adapters.breakout_adapter import BreakoutAdapter
from .proprioceptive_channel import ProprioceptiveChannel

class AnalyticsLogger:
    """Token-efficient structured logger for training sessions."""
    def __init__(self, base_dir="exports/sessions"):
        self.base_dir = base_dir
        self.session_id = time.strftime("%Y%m%d-%H%M%S")
        self.session_path = os.path.join(self.base_dir, self.session_id)
        self.log_file = os.path.join(self.session_path, "events.jsonl")
        self.enabled = False
        self.tick = 0
        self.sample_rate = 10 # Sample brain metrics every 10 ticks (~2.5Hz)

    def start_session(self):
        if not os.path.exists(self.session_path):
            os.makedirs(self.session_path, exist_ok=True)
        self.enabled = True
        self.log_event("session_start", {"id": self.session_id, "time": time.ctime()})
        print(f"Analytics: Session started at {self.log_file}")

    def log_event(self, event_type, data):
        if not self.enabled:
            return
        
        entry = {
            "t": round(time.time(), 3),
            "type": event_type,
            "data": data
        }
        with open(self.log_file, "a") as f:
            f.write(json.dumps(entry) + "\n")

    def log_brain_metrics(self, workspace):
        """Sampled logging of high-level cognitive state."""
        if not self.enabled:
            return
        
        self.tick += 1
        if self.tick % self.sample_rate == 0:
            metrics = {
                "tle": round(workspace.meta_epm.last_tle, 5),
                "dopamine": round(workspace.meta_epm.dopamine, 3),
                "serotonin": round(workspace.meta_epm.serotonin, 3),
                "resonance": round(workspace.voter.last_consensus.resonance_score, 3) if workspace.voter.last_consensus else 0,
                "nodes": workspace.meta_epm.memory.node_count
            }
            self.log_event("brain_metrics", metrics)


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Suppress engineio payload error spam during burst connections
engineio_logger = logging.getLogger('engineio.server')
engineio_logger.setLevel(logging.CRITICAL)

class BrainSocketServer:
    def __init__(self, voter_node: LateralVoterNode):
        self.sio = socketio.AsyncServer(async_mode='aiohttp', cors_allowed_origins='*')
        self.app = web.Application()
        self.sio.attach(self.app)
        self.app.router.add_post('/api/active_inference', self.handle_api_active_inference)
        self.voter = voter_node
        self.runner = None
        self.site = None
        # Config
        self.timeout_enabled = True
        self.timeout_seconds = 5.0
        
        # Analytics
        self.analytics = AnalyticsLogger()
        
        # State tracking
        self.agent_last_active = {}
        self.agent_sids = {} # mapping sid -> agent_id
        
        # Modular Harness Management
        self.legacy_harness = BreakoutHarness(self.voter, self)
        self.workspace_harness = GlobalWorkspaceHarness(self.voter, self, loop=asyncio.get_event_loop())
        self.active_harness = self.workspace_harness  # Default to True Active Inference

        # Proprioceptive channel — bridges body telemetry to the cognitive loop
        _adapter = BreakoutAdapter()
        self._proprioceptive_channel = ProprioceptiveChannel(_adapter, self.voter)
        self.workspace_harness.set_proprioceptive_channel(self._proprioceptive_channel)
        
        # Timeout task handle
        self.timeout_task = None

        # UI decoupling: voter loop writes here; UI timer reads and clears at its own rate
        self._pending_ui_data = None

        # UDP monitor_feed throttle: accumulate per-agent tokens, emit at 10Hz
        self._pending_monitor_tokens = {}
        self._last_monitor_emit = 0.0
        
        # Determine Register Handlers
        self.sio.on('connect', self.on_connect)
        self.sio.on('disconnect', self.on_disconnect)
        self.sio.on('register', self.on_register)
        self.sio.on('reality_token', self.on_reality_token)
        # Benchmark Action Relays
        self.sio.on('request_action_p1', self.on_request_action_p1)
        self.sio.on('request_action_p2', self.on_request_action_p2)
        self.sio.on('raw_game_state', self.on_raw_game_state)
        self.sio.on('set_active_inference', self.on_set_active_inference)

    async def start_server(self, host='0.0.0.0', port=5000):
        self.runner = web.AppRunner(self.app, access_log=None) # Silence aiohttp access logs
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, host, port)
        await self.site.start()
        logger.info(f"Brain Socket Server running on {host}:{port}")
        
        # Start timeout loop
        self.timeout_task = asyncio.create_task(self._timeout_loop())
        # Start UDP listener for C++ EPMs
        self.udp_task = asyncio.create_task(self._udp_listener())
        # Start Voter/Consensus loop for Active Inference
        self.voter_task = asyncio.create_task(self._voter_loop())

    async def stop_server(self):
        if self.timeout_task:
            self.timeout_task.cancel()
        if self.udp_task:
            self.udp_task.cancel()
        if self.voter_task:
            self.voter_task.cancel()
        if self.site:
            await self.site.stop()
        if self.runner:
            await self.runner.cleanup()
        logger.info("Brain Socket Server stopped.")
        
    def set_timeout_enabled(self, enabled: bool):
        self.timeout_enabled = enabled
        
    def set_timeout_seconds(self, seconds: float):
        self.timeout_seconds = seconds
        
    async def _timeout_loop(self):
        while True:
            await asyncio.sleep(1.0)
            if not self.timeout_enabled:
                continue
                
            now = time.time()
            disconnected_agents = []
            
            # Find timed out agents
            for agent_id, last_time in list(self.agent_last_active.items()):
                if now - last_time > self.timeout_seconds:
                    logger.warning(f"Agent Timeout: {agent_id} (> {self.timeout_seconds}s inactivity)")
                    disconnected_agents.append(agent_id)
            
            # Cleanup and disconnect
            for aid in disconnected_agents:
                del self.agent_last_active[aid]
                
                # Find SID, trigger disconnect
                sid_to_remove = None
                for sid, mapped_aid in self.agent_sids.items():
                    if mapped_aid == aid:
                        sid_to_remove = sid
                        break
                        
                if sid_to_remove:
                    await self.sio.disconnect(sid_to_remove)
                    del self.agent_sids[sid_to_remove]
                    
                # Signal voter to clear buffers
                if aid in self.voter.input_buffers:
                    del self.voter.input_buffers[aid]

    async def on_connect(self, sid, environ):
        pass # Silent connect

    async def on_disconnect(self, sid):
        agent_id = self.agent_sids.pop(sid, 'unknown')
        logger.info(f"Agent Disconnected: {agent_id} (SID: {sid})")
        if agent_id in self.agent_last_active:
            del self.agent_last_active[agent_id]
        if agent_id in self.voter.input_buffers:
             del self.voter.input_buffers[agent_id]

    async def on_register(self, sid, data):
        agent_id = data.get('agent_id', 'unknown')
        logger.info(f"Agent Registered: {agent_id} (SID: {sid})")
        self.agent_sids[sid] = agent_id
        self.agent_last_active[agent_id] = time.time()
        # Start the session on first EPM registration so training-phase events
        # are captured. Subsequent registrations (reconnects) are no-ops.
        if not self.analytics.enabled:
            self.analytics.start_session()
            logger.info(f"Analytics: Session started on first EPM registration ({agent_id})")

    async def on_reality_token(self, sid, data):
        # Update last active time if registered
        agent_id = self.agent_sids.get(sid)
        if agent_id:
             self.agent_last_active[agent_id] = time.time()
             
        # Pass to Lateral Voter
        self.voter.ingest_token(data)
        
        # Broadcast to all UI clients for real-time monitoring
        await self.sio.emit('monitor_feed', data)

    async def on_request_action_p1(self, sid, data):
        """Relay action from Workspace to Game Client (P1)"""
        await self.sio.emit('p1_action', data)

    async def on_request_action_p2(self, sid, data):
        """Relay action from Workspace to Game Client (P2)"""
        await self.sio.emit('p2_action', data)

    async def on_raw_game_state(self, sid, data):
        """Handle raw telemetry from a 'dumb' environment"""
        self.agent_last_active['breakout_game'] = time.time()
        
        # Log high-level game events (hits/misses) if present
        event = data.get('event')
        if event in ['self_paddle_hit', 'paddle_miss']:
            self.analytics.log_event("game_event", data)

        if self.active_harness:
            self.active_harness.on_raw_state(data)

    async def broadcast_action(self, player, velocity):
        """Direct broadcast from server-side workspace logic"""
        event = f'p{player}_action'
        await self.sio.emit(event, {'velocity': velocity})

    async def on_set_active_inference(self, sid, data):
        """Toggle Active Inference and signal the game to change mode."""
        await self._set_active_inference_logic(data.get('active', False))

    async def handle_api_active_inference(self, request):
        """HTTP POST handler for Active Inference toggle."""
        try:
            data = await request.json()
            active = data.get('active', False)
            await self._set_active_inference_logic(active)
            return web.json_response({'status': 'ok', 'active': active})
        except Exception as e:
            return web.json_response({'status': 'error', 'message': str(e)}, status=400)

    async def _set_active_inference_logic(self, active):
        """Common logic for toggling Active Inference."""
        logger.info(f"Control: Setting Active Inference to {active}")
        
        if hasattr(self.workspace_harness, 'is_active'):
            self.workspace_harness.is_active = active
            
        if active:
            if not self.analytics.enabled:
                self.analytics.start_session()  # fallback if no EPMs registered yet
            self.analytics.log_event("active_inference_start", {"t": time.time()})
            # Signal game to switch to single player / external mode
            await self.sio.emit('set_game_mode', {
                'mode': 'single',
                'p1': 'external',
                'p2': 'auto'
            })
        else:
            self.analytics.enabled = False
            # Signal game to return to demo mode
            await self.sio.emit('set_game_mode', {
                'mode': 'demo',
                'p1': 'auto',
                'p2': 'auto'
            })

    async def _udp_listener(self):
        """Listen for UDP RealityTokens from C++ Edge Nodes (Port 4321)"""
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', 4321))
        loop = asyncio.get_event_loop()
        logger.info("UDP RealityToken Listener running on port 4321")
        
        while True:
            try:
                data, addr = await loop.run_in_executor(None, sock.recvfrom, 65535)
                
                try:
                    msg = json.loads(data.decode())
                    if 'header' not in msg:
                        msg = {
                            'header': {'agent_id': f"udp-{addr[0]}", 'timestamp': time.time()},
                            'payload': msg
                        }
                    
                    agent_id = msg['header'].get('agent_id', f"udp-{addr[0]}")
                    self.agent_last_active[agent_id] = time.time()

                    self.voter.ingest_token(msg)

                    # Throttle monitor_feed to 10Hz: accumulate latest token per agent
                    self._pending_monitor_tokens[agent_id] = msg
                    now = time.time()
                    if now - self._last_monitor_emit >= 0.1:
                        self._last_monitor_emit = now
                        for token in self._pending_monitor_tokens.values():
                            await self.sio.emit('monitor_feed', token)
                        self._pending_monitor_tokens.clear()
                    
                except json.JSONDecodeError:
                    pass
                    
            except asyncio.CancelledError:
                sock.close()
                break
            except (BlockingIOError, InterruptedError):
                await asyncio.sleep(0.1)
            except OSError as e:
                if e.errno == 11: # EAGAIN / Resource temporarily unavailable
                    await asyncio.sleep(0.1)
                else:
                    logger.error(f"UDP Listener Runtime Error: {e}")
                    await asyncio.sleep(0.1)

    async def _voter_loop(self):
        """
        Background task to pump the Lateral Voter and trigger Active Inference.
        Runs at ~25Hz.
        """
        logger.info("Starting Unified Consensus Loop...")
        while True:
            try:
                # 1. Pump the Voter
                # process_consensus is synchronous but fast
                consensus = self.voter.process_consensus()
                
                if consensus and self.active_harness:
                    # 2. Trigger Active Inference Harness
                    # We pass candidates as a list of the last tokens in buffers
                    candidates = [b[-1] for b in self.voter.input_buffers.values() if b]
                    self.active_harness.on_consensus(consensus, candidates)
                    
                    # Log Sampled Brain Metrics
                    self.analytics.log_brain_metrics(self.active_harness)

                    # 3. Stage UI data for the UI timer to pick up at its own rate
                    meta = getattr(self.active_harness, 'meta_epm', None)
                    self._pending_ui_data = {
                        'consensus': consensus,
                        'candidates': candidates,
                        'resonance': consensus.resonance_score,
                        'tle': getattr(meta, 'last_tle', 0.0) if meta else 0.0,
                        'node_count': getattr(getattr(meta, 'memory', None), 'node_count', 0) if meta else 0
                    }
                    
                await asyncio.sleep(0.04) # 25Hz
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Voter Loop Error: {e}")
                await asyncio.sleep(1.0)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Ogma Brain Socket Server")
    parser.add_argument("--wait-agents", type=str, default="", help="Comma separated list of agent IDs to wait for")
    args = parser.parse_args()
    
    voter_node = LateralVoterNode() # Defaults
    server = BrainSocketServer(voter_node)
    
    required_agents = set()
    if args.wait_agents:
        required_agents = set(args.wait_agents.split(','))
        
    async def main():
        await server.start_server(host='0.0.0.0', port=5000)
        
        if required_agents:
            logger.info(f"Waiting for agents: {required_agents}")
            while True:
                active_agents = set(server.agent_last_active.keys())
                if required_agents.issubset(active_agents):
                    logger.info("All requested agents registered! Commencing session.")
                    break
                await asyncio.sleep(1.0)
                
        # Keep running
        while True:
            await asyncio.sleep(3600)
            
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        asyncio.run(server.stop_server())
        logger.info("Server Shutdown")

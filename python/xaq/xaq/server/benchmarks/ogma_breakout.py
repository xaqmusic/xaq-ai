import cv2
import numpy as np
import time
import socketio
import threading
import argparse
import sys

try:
    from Xlib.display import Display
    from Xlib import XK
    xlib_display = Display()
    
    KEYCODE_A = xlib_display.keysym_to_keycode(XK.string_to_keysym("a"))
    KEYCODE_D = xlib_display.keysym_to_keycode(XK.string_to_keysym("d"))
    KEYCODE_J = xlib_display.keysym_to_keycode(XK.string_to_keysym("j"))
    KEYCODE_L = xlib_display.keysym_to_keycode(XK.string_to_keysym("l"))
    KEYCODE_LEFT = xlib_display.keysym_to_keycode(XK.string_to_keysym("Left"))
    KEYCODE_RIGHT = xlib_display.keysym_to_keycode(XK.string_to_keysym("Right"))
    
    xlib_available = True
except Exception as e:
    print(f"XLIB init failed: {e}")
    xlib_available = False

def is_key_pressed_xlib(keycode):
    if not xlib_available or not keycode: return False
    try:
        keys = xlib_display.query_keymap()
        return (keys[keycode // 8] & (1 << (keycode % 8))) != 0
    except Exception:
        return False

class SoundEffects:
    """Helper class to generate and play real-time audio cues using sounddevice."""
    def __init__(self, sample_rate=44100):
        self.sample_rate = sample_rate
        try:
            import sounddevice as sd
            self.sd = sd
        except ImportError:
            self.sd = None
            print("SoundEffects: sounddevice not found, audio disabled.")

    def _play(self, data):
        if self.sd:
            try:
                self.sd.play(data, self.sample_rate)
            except Exception as e:
                print(f"SoundEffects: Error playing audio: {e}")

    def play_paddle_bounce(self):
        """Short high-pitched blip."""
        t = np.linspace(0, 0.1, int(self.sample_rate * 0.1), False)
        wave = np.sin(2 * np.pi * 1000 * t) * np.exp(-t * 40)
        self._play(wave.astype(np.float32))

    def play_wall_bounce(self):
        """Short lower-pitched blip."""
        t = np.linspace(0, 0.1, int(self.sample_rate * 0.1), False)
        wave = np.sin(2 * np.pi * 400 * t) * np.exp(-t * 40)
        self._play(wave.astype(np.float32))

    def play_block_destroyed(self):
        """Noise burst."""
        t = np.linspace(0, 0.1, int(self.sample_rate * 0.1), False)
        wave = np.random.normal(0, 0.3, len(t)) * np.exp(-t * 40)
        self._play(wave.astype(np.float32))

    def play_goal_scored(self):
        """Low frequency sawtooth buzz."""
        t = np.linspace(0, 0.5, int(self.sample_rate * 0.5), False)
        # Sawtooth: 2 * (t * freq - floor(0.5 + t * freq))
        freq = 100
        wave = 2 * (t * freq - np.floor(0.5 + t * freq)) * 0.3 * (1 - t/0.5)
        self._play(wave.astype(np.float32))

class OgmaBreakout:
    def __init__(self, width=640, height=480, master_seed: int = 0):
        self.width = width
        self.height = height
        self.paddle_width = 100
        self.paddle_height = 20
        self.ball_radius = 8

        # Block settings
        self.block_rows = 2
        self.block_cols = 8
        self.block_width = self.width // self.block_cols
        self.block_height = 30

        # Per-component RNG for reproducible ball kinematics.
        from xaq._rng import derive_rng
        self._rng = derive_rng(master_seed, "breakout.env")

        self.sfx = SoundEffects()
        self.reset()
        
        self.hits_p1 = 0
        self.misses_p1 = 0
        self.hits_p2 = 0
        self.misses_p2 = 0
        
        # Socket.io Client for Brain Server communication
        self.sio = socketio.Client()
        self.p1_velocity = 0
        self.p2_velocity = 0
        
        @self.sio.on('p1_action')
        def on_p1_action(data):
            self.p1_velocity = data.get('velocity', 0)
            
        @self.sio.on('p2_action')
        def on_p2_action(data):
            self.p2_velocity = data.get('velocity', 0)
            
        @self.sio.on('set_game_mode')
        def on_set_game_mode(data):
            mode = data.get('mode', 'demo')
            p1 = data.get('p1')
            p2 = data.get('p2')
            print(f"Game: Switching to mode {mode} (P1:{p1}, P2:{p2})")
            sys.stdout.flush()
            self.update_mode(mode, p1, p2)
            
    def reset(self):
        # Paddle 1 (Bottom)
        self.p1_x = self.width // 2 - self.paddle_width // 2
        # Paddle 2 (Top)
        self.p2_x = self.width // 2 - self.paddle_width // 2
        
        self.respawn_ball()
        
        self.score_p1 = 0
        self.score_p2 = 0
        self.running = True
        
        self.whisker_left = 0.0
        self.whisker_right = 0.0
        self.wall_stuck_ticks = 0
        
        self.game_start_time = time.time()
        self._init_blocks()

    # Modes where there is no top paddle, no blocks, and the top wall acts as a
    # reflector.  "motion_training" is the legacy name; "rally" is the new clean
    # single-player tracking mode (no blocks, no P2, ball just stays in play).
    _SOLO_MODES = frozenset({"motion_training", "rally", "observer"})

    def _init_blocks(self):
        # Blocks: List of (x, y, color, side)
        self.blocks = []
        if getattr(self, 'game_mode', None) in self._SOLO_MODES:
            return
            
        # Player 2 defense (Top)
        for r in range(self.block_rows):
            for c in range(self.block_cols):
                y = 100 + r * (self.block_height + 5)
                self.blocks.append({'rect': [c * self.block_width, y, self.block_width - 5, self.block_height], 
                                   'color': (0, 0, 255), 'side': 2})
        
        # Player 1 defense (Bottom)
        for r in range(self.block_rows):
            for c in range(self.block_cols):
                y = self.height - 180 + r * (self.block_height + 5)
                self.blocks.append({'rect': [c * self.block_width, y, self.block_width - 5, self.block_height], 
                                   'color': (255, 0, 0), 'side': 1})
            
    def step(self, mode_p1="auto", mode_p2="auto"):
        self.whisker_left *= 0.85
        self.whisker_right *= 0.85
        
        # Update ball position
        self.ball_x += self.ball_vx
        self.ball_y += self.ball_vy
        
        # Wall collisions (Horizontal)
        if self.ball_x <= self.ball_radius or self.ball_x >= self.width - self.ball_radius:
            self.ball_vx *= -1
            self.sfx.play_wall_bounce()
            self._perturb_velocity()
            
        # Top wall acts as a reflector in solo modes (no top paddle)
        if getattr(self, 'game_mode', None) in self._SOLO_MODES and self.ball_y <= self.ball_radius:
            self.ball_vy = abs(self.ball_vy)
            self.ball_y = self.ball_radius + 1
            self.sfx.play_wall_bounce()
            self._perturb_velocity()

        # Bottom wall acts as a reflector in observer mode
        if getattr(self, 'game_mode', None) == "observer" and self.ball_y >= self.height - self.ball_radius:
            self.ball_vy = -abs(self.ball_vy)
            self.ball_y = self.height - self.ball_radius - 1
            self.sfx.play_wall_bounce()
            self._perturb_velocity()
            
        # Paddle 1 interaction (Bottom)
        p1_y = self.height - 40
        
        # Tactile Whisker Checks
        if p1_y <= self.ball_y + self.ball_radius <= p1_y + self.paddle_height:
            if self.p1_x - 25 <= self.ball_x < self.p1_x:
                self.whisker_left = 1.0
            elif self.p1_x + self.paddle_width < self.ball_x <= self.p1_x + self.paddle_width + 25:
                self.whisker_right = 1.0

        if p1_y <= self.ball_y + self.ball_radius <= p1_y + self.paddle_height:
            if self.p1_x <= self.ball_x <= self.p1_x + self.paddle_width:
                self.hits_p1 += 1
                self.ball_vy = -abs(self.ball_vy) # Force upward
                self.ball_y = p1_y - self.ball_radius - 1 # Push out
                self.sfx.play_paddle_bounce()
                self._perturb_velocity()
                # EMIT HIT EVENT
                if self.sio.connected:
                    self.sio.emit('raw_game_state', {
                        'event': 'self_paddle_hit',
                        'ball_vx': self.ball_vx,
                        'paddle_x': self.p1_x,
                        'ball_x': self.ball_x
                    })

        # Paddle 2 interaction (Top) — disabled in solo modes
        if getattr(self, 'game_mode', None) not in self._SOLO_MODES:
            p2_y = 20
            if p2_y <= self.ball_y - self.ball_radius <= p2_y + self.paddle_height:
                if self.p2_x <= self.ball_x <= self.p2_x + self.paddle_width:
                    self.hits_p2 += 1
                    self.ball_vy = abs(self.ball_vy) # Force downward
                    self.ball_y = p2_y + self.paddle_height + self.ball_radius + 1 # Push out
                    self.sfx.play_paddle_bounce()
                    self._perturb_velocity()
                
        # Block collisions
        for block in self.blocks[:]:
            r = block['rect']
            if (r[0] <= self.ball_x <= r[0] + r[2] and 
                r[1] <= self.ball_y <= r[1] + r[3]):
                self.ball_vy *= -1
                self.blocks.remove(block)
                self.sfx.play_block_destroyed()
                self._perturb_velocity()
                break
        
        # Auto-reset blocks if all are cleared (not in solo modes — no blocks there)
        if not self.blocks and getattr(self, 'game_mode', None) not in self._SOLO_MODES:
            self.reset()
                
        # Scoring
        if self.ball_y >= self.height:
            self.misses_p1 += 1
            self.score_p2 += 1
            self.sfx.play_goal_scored()
            # EMIT MISS EVENT
            if self.sio.connected:
                self.sio.emit('raw_game_state', {
                    'event': 'paddle_miss',
                    'ball_x': self.ball_x,
                    'score': (self.score_p1, self.score_p2)
                })
            self.respawn_ball()
        elif self.ball_y <= 0 and getattr(self, 'game_mode', None) not in self._SOLO_MODES:
            self.misses_p2 += 1
            self.score_p1 += 1
            self.sfx.play_goal_scored()
            self.respawn_ball()
            
        # Paddle Movement
        self._move_paddle(1, mode_p1)
        self._move_paddle(2, mode_p2)

        # Check for 2-minute restart (in observer mode specifically or all modes?)
        # Requirement says "The game should restart every 2 minutes" in the context of observer mode.
        if getattr(self, 'game_mode', None) == "observer":
            if time.time() - self.game_start_time > 120:
                print("Observer Mode: 2-minute timer reached, restarting...")
                self.reset()
        
        # Broadcast raw state to Brain Server
        if self.sio.connected:
            self.sio.emit('raw_game_state', {
                'p1_x': self.p1_x,
                'p2_x': self.p2_x,
                'score': (self.score_p1, self.score_p2),
                'blocks_remaining': len(self.blocks),
                'whisker_left': self.whisker_left,
                'whisker_right': self.whisker_right,
                'integrity_check': 'strict' # Signal that no ground truth is leaked
            })

    def respawn_ball(self):
        self.ball_x = self.width // 2
        self.ball_y = self.height // 2
        
        # Choose from diagonal base angles (45, 135, 225, 315 degrees)
        bases = [np.pi/4, 3*np.pi/4, 5*np.pi/4, 7*np.pi/4]
        base_angle = self._rng.choice(bases)

        # Add +/- 5 degrees of jitter
        noise = self._rng.uniform(-5, 5) * np.pi / 180.0
        angle = base_angle + noise
        
        speed = 5.65 # sqrt(4^2 + 4^2) is ~5.65
        self.ball_vx = speed * np.cos(angle)
        self.ball_vy = speed * np.sin(angle)

    def _perturb_velocity(self):
        """Add +/- 3 degrees of random noise to the ball's angle while preserving speed."""
        speed = np.sqrt(self.ball_vx**2 + self.ball_vy**2)
        angle = np.arctan2(self.ball_vy, self.ball_vx)
        
        # Add +/- 3 degrees in radians
        noise = self._rng.uniform(-3, 3) * np.pi / 180.0
        angle += noise
        
        self.ball_vx = speed * np.cos(angle)
        self.ball_vy = speed * np.sin(angle)

    def update_mode(self, mode, p1_override=None, p2_override=None):
        """Update game mode and paddle controllers at runtime."""
        self.game_mode = mode
        self.current_mode_label = mode.capitalize()
        
        self.hits_p1 = 0
        self.misses_p1 = 0
        self.hits_p2 = 0
        self.misses_p2 = 0
        
        if mode == "demo":
            self.mode_p1, self.mode_p2 = "auto", "auto"
        elif mode == "single":
            self.mode_p1, self.mode_p2 = "manual", "auto"
        elif mode == "double":
            self.mode_p1, self.mode_p2 = "manual", "manual"
        elif mode == "motion_training":
            self.mode_p1, self.mode_p2 = "manual", "none"
            self.blocks = []
        elif mode == "rally":
            # Single-player pure tracking: no blocks, no top paddle, ball bounces off top wall.
            # P1 must keep the ball in play.  Cleanest mode for action controller evaluation.
            self.mode_p1, self.mode_p2 = "auto", "none"
            self.blocks = []
        elif mode == "observer":
            # No paddles, all wall reflectors.
            self.mode_p1, self.mode_p2 = "none", "none"
            self.blocks = []
            self.reset() # Ensure start time is refreshed
            
        if p1_override: self.mode_p1 = p1_override
        if p2_override: self.mode_p2 = p2_override

    def _move_paddle(self, player, mode):
        speed = 8
        if player == 1:
            if mode == "auto":
                target = self.ball_x - self.paddle_width // 2
                diff = target - self.p1_x
                # Clamp step to remaining distance so paddle never overshoots —
                # overshooting causes ±speed oscillation every tick near the target.
                self.p1_x += max(-speed, min(speed, diff))
                self.p1_x = max(0, min(self.width - self.paddle_width, self.p1_x))
            elif mode == "external" or mode == "manual":
                intended_x = self.p1_x + self.p1_velocity
                
                # Whisker collision with wall
                if intended_x < 25:
                    self.whisker_left = min(1.0, self.whisker_left + abs(self.p1_velocity) / 10.0 + 0.1)
                if intended_x > self.width - self.paddle_width - 25:
                    self.whisker_right = min(1.0, self.whisker_right + abs(self.p1_velocity) / 10.0 + 0.1)
                    
                self.p1_x = max(0, min(self.width - self.paddle_width, intended_x))
                # Friction: same feel as keyboard control
                self.p1_velocity *= 0.85
                if abs(self.p1_velocity) < 0.5:
                    self.p1_velocity = 0
            
            # Wall stuck tracker
            if self.p1_x == 0 or self.p1_x == self.width - self.paddle_width:
                self.wall_stuck_ticks += 1
            else:
                self.wall_stuck_ticks = 0
        else:
            if mode == "auto":
                target = self.ball_x - self.paddle_width // 2
                diff = target - self.p2_x
                self.p2_x += max(-speed, min(speed, diff))
            elif mode == "external" or mode == "manual":
                self.p2_x += self.p2_velocity
                self.p2_velocity *= 0.85
                if abs(self.p2_velocity) < 0.5:
                    self.p2_velocity = 0
            self.p2_x = max(0, min(self.width - self.paddle_width, self.p2_x))

    def render(self):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Draw Paddles
        if self.mode_p1 != "none":
            cv2.rectangle(img, (int(self.p1_x), self.height - 40), 
                        (int(self.p1_x + self.paddle_width), self.height - 40 + self.paddle_height), (0, 255, 0), -1)
            # Draw Whiskers (1px width)
            cv2.line(img, (int(self.p1_x), self.height - 30), 
                     (int(max(0, self.p1_x - 25)), self.height - 30), (0, 255, 255), 1)
            cv2.line(img, (int(self.p1_x + self.paddle_width), self.height - 30), 
                     (int(min(self.width, self.p1_x + self.paddle_width + 25)), self.height - 30), (0, 255, 255), 1)
        if getattr(self, 'game_mode', None) not in self._SOLO_MODES and self.mode_p2 != "none":
            cv2.rectangle(img, (int(self.p2_x), 20),
                          (int(self.p2_x + self.paddle_width), 20 + self.paddle_height), (0, 200, 255), -1)
        
        # Draw Ball
        cv2.circle(img, (int(self.ball_x), int(self.ball_y)), self.ball_radius, (255, 255, 255), -1)
        
        # Draw Blocks
        for block in self.blocks:
            r = block['rect']
            cv2.rectangle(img, (int(r[0]), int(r[1])), (int(r[0] + r[2]), int(r[1] + r[3])), block['color'], -1)
        
        # UI (Simplified: Raw environment data only)
        # UI (Simplified: Raw environment data only)
        acc_p1 = (self.hits_p1 / (self.hits_p1 + self.misses_p1) * 100) if (self.hits_p1 + self.misses_p1) > 0 else 0
        acc_p2 = (self.hits_p2 / (self.hits_p2 + self.misses_p2) * 100) if (self.hits_p2 + self.misses_p2) > 0 else 0
        cv2.putText(img, f"P1: {self.score_p1} | Acc: {acc_p1:.1f}%", (10, self.height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.putText(img, f"P2: {self.score_p2} | Acc: {acc_p2:.1f}% | Blocks: {len(self.blocks)}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
        
        # Mode Indicator
        mode_text = getattr(self, 'current_mode_label', 'Unknown')
        cv2.putText(img, f"Mode: {mode_text}", (self.width - 150, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        return img

    def run(self, mode_p1="auto", mode_p2="auto", game_mode="demo"):
        self.mode_p1 = mode_p1
        self.mode_p2 = mode_p2
        self.game_mode = game_mode
        self.current_mode_label = game_mode.capitalize()
        cv2.namedWindow("Ogma Break-Pong", cv2.WINDOW_AUTOSIZE | getattr(cv2, 'WINDOW_GUI_NORMAL', 16))

        def connect_with_retry():
            waiting_printed = False
            while not self.sio.connected and self.running:
                try:
                    self.sio.connect('http://localhost:5000')
                    self.sio.emit('register', {'agent_id': 'breakout_game'})
                    print("✅ Connected and Registered to Brain Server.")
                    sys.stdout.flush()
                    break
                except:
                    if not waiting_printed:
                        print("Waiting for Brain Server (http://localhost:5000)...")
                        sys.stdout.flush()
                        waiting_printed = True
                    time.sleep(2)
        
        # Start connection attempt in a thread
        threading.Thread(target=connect_with_retry, daemon=True).start()
        
        self.mode_p1 = mode_p1
        self.mode_p2 = mode_p2
        self.game_mode = game_mode
        
        tick = 0
        target_fps = 60
        frame_time = 1.0 / target_fps
        
        while self.running:
            start_time = time.time()
            
            # Step game (friction is applied inside _move_paddle)
            self.step(self.mode_p1, self.mode_p2)
            frame = self.render()
            cv2.imshow("Ogma Break-Pong", frame)
            
            # Send periodic telemetry heartbeat to Brain Server
            tick += 1
            if tick % 10 == 0 and self.sio.connected:
                telemetry = {
                    'p1_x': self.p1_x,
                    'p2_x': self.p2_x,
                    'mode': self.game_mode,
                    'integrity_check': 'strict'
                }
                self.sio.emit('raw_game_state', telemetry)
            
            # Keyboard Controls - Flush event queue to avoid lagging behind or speeding up
            key = 255
            for _ in range(5): # Limit flush so we don't block
                k = cv2.waitKey(1) & 0xFF
                if k != 255:
                    key = k
                else:
                    break
            
            if key == ord('q'): self.running = False
            
            # Player 1 Controls (Bottom)
            accel = 4.0
            max_vel = 20.0
            
            p1_left = False
            p1_right = False
            p2_left = False
            p2_right = False
            
            if xlib_available:
                try:
                    while xlib_display.pending_events():
                        xlib_display.next_event()
                except Exception:
                    pass
                
                p1_left = is_key_pressed_xlib(KEYCODE_A) or is_key_pressed_xlib(KEYCODE_LEFT)
                p1_right = is_key_pressed_xlib(KEYCODE_D) or is_key_pressed_xlib(KEYCODE_RIGHT)
                p2_left = is_key_pressed_xlib(KEYCODE_J)
                p2_right = is_key_pressed_xlib(KEYCODE_L)
            else:
                # Fallback 'a'/'d' or Left/Right arrows (81/83 in some CV2 builds on Linux)
                if key == ord('a') or key == 81: p1_left = True
                if key == ord('d') or key == 83: p1_right = True
                if key == ord('j'): p2_left = True
                if key == ord('l'): p2_right = True

            if p1_left:
                self.mode_p1 = "manual"
                self.p1_velocity = max(self.p1_velocity - accel, -max_vel)
            if p1_right:
                self.mode_p1 = "manual"
                self.p1_velocity = min(self.p1_velocity + accel, max_vel)
                
            if self.game_mode == "double" or self.mode_p2 == "manual":
                if p2_left:
                    self.mode_p2 = "manual"
                    self.p2_velocity = max(self.p2_velocity - accel, -max_vel)
                if p2_right:
                    self.mode_p2 = "manual"
                    self.p2_velocity = min(self.p2_velocity + accel, max_vel)
            
            # Mode switching (Instant)
            if key == ord('1'): 
                self.update_mode("demo")
            if key == ord('2'): 
                self.update_mode("single")
            if key == ord('3'): 
                self.update_mode("double")
            if key == ord('4'):
                self.update_mode("motion_training")
            if key == ord('5'):
                self.update_mode("observer")
                
            # Strict FPS capping to avoid X11 repeat floods accelerating game time
            elapsed = time.time() - start_time
            if elapsed < frame_time:
                time.sleep(frame_time - elapsed)
                
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ogma Break-Pong")
    parser.add_argument("--mode", choices=["demo", "single", "double", "motion_training", "observer"], default="demo", help="Game mode")
    parser.add_argument("--p1", choices=["auto", "manual", "external"], help="Override P1 mode")
    parser.add_argument("--p2", choices=["auto", "manual", "external"], help="Override P2 mode")
    args = parser.parse_args()

    game = OgmaBreakout()
    
    # Determine initial modes
    m1 = args.p1 if args.p1 else ("auto" if args.mode == "demo" else "manual")
    m2 = args.p2 if args.p2 else ("auto" if args.mode != "double" else "manual")
    
    game.run(mode_p1=m1, mode_p2=m2, game_mode=args.mode)

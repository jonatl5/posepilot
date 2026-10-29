"""Planar command-response simulator. NOT a validated TT flight model.

Wall x=0, stationary poster at (0,0), drone x<0; heading positive toward +y.
Pixels/ToF are synthetic sensors. Truth is used only for dynamics/reward/info.
"""
from collections import deque
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from .core import Config, Detection, RangeSample, FeatureBuilder, Supervisor

class ApproachEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 10}

    def __init__(self, config=None, randomized=True, render_mode=None):
        self.c = config or Config()
        self.randomized, self.render_mode = randomized, render_mode
        self.observation_space = spaces.Box(-1., 1., (16*self.c.history,), np.float32)
        self.action_space = spaces.Box(-1., 1., (2,), np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        r, c = self.np_random, self.c
        self.x = -float(r.uniform(1.4, 3.0))
        self.y = float(r.uniform(-.65, .65))
        self.heading = float(r.uniform(-.25, .25))
        self.v = self.w = self.time = 0.
        self.steps = self.hold = self.lost = 0
        self.success = False
        self.previous = np.zeros(2, dtype=np.float32)
        self.gain = float(r.uniform(.18, .4)) if self.randomized else .28
        self.yaw_gain = float(r.uniform(.4, .9)) if self.randomized else .65
        self.tau = float(r.uniform(.15, .45)) if self.randomized else .25
        self.camera_lag = int(r.integers(0, 4)) if self.randomized else 1
        self.command_lag = int(r.integers(0, 3)) if self.randomized else 1
        self.dropout = float(r.uniform(0, .12)) if self.randomized else 0.
        self.visual_bias = float(r.uniform(.9, 1.1)) if self.randomized else 1.
        self.hfov = math.radians(float(r.uniform(66, 78)) if self.randomized else 72)
        self.sensors = deque(maxlen=self.camera_lag+1)
        self.commands = deque([np.zeros(2)]*self.command_lag)
        self.builder, self.supervisor = FeatureBuilder(c), Supervisor(c)
        self.obs = self._observe()
        return self.obs, self._info("running")

    def _geometry(self):
        distance = math.hypot(self.x, self.y)
        bearing = math.atan2(-self.y, -self.x)-self.heading
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        ex = math.tan(bearing)/(2*math.tan(self.hfov/2)) if abs(bearing)<1.5 else 2.
        return distance, bearing, ex

    def _observe(self):
        r, c = self.np_random, self.c
        distance, bearing, ex = self._geometry()
        det = None
        if abs(ex) < .48 and r.random() >= self.dropout:
            noise = .006 if self.randomized else 0
            det = Detection(.5+ex+float(r.normal(0,noise)), .5+float(r.normal(0,noise)),
                            min(.95, c.visual_scale_m*.7/distance),
                            min(.95, c.visual_scale_m*self.visual_bias/distance*float(r.uniform(.97,1.03))),
                            .9, self.time, self.steps)
        self.sensors.append(det)
        delayed = self.sensors[0] if len(self.sensors)==self.sensors.maxlen else None
        ray = -self.x/math.cos(self.heading) if math.cos(self.heading)>.1 else 100.
        range_sample = RangeSample(None, self.time, "sim_forward", "out_of_range")
        if .02 < ray <= c.forward_range_max_m and r.random() >= self.dropout*.5:
            range_sample = RangeSample(max(.02,ray+float(r.normal(0,.01))), self.time, "sim_forward")
        return self.builder.build(delayed, range_sample, self.time, self.previous)

    def step(self, action):
        c = self.c
        safe, reason = self.supervisor.apply(action, self.obs)
        self.commands.append(safe.copy())
        applied = self.commands.popleft()
        alpha = 1-math.exp(-c.dt/self.tau)
        self.v += alpha*(self.gain*applied[0]-self.v)
        self.w += alpha*(self.yaw_gain*applied[1]-self.w)
        self.heading += self.w*c.dt
        self.x += self.v*math.cos(self.heading)*c.dt
        self.y += self.v*math.sin(self.heading)*c.dt
        self.steps += 1
        self.time = self.steps*c.dt
        distance, bearing, ex = self._geometry()
        clearance = -self.x
        smooth = float(np.square(safe-self.previous).sum())
        self.previous = safe
        self.obs = self._observe()
        f = self.obs[-16:]
        arrived = (f[6]>.5 and abs(f[0])<c.arrival_x
                   and abs(f[4]*c.forward_range_max_m-c.stand_off_m)<c.arrival_range_m
                   and abs(self.v)<.04 and abs(clearance-c.stand_off_m)<c.arrival_range_m)
        self.hold = self.hold+1 if arrived else 0
        self.lost = self.lost+1 if f[8]<.5 else 0
        self.success = self.hold*c.dt >= c.arrival_hold_s
        collision = clearance <= c.minimum_m
        out = abs(self.y)>2 or self.x < -4 or abs(self.heading)>math.pi/2
        lost = self.lost*c.dt > 3.
        terminated = bool(self.success or collision or out or lost)
        truncated = bool(self.steps>=c.max_steps and not terminated)
        # Truth-dependent reward, deliberately absent from the policy observation.
        reward = (-.6*min(abs(distance-c.stand_off_m),3) - 1.5*min(abs(ex),1)
                  -.08*smooth -.02)
        if self.success: reward += 30
        if collision: reward -= 30
        if out or lost: reward -= 10
        outcome = "success" if self.success else "minimum_distance" if collision else "boundary" if out else "lost" if lost else "timeout" if truncated else "running"
        info = self._info(outcome)
        info.update(supervisor=reason, applied_action=applied.tolist(), safe_action=safe.tolist(), smoothness=smooth)
        return self.obs, float(reward), terminated, truncated, info

    def _info(self, outcome):
        distance, _, ex = self._geometry()
        return {"outcome":outcome, "success":bool(self.success), "wall_clearance_m":float(-self.x),
                "target_distance_m":float(distance), "center_error":float(ex), "time_s":float(self.time)}

    def render(self):
        import cv2
        canvas = np.full((480,640,3),245,np.uint8)
        distance, _, ex = self._geometry()
        h = int(min(400,self.c.visual_scale_m/distance*480))
        x = int((.5+ex)*640)
        cv2.rectangle(canvas,(x-h//3,240-h//2),(x+h//3,240+h//2),(40,90,160),-1)
        cv2.putText(canvas,"SYNTHETIC SENSORS - NOT DETECTOR VALIDATION",(12,24),0,.45,(0,0,0),1)
        return canvas

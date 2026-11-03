"""Conservative F1-like local trajectory sampling; no external track state."""
from dataclasses import dataclass
import math
import numpy as np
from team01_race.race_strategy import RacingLine, LapLearner
from team01_race.curved_path import adaptive_pursuit_candidate, preview_curvature


@dataclass
class Motion:
    speed: float
    steering: float  # race contract: +right, -left, radians
    mode: str
    reason: str


class TrafficGate:
    """Arm only on real red-light observations; launch on stable all-off.

    A partial camera field of view is permitted: 4 visible red LEDs may be
    the peak of a physical 5-LED sequence. Count-only mode is for unit tests;
    runtime mode also checks the change in intensity from a dark baseline.
    Before any light is observed, a blank image can never authorize movement.
    """
    def __init__(self, required=4, on_frames=2, off_frames=3,
                 min_sequence_s=3.0):
        self.required=int(required)
        self.on_frames=int(on_frames)
        self.off_frames=int(off_frames)
        self.min_sequence_s=float(min_sequence_s)
        self.active_frames=0
        self.dark_frames=0
        self.saw_full_red=False
        self.lights_out=False
        self.frame_count=0
        self.baseline_samples=[]
        self.baseline=None
        self.peak_strength=0.0
        self.first_bright_time=None
        self.last_blobs=0
        self.last_strength=0.0
        self.first_bright_blobs=0
        self.armed_at=None
        self.maximum_blobs=0
        # Slow headless cameras may see each light count just once.
        self.progressive_red_counts=set()
        self.status='WAIT_RED'

    def observe(self, blobs, strength=None, timestamp=None):
        import time
        t=time.monotonic() if timestamp is None else float(timestamp)
        blobs=max(0,int(blobs))
        self.last_blobs=blobs
        self.maximum_blobs=max(blobs,self.maximum_blobs)
        if self.lights_out:
            return True
        if strength is None:
            # Test/diagnostic mode: no illumination measurements available.
            if blobs>=self.required:
                self.active_frames+=1
                self.dark_frames=0
                if self.active_frames>=self.on_frames:
                    self.saw_full_red=True
                    self.status='ARMED'
            elif self.saw_full_red and blobs==0:
                self.dark_frames+=1
                self.status='VERIFY_OUT'
                if self.dark_frames>=self.off_frames:
                    self.lights_out=True
                    self.status='LIGHTS_OUT'
            else:
                if not self.saw_full_red:self.active_frames=0
                self.dark_frames=0
            return self.lights_out

        strength=max(0.,float(strength))
        if not math.isfinite(strength):return False
        self.last_strength=strength
        self.frame_count+=1
        if self.baseline is None:
            # The launcher holds the lamps off until all ROS streams have
            # initialized. Median suppresses an occasional noisy first frame.
            self.baseline_samples.append(strength)
            if len(self.baseline_samples)>=8:
                self.baseline=float(np.median(self.baseline_samples))
            return False
        excess=strength-self.baseline
        # Emissive overlays have a substantial energy difference relative
        # to the unlit gantry. 4/5 visible lamps are sufficient to arm.
        delta_threshold=max(120.,self.baseline*.18)
        illuminated=excess>delta_threshold
        if illuminated:
            if blobs>0:
                self.progressive_red_counts.add(blobs)
            if self.first_bright_time is None:
                self.first_bright_time=t
                self.first_bright_blobs=blobs
            self.peak_strength=max(self.peak_strength,strength)
        if not self.saw_full_red:
            if illuminated and blobs>=self.required:
                self.active_frames+=1
                # A real increasing 1->3->4 sequence can be trusted even
                # when the 4-lamp image arrived only once. A solitary 4
                # flash still requires the original multi-frame proof.
                observed_ramp=(self.first_bright_time is not None and
                    t-self.first_bright_time>=.6 and
                    len([k for k in self.progressive_red_counts if 0<k<self.required])>=2)
                if self.active_frames>=self.on_frames or observed_ramp:
                    self.saw_full_red=True
                    self.armed_at=t
                    self.status='ARMED'
            else:
                self.active_frames=0
            return False
        # No movement before the physical red sequence can have ended.
        # Total time measured from *first visible red* is >=3.0 s; thus
        # a temporary black frame after lamp #4 cannot trigger departure.
        if self.first_bright_time is None:
            return False
        # If the first three lamps were outside the FOV, the first clearly
        # detected event may already be 4 reds. The sequence is still valid
        # if four reds were held, then disappear stably after an arm delay.
        elapsed=t-self.first_bright_time
        last_four_only=(self.first_bright_blobs>=self.required and
                        elapsed>=.65 and self.armed_at is not None and
                        t-self.armed_at>=.45)
        if elapsed<self.min_sequence_s-1e-6 and not last_four_only:
            return False
        observed_gain=max(0.,self.peak_strength-self.baseline)
        returned_to_dark=strength<=self.baseline+max(delta_threshold*.5,observed_gain*.30)
        # Use count=0 when visible lamps vanish. The fallback energy drop
        # handles permanent red lens appearance after emissive model removal.
        all_off=(blobs==0 or returned_to_dark)
        if all_off:
            self.dark_frames+=1
            self.status='VERIFY_OUT'
            if self.dark_frames>=self.off_frames:
                self.lights_out=True
                self.status='LIGHTS_OUT'
        else:
            self.dark_frames=0
            self.status='ARMED'
        return self.lights_out



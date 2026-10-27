"""차량 운전 제어 핵심: fail-closed 게이트, 센서 watchdog, 조향/가속 제한."""
from __future__ import annotations
from dataclasses import dataclass,field
import math
from vehicle_generator.control import setpoints
from vehicle_generator.model import Vehicle
from .lane import LaneEstimate

@dataclass
class CameraDrive:
    vehicle:Vehicle
    camera_timeout_s:float=.42
    gate_timeout_s:float=.26
    speed_limit_mps:float=1.10
    turn_speed_mps:float=.42
    steer_limit_rad:float=.32
    accel_mps2:float=.55
    steer_rate_rps:float=.85
    last_camera_at:float|None=None
    last_gate_at:float|None=None
    started:bool=False
    lane:LaneEstimate=field(default_factory=lambda:LaneEstimate(False))
    speed:float=0.
    steering:float=0.
    previous_tick:float|None=None
    last_reason:str=""

    def camera(self,estimate:LaneEstimate,at:float):
        if not math.isfinite(at):raise ValueError("카메라 수신 시각이 유한하지 않습니다")
        if self.last_camera_at is not None and at<self.last_camera_at:
            self.started=False
            self.lane=LaneEstimate(False,reason="카메라 시간 역행")
            raise ValueError("카메라 시간 역행")
        self.last_camera_at=at;self.lane=estimate

    def permission(self,allowed:bool,at:float):
        if not math.isfinite(at):raise ValueError("출발 권한 수신 시각이 유한하지 않습니다")
        if self.last_gate_at is not None and at<self.last_gate_at:
            self.started=False;raise ValueError("출발 권한 수신 시각 역행")
        self.last_gate_at=at
        self.started=bool(allowed)
        if not self.started:self.speed=0.  # 출발 취소는 가속 램프보다 우선

    def tick(self,at:float):
        if not math.isfinite(at):
            raise ValueError("틱 시각 오류")
        if self.previous_tick is not None and at<=self.previous_tick:
            self.speed=self.steering=0.
            self.started=False
            raise ValueError("틱 시간 중복 또는 역행")
        elapsed=0. if self.previous_tick is None else min(.3,at-self.previous_tick)
        self.previous_tick=at
        fail=None
        if not self.started:fail="출발 미허가"
        elif self.last_gate_at is None or at-self.last_gate_at>self.gate_timeout_s or at<self.last_gate_at:
            fail="출발 허가 watchdog 만료"
        elif self.last_camera_at is None or at-self.last_camera_at>self.camera_timeout_s or at<self.last_camera_at:
            fail="주행 카메라 watchdog 만료"
        elif not self.lane.valid:
            fail=self.lane.reason or "카메라 도로 검출 실패"
        if fail:
            # 차량이 정지하도록 목표 휠 속도를 0으로 즉시 명령한다.
            self.last_reason=fail
            self.speed=self.steering=0.
        else:
            self.last_reason="카메라 도로 인식 주행"
            target_steer=max(-self.steer_limit_rad,min(self.steer_limit_rad,
                1.45*self.lane.error))
            if elapsed:
                max_delta=self.steer_rate_rps*elapsed
                self.steering=max(self.steering-max_delta,
                                  min(self.steering+max_delta,target_steer))
            target_speed=self.turn_speed_mps+(self.speed_limit_mps-self.turn_speed_mps)*(
                    (1-abs(self.steering)/self.steer_limit_rad)**1.3)
            target_speed*=max(.55,self.lane.confidence)
            if elapsed:self.speed=min(target_speed,self.speed+self.accel_mps2*elapsed)
        targets=setpoints(self.vehicle,self.speed,self.steering)
        return {"allowed":fail is None,"reason":self.last_reason,
                "speed_mps":self.speed,"race_steer_rad":self.steering,
                **targets}

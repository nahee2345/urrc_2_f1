"""Manually validated speed stages; never override live road safety."""


class SpeedStages:
    def __init__(self,cfg):
        requested=float(cfg.get('validated_speed_stage_mps',5.0))
        self.requested=requested if requested in (5.0,8.0,10.0) else 5.0
        self.effective=5.0

    def cap(self,map_ready):
        self.effective=self.requested if map_ready else 5.0
        return self.effective

"""High speeds remain locked until independent Gazebo QA signs each stage."""


class SpeedStages:
    def __init__(self,cfg):
        requested=float(cfg.get('validated_speed_stage_mps',5.))
        self.requested=requested if requested in (5.,8.,10.) else 5.
        self.corner=cfg.get('qa_approved_first_corner',False) is True
        self.lap=cfg.get('qa_approved_monza_lap',False) is True
        self.eight=cfg.get('qa_approved_8mps',False) is True
        self.effective=5.
        self.reason='QA_NOT_APPROVED'

    def cap(self,map_ready):
        if not map_ready:
            self.effective=5.;self.reason='MAP_UNVERIFIED'
        elif not (self.corner and self.lap):
            self.effective=5.;self.reason='FIRST_CORNER_OR_LAP_NOT_APPROVED'
        elif self.requested==10. and not self.eight:
            self.effective=8.;self.reason='EIGHT_MPS_NOT_APPROVED'
        else:
            self.effective=self.requested;self.reason='QA_APPROVED'
        return self.effective

#!/usr/bin/env python3
"""Independent QA frame/unit regression checks."""
import json
import math
import tempfile
from pathlib import Path
from team01_race.qa_lap_frame import OdomWorld

def main():
    with tempfile.TemporaryDirectory() as folder:
        path=Path(folder)/'spawn.json'
        x0=.37867235;y0=-2.0560354;theta=1.45738442
        path.write_text(json.dumps({'x':x0,'y':y0,'yaw':theta}))
        relative=OdomWorld(path)
        x,y,h=relative.transform(1.,0.,0.)
        assert relative.mode=='SPAWN_RELATIVE'
        assert abs(x-(x0+math.cos(theta)))<1e-6
        assert abs(y-(y0+math.sin(theta)))<1e-6
        assert abs(h-theta)<1e-6
        world=OdomWorld(path)
        assert world.transform(x0,y0,theta)==(x0,y0,theta)
        assert world.mode=='WORLD'
        unknown=OdomWorld(path)
        try:
            unknown.transform(5.,5.,0.)
        except ValueError:
            pass
        else:
            raise AssertionError('unknown frame must fail closed')
    print('[QA] coordinate-frame regression PASS')
if __name__=='__main__':main()

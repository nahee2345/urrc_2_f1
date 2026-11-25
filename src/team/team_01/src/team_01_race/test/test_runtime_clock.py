import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team_01_race.runtime_clock import RuntimeClock

def test_slow_simulation_elapsed_time_does_not_become_wall_time():
 c=RuntimeClock();assert c.update(1.,10.)==1.
 assert abs(c.update(1.1,11.)-1.1)<1e-9
 assert c.alive(11.2,.25)

def test_repeated_clock_messages_do_not_extend_liveness():
 c=RuntimeClock();c.update(1.,10.);c.update(1.,10.3)
 assert not c.alive(10.3,.25)

def test_clock_rewind_never_makes_old_sensor_ages_negative():
 c=RuntimeClock();c.update(2.,10.);assert c.update(1.,10.1)==2.
 assert c.update(1.1,10.2)==2.
 assert not c.alive(10.3,.25)
 assert abs(c.update(2.1,10.4)-2.1)<1e-9


def test_alternating_old_clock_does_not_accumulate_fake_age():
 c=RuntimeClock();c.update(190.,10.)
 for i in range(1000):
  c.update(53.+i*.001,10.+i*.001)
  assert abs(c.update(190.+i*.001,10.+i*.001)-190.-i*.001)<1e-8
 assert c.elapsed<192.


def test_old_clock_stream_cannot_keep_dead_current_clock_alive():
 c=RuntimeClock();c.update(190.,10.)
 for i in range(100):c.update(53.+i*.001,10.+i*.01)
 assert c.elapsed==190.
 assert not c.alive(11.,.25)


def test_many_laps_preserve_sensor_age_without_clock_drift():
 c=RuntimeClock()
 for i in range(30000):
  stamp=1.+i*.02
  now=c.update(stamp,100.+i*.03)
  c.update(max(0.,stamp-139.),100.+i*.03)
  assert abs((now-(stamp-.05))-.05)<1e-8

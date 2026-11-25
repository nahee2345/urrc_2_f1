"""Production launch must keep publishing when its terminal stops consuming output."""
import json,subprocess,sys
from pathlib import Path


def test_stalled_terminal_preserves_red_stop_and_continuous_start(tmp_path):
    probe=Path(__file__).with_name('probe_launch_backpressure.py')
    result=subprocess.run([sys.executable,str(probe),str(tmp_path)],capture_output=True,text=True,timeout=45)
    assert result.returncode==0,result.stdout+result.stderr
    data=json.loads((tmp_path/'result.json').read_text())
    red=data['red_phase']
    assert red['max_speed_last_sec']==0.,red
    unread=data['unread']
    assert unread['launch_exit'] is None
    assert unread['speed_messages_last_sec']>=35,unread
    assert unread['last_speed_age_sec']<.25,unread
    assert unread['min_speed_last_sec']>1.,unread
    assert not unread['last_trace']['stop'],unread
    assert unread['console_unread_bytes']<4096,unread
    assert data['drained']['min_speed_last_sec']>1.,data

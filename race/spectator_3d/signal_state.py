"""Read confirmed events from the original Gazebo signal script, never send commands."""
import re
RED = re.compile(r'^RED ([1-5])/5 at [0-9.]+s$')
OFF = re.compile(r'^LIGHTS OUT first_on_to_off=[0-9.]+s model_remove=single_request$')

def parse_signal(text):
    count=0; phase='waiting'
    for line in text.splitlines():
        line=line.strip()
        match=RED.fullmatch(line)
        if match and phase in ('waiting','countdown'):
            n=int(match.group(1))
            if n==count+1: count=n;phase='countdown'
        elif OFF.fullmatch(line) and count==5 and phase=='countdown':
            phase='started';count=0
        elif '[URRC SIGNAL ERROR]' in line and phase!='started':
            phase='error';count=0
    return phase,count

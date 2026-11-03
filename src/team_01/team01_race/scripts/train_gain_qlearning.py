#!/usr/bin/env python3
"""Offline tabular Q-learning; a toy vehicle is NOT a Gazebo PASS."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from team01_race.rl_tuner import ACTIONS,state


def train(episodes=1200,seed=11):
    rng=np.random.default_rng(seed)
    q={}
    def row(k):
        if k not in q:q[k]=[0.]*len(ACTIONS)
        return q[k]
    for ep in range(episodes):
        amplitude=float(rng.uniform(0.,.26))
        period=float(rng.uniform(9.,20.))
        speed=float(rng.uniform(2.,5.))
        x=0.;y=float(rng.uniform(-.28,.28));yaw=0.;prev=0.
        def center(t):return amplitude*math.sin(2*math.pi*t/period)
        for _ in range(65):
            cross=center(x+.85)-y-math.sin(yaw)*.85
            heading=math.atan((center(x+1.2)-center(x+.5))/.7)-yaw
            k=state(cross,heading,speed)
            values=row(k)
            eps=max(.03,.45*(1.-ep/max(1,episodes)))
            ix=int(rng.integers(len(ACTIONS))) if rng.random()<eps else int(np.argmax(values))
            look,kh,kc=ACTIONS[ix]
            slope=(center(x+look+.35)-center(x+look-.35))/.7
            steer=float(np.clip(kh*(math.atan(slope)-yaw)+
                math.atan2(kc*cross,speed+1.5),-.45,.45))
            dt=.1
            yaw+=speed*math.tan(steer)/.34*dt
            x+=speed*math.cos(yaw)*dt
            y+=speed*math.sin(yaw)*dt
            error=y-center(x)
            reward=speed*dt-4.*error**2-.1*steer**2-.4*(steer-prev)**2
            prev=steer
            done=abs(error)>.85
            if done:reward-=50.
            nxt=state(center(x+.85)-y,math.atan(
                (center(x+1.2)-center(x+.5))/.7)-yaw,speed)
            values[ix]+=.12*(reward+(.92*max(row(nxt)) if not done else 0.)-values[ix])
            if done:break
    return {'schema':1,'episodes':episodes,'seed':seed,
        'model':'toy kinematic bicycle; unverified in Gazebo',
        'actions':ACTIONS,'q':q}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--episodes',type=int,default=1200)
    parser.add_argument('--seed',type=int,default=11)
    parser.add_argument('--out',default='rl_gain_policy.json')
    args=parser.parse_args()
    if not 1<=args.episodes<=50000:parser.error('invalid episodes')
    Path(args.out).write_text(json.dumps(train(args.episodes,args.seed),indent=2),
                              encoding='utf-8')
    print('Experimental offline policy:',args.out)


if __name__=='__main__':main()

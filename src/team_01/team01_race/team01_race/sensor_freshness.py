"""Use Gazebo clock for feedback age, with separate wall-clock disconnect guard."""


def feedback_fresh(sim_now,stamp,wall_now,last_wall,
                   max_sim_age=.35,max_wall_age=2.5):
    if last_wall<=0 or wall_now<last_wall or wall_now-last_wall>max_wall_age:
        return False
    if sim_now>0 and stamp>0:
        return 0.<=sim_now-stamp<=max_sim_age
    return wall_now-last_wall<=.85

"""Command watchdog based on physics seconds, with a separate wall-time guard."""


def fresh_command(now_sim,then_sim,now_wall,then_wall,
                  max_sim_age=.32,max_wall_age=2.5,preclock_wall_age=.30):
    wall_age=float(now_wall)-float(then_wall)
    if then_wall<=0 or wall_age<0:return False
    if now_sim>0 and then_sim>0:
        sim_age=float(now_sim)-float(then_sim)
        # ROS clock reset or stopped publisher fails closed.
        return 0.<=sim_age<=max_sim_age and wall_age<=max_wall_age
    # Never extend stale commands if /clock itself is absent.
    return wall_age<=preclock_wall_age


def command_age(now_sim,then_sim,now_wall,then_wall):
    if then_wall<=0:return None,None
    sim_age=(max(0.,now_sim-then_sim) if now_sim>0 and then_sim>0 else None)
    return sim_age,max(0.,now_wall-then_wall)

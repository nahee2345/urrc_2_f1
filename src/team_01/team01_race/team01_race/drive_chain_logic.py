"""Pure drive-chain stage classification without ROS dependencies."""
def classify_drive(cmd, adapter, speed, joint, odom_age):
    if cmd is None:return 'NO_CONTROLLER_COMMAND'
    if adapter is None:return 'NO_ADAPTER_DIAGNOSTICS'
    if not adapter.get('send_enabled',False):
        return 'ADAPTER_STOP_'+str(adapter.get('stop_reason','UNKNOWN'))
    if speed is None or odom_age>1.:return 'ODOMETRY_MISSING_OR_STALE'
    if cmd>.4 and adapter.get('sent_linear_mps',0.)>.4 and speed<.04:
        return 'COMMAND_SENT_NO_MEASURED_MOTION'
    if joint is None:return 'STEERING_JOINT_UNKNOWN'
    return 'CHAIN_OBSERVED'


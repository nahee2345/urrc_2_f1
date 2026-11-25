import importlib.util
from pathlib import Path
from unittest.mock import Mock,patch
import pytest
from launch import LaunchContext
spec=importlib.util.spec_from_file_location('clock_sensor_launch',Path(__file__).parents[1]/'launch/sensors.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

@pytest.fixture(autouse=True)
def local_logs(tmp_path,monkeypatch):
 monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'ros'))

def context(world='',enabled='true'):
 c=LaunchContext();c.launch_configurations.update(world_name=world,clock_topic='/team_01/clock',bridge_clock=enabled)
 c.environment['GZ_PARTITION']='clock_test_partition';return c

def test_unique_world_is_discovered_in_selected_partition():
 c=context()
 with patch.object(module.subprocess,'run',return_value=Mock(returncode=0,stdout='/world/random_course/scene/info\n')) as run,patch.object(module,'Node',side_effect=lambda **kw:kw):
  node=module.clock_bridge(c)[0]
 assert node['arguments']==['/world/random_course/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
 assert node['remappings'][0]==('/world/random_course/clock','/team_01/clock')
 assert run.call_args.kwargs['env']['GZ_PARTITION']=='clock_test_partition'

@pytest.mark.parametrize('output',['','/world/a/scene/info\n/world/b/scene/info\n'])
def test_no_unique_world_never_subscribes_to_global_clock(output):
 with patch.object(module.subprocess,'run',return_value=Mock(returncode=0,stdout=output)):
  with pytest.raises(RuntimeError):module.clock_bridge(context())

def test_explicit_event_world_needs_no_discovery():
 with patch.object(module.subprocess,'run') as run,patch.object(module,'Node',side_effect=lambda **kw:kw):
  assert module.clock_bridge(context('event_world'))[0]['arguments'][0].startswith('/world/event_world/clock@')
 run.assert_not_called()

def test_autonomy_can_disable_clock_owner():
 with patch.object(module.subprocess,'run') as run:
  assert module.clock_bridge(context(enabled='false'))==[]
 run.assert_not_called()


def test_explicit_partition_applies_to_discovery_and_child_bridges():
 c=context();c.launch_configurations['gz_partition']='portable_connection'
 for action in module.transport_partition(c):action.execute(c)
 assert c.environment['GZ_PARTITION']=='portable_connection'
 with patch.object(module.subprocess,'run',return_value=Mock(returncode=0,stdout='/world/renamed_course/scene/info\n')) as run,patch.object(module,'Node',side_effect=lambda **kw:kw):
  node=module.clock_bridge(c)[0]
 assert run.call_args.kwargs['env']['GZ_PARTITION']=='portable_connection'
 assert node['arguments'][0].startswith('/world/renamed_course/clock@')


def test_empty_partition_override_preserves_operator_environment():
 c=context();c.launch_configurations['gz_partition']=''
 assert module.transport_partition(c)==[]
 assert c.environment['GZ_PARTITION']=='clock_test_partition'


def test_partition_error_identifies_selected_connection_without_global_clock_fallback():
 with patch.object(module.subprocess,'run',return_value=Mock(returncode=0,stdout='')):
  with pytest.raises(RuntimeError,match='GZ_PARTITION=clock_test_partition'):
   module.clock_bridge(context())

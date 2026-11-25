import xml.etree.ElementTree as E
from pathlib import Path

def add(parent, tag, value=None, **attrs):
    e = E.SubElement(parent, tag, attrs)
    if value is not None:
        e.text = str(value)
    return e

def inertia(link, mass, ix, iy, iz):
    i = add(link, 'inertial')
    add(i, 'mass', mass)
    q = add(i, 'inertia')
    for k,v in dict(ixx=ix,iyy=iy,izz=iz,
                    ixy=0,ixz=0,iyz=0).items():
        add(q,k,v)

def shape(link, kind, values, pose, color):
    for t in ['collision','visual']:
        n = add(link,t,name=t)
        add(n,'pose',pose)
        g = add(add(n,'geometry'),kind)
        for k,v in values.items():
            add(g,k,v)
        if t == 'visual':
            m = add(n,'material')
            add(m,'ambient',color)
            add(m,'diffuse',color)

def joint(name,parent,child,axis,steering=False):
    j=add(model,'joint',name=name,type='revolute')
    add(j,'parent',parent)
    add(j,'child',child)
    a=add(j,'axis')
    add(a,'xyz',axis)
    if steering:
        l=add(a,'limit')
        add(l,'lower',-.70)
        add(l,'upper',.70)
        add(l,'effort',10)
        add(l,'velocity',15)
    else:
        l=add(a,'limit')
        add(l,'lower',-1e16)
        add(l,'upper',1e16)
    add(add(a,'dynamics'),'damping',.005)

sdf=E.Element('sdf',version='1.9')
model=add(sdf,'model',name='team_05')
add(model,'static','false')
add(model,'self_collide','false')

base=add(model,'link',name='base_link')
inertia(base,1.7,.004,.020,.022)
shape(base,'box',{'size':'.36 .15 .045'},
      '0 0 .0175 0 0 0','0.1 0.3 0.8 1')

for side,sign in [('left',1),('right',-1)]:
    y=sign*.095
    name=f'front_{side}_steering'
    knuckle=add(model,'link',name=name)
    add(knuckle,'pose',f'.14 {y} -.005 0 0 0')
    inertia(knuckle,.04,.0001,.0001,.0001)
    joint(name+'_joint','base_link',name,'0 0 1',True)

    for axle,x in [('front',.14),('rear',-.14)]:
        wheel=f'{axle}_{side}_wheel'
        w=add(model,'link',name=wheel)
        add(w,'pose',f'{x} {y} -.005 0 0 0')
        inertia(w,.06,.00005,.00005,.00005)
        shape(w,'cylinder',{'radius':'.035','length':'.022'},
              '0 0 0 1.5707963 0 0','0.03 0.03 0.03 1')
        parent=name if axle=='front' else 'base_link'
        joint(f'{axle}_{side}_joint',parent,wheel,'0 1 0')

cam=add(base,'sensor',name='front_camera',type='camera')
add(cam,'pose','.16 0 .06 0 -.20 0')
add(cam,'topic','/team_05/camera/image_raw')
add(cam,'always_on','true')
add(cam,'update_rate',10)
c=add(cam,'camera')
add(c,'horizontal_fov',1.4)
im=add(c,'image')
add(im,'width',320)
add(im,'height',240)
add(im,'format','R8G8B8')
cl=add(c,'clip')
add(cl,'near',.02)
add(cl,'far',30)

lidar=add(base,'sensor',name='lidar',type='gpu_lidar')
add(lidar,'pose','0 0 .07 0 0 0')
add(lidar,'topic','/team_05/scan')
add(lidar,'always_on','true')
add(lidar,'update_rate',15)
ld=add(lidar,'lidar')
h=add(add(ld,'scan'),'horizontal')
add(h,'samples',360)
add(h,'resolution',1)
add(h,'min_angle',-3.14159)
add(h,'max_angle',3.14159)
r=add(ld,'range')
add(r,'min',.04)
add(r,'max',10)
add(r,'resolution',.02)

p=add(model,'plugin',
      filename='gz-sim-ackermann-steering-system',
      name='gz::sim::systems::AckermannSteering')

for k,v in [
    ('left_joint','front_left_joint'),
    ('left_joint','rear_left_joint'),
    ('right_joint','front_right_joint'),
    ('right_joint','rear_right_joint'),
    ('left_steering_joint','front_left_steering_joint'),
    ('right_steering_joint','front_right_steering_joint'),
    ('wheel_base',.28),
    ('wheel_separation',.19),
    ('kingpin_width',.19),
    ('wheel_radius',.035),
    ('steering_limit',.57),
    ('steer_p_gain',8),
    ('min_velocity',0),
    ('max_velocity',1.2),
    ('min_acceleration',-3.0),
    ('max_acceleration',.7),
    ('odom_publish_frequency',30),
    ('topic','/model/team_05/cmd_vel')
]:
    add(p,k,v)

E.indent(sdf)
out=Path(__file__).parent/'models/car/model.sdf'
E.ElementTree(sdf).write(out,encoding='utf-8',
                        xml_declaration=True)
print('VEHICLE MODEL CREATED')

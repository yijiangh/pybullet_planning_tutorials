import os
import sys
import numpy as np
import argparse
from termcolor import cprint

from pybullet_planning import BASE_LINK, RED, BLUE, GREEN
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui, apply_alpha
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert, get_distance
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion
from pybullet_planning import dump_world, set_pose
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import pairwise_collision, pairwise_collision_info, draw_collision_diagnosis, body_collision_info

HERE = os.path.dirname(__file__)
UR_ROBOT_URDF = os.path.join(HERE, 'data', 'universal_robot', 'ur_description', 'urdf', 'ur5.urdf')
RFL_ROBOT_URDF = os.path.join(HERE, 'data', 'eth_rfl_robot', 'eth_rfl_description', 'urdf', 'eth_rfl.urdf')

MIT_WORKSPACE_PATH = os.path.join(HERE, 'data', 'mit_3-412_workspace', 'urdf', 'mit_3-412_workspace.urdf')
EE_PATH = os.path.join(HERE, 'data', 'dms_bar_gripper.obj')
ATTACH_OBJ_PATH = os.path.join(HERE, 'data', 'bar_attachment.obj')
OBSTACLE_OBJ_PATH = os.path.join(HERE, 'data', 'box_obstacle.obj')
DUCK_OBJ_PATH = os.path.join(HERE, 'data', 'duck.obj')
ASSEMBLY_OBJ_DIR = os.path.join(HERE, 'data', 'kathrin_assembly')

TUTORIALS = {'DUCK', 'UR', 'RFL', 'Assembly'}

def assembly_demo(viewer=True, debug=False):
    connect(use_gui=viewer)

    # * zoom in so we can see it, this is optional
    camera_base_pt = (0,0,0)
    camera_pt = np.array(camera_base_pt) + np.array([0.1, -0.1, 0.1])
    set_camera_pose(tuple(camera_pt), camera_base_pt)

    # * a simple example showing two artificial beam colliding at the interface
    # compare this "brep-like" approach to the mesh approach below,
    # here we have less numerical error at the interface
    cprint('='*10)
    cprint('Checking collisions between two created boxes.', 'cyan')
    w = 0.05
    l = 0.5
    h = 0.01
    eps_list = [-1e-14, 0, 1e-3, 1e-6, 1e-14]
    b0_body = create_box(w, l, h, color=apply_alpha(RED, 0.5))
    b1_body = create_box(w, l, h, color=apply_alpha(BLUE, 0.5))
    for eps in eps_list:
        print('='*5)
        cprint('Pertubation distance: {}'.format(eps), 'yellow')
        # the beam on the right is perturbed towards left by eps
        set_pose(b0_body, Pose(point=[0,l/2-eps,0]))
        set_pose(b1_body, Pose(point=[0,-l/2,0]))

        is_collided = pairwise_collision(b0_body, b1_body)
        if is_collided:
            cprint('Collision detected! The penetration depth shown below should be close to eps={}'.format(eps), 'red')
            cr = pairwise_collision_info(b0_body, b1_body)
            draw_collision_diagnosis(cr, focus_camera=True)
        else:
            cprint('No collision detected between b0 and b1.', 'green')
        assert eps <= 0 or is_collided

    # * now let's load Kathrin's beams and check pairwise collision among them
    # notice that in pybullet everything is in METER, and each beam is configured at its pose in the world
    # already in the obj files (directly exported from Rhino).
    cprint('='*10)
    cprint('Checking collisions among loaded obj elements.', 'cyan')
    beam_from_names = {}
    for i in range(150):
        beam_path = os.path.join(ASSEMBLY_OBJ_DIR, 'element_{}.obj'.format(i))
        beam_from_names[i] = create_obj(beam_path, scale=1, color=apply_alpha(BLUE, 0.2))

    collided_pairs = set()
    for i in range(150):
        for j in range(i+1, 150):
            if (i, j) not in collided_pairs and (j, i) not in collided_pairs:
                if pairwise_collision(beam_from_names[i], beam_from_names[j]):
                    # using meshes (comparing to the `create_box` approach above) induces numerical errors
                    # in our case here the touching faces will be checked as collisions.
                    # Thus, we have to query the getClosestPoint info and use the penetration depth to filter these "touching" cases out
                    # reference for these info: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.cb0co8y2vuvc
                    cr = body_collision_info(beam_from_names[i], beam_from_names[j])
                    penetration_depth = get_distance(cr[0][5], cr[0][6])
                    # this numner `3e-3` below is based on some manual experiement, 
                    # might need to be changed accordingly for specific scales and input models
                    if penetration_depth > 3e-3:
                        cprint('({}-{}) colliding : penetrating depth {:.4E}'.format(i,j, penetration_depth), 'red')
                        collided_pairs.add((i,j))
                        if debug:
                            draw_collision_diagnosis(cr, focus_camera=False)

    # all the colliding information is in `collided_pairs`
    cprint('In total we\'ve found {} collided element pairs!'.format(len(collided_pairs)), 'green')
    wait_if_gui('End.')


def duck_demo(viewer=True):
    connect(use_gui=viewer)

    # * in this case, the duck obj is in millimeter, we scale it into meter
    duck_body = create_obj(DUCK_OBJ_PATH, scale=1e-3, color=(0,0,1,1))

    # * zoom in so we can see it, this is optional
    camera_base_pt = (0,0,0)
    camera_pt = np.array(camera_base_pt) + np.array([0.1, -0.1, 0.1])
    set_camera_pose(tuple(camera_pt), camera_base_pt)

    cprint('hello duck! <ctrl+left mouse> to pan', 'green')
    wait_for_user()

def rfl_demo(viewer=True):
    arm='right'

    connect(use_gui=viewer)
    robot = load_pybullet(RFL_ROBOT_URDF, fixed_base=True)
    set_camera(yaw=-90, pitch=-40, distance=10, target_position=(0, 7.5, 0))

    cprint('hello RFL! <ctrl+left mouse> to pan', 'green')
    wait_for_user()

    # create a box to be picked up
    # see: https://pybullet-planning.readthedocs.io/en/latest/reference/generated/pybullet_planning.interfaces.env_manager.create_box.html#pybullet_planning.interfaces.env_manager.create_box
    block = create_box(0.059, 20., 0.089)
    block_x = 10
    block_y = 5.
    block_z = 1.5
    set_pose(block, Pose(Point(x=block_x, y=block_y, z=block_z), Euler(yaw=np.pi/2)))

    arm_tag = arm[0]
    arm_joint_names = ['gantry_x_joint',
                       '{}_gantry_y_joint'.format(arm_tag),
                       '{}_gantry_z_joint'.format(arm_tag)] + \
                      ['{}_robot_joint_{}'.format(arm_tag, i+1) for i in range(6)]
    arm_joints = joints_from_names(robot, arm_joint_names)
    # * if a subset of joints is used, use:
    # arm_joints = joints_from_names(robot, arm_joint_names[1:]) # this will disable the gantry-x joint
    cprint('Used joints: {}'.format(get_joint_names(robot, arm_joints)), 'yellow')

    # * get a joint configuration sample function:
    # it separately sample each joint value within the feasible range
    sample_fn = get_sample_fn(robot, arm_joints)

    cprint('Randomly sample robot configuration and set them! (no collision check is performed)', 'blue')
    wait_for_user()
    for i in range(5):
        # randomly sample a joint conf
        sampled_conf = sample_fn()
        set_joint_positions(robot, arm_joints, sampled_conf)
        cprint('#{} | Conf sampeld: {}'.format(i, sampled_conf), 'green')
        wait_for_user()

    # * now let's plan a trajectory
    # we use y-z-arm 6 joint all together here
    cprint('Randomly sample robot start/end configuration and comptue a motion plan! (no self-collision check is performed)', 'blue')
    print('Disabled collision links needs to be given (can be parsed from a SRDF via compas_fab)')
    for _ in range(5):
        print('='*10)

        q1 = list(sample_fn())
        # intentionly make the robot needs to cross the collision object
        # let it start from the right side
        q1[0] = 0.
        q1[1] = 0

        set_joint_positions(robot, arm_joints, q1)
        cprint('Sampled start conf: {}'.format(q1), 'cyan')
        wait_for_user()

        # let it ends at the left side
        q2 = list(sample_fn())
        q2[0] = 0.5
        q2[1] = 7.0
        cprint('Sampled end conf: {}'.format(q2), 'cyan')

        path = plan_joint_motion(robot, arm_joints, q2, obstacles=[block], self_collisions=False,
            custom_limits={arm_joints[0]:[0.0, 1.2]})
        if path is None:
            cprint('no plan found', 'red')
            continue
        else:
            wait_for_user('a motion plan is found! Press enter to start simulating!')

        # adjusting this number will adjust the simulation speed
        time_step = 0.03
        for conf in path:
            set_joint_positions(robot, arm_joints, conf)
            wait_for_duration(time_step)

def ur_demo(viewer=True, robot_path=UR_ROBOT_URDF, ee_path=EE_PATH, \
    workspace_path=MIT_WORKSPACE_PATH, attach_obj_path=ATTACH_OBJ_PATH, obstacle_obj_path=OBSTACLE_OBJ_PATH):

    # * this will create the pybullet GUI
    # setting viewers=False will enter GUI-free mode
    connect(use_gui=viewer)
    cprint("Welcome to pybullet! <Ctrl+left mouse> to rotate, <Ctrl+middle mouse> to move the camera, <Ctrl+right mouse> to zoom", 'green')
    cprint('But the space is empty, let\'s load our robot!', 'yellow')
    # wait_for_user is your friend! It will pause the console, but having a separate thread keeping
    # the GUI running so you can rotate and see
    wait_for_user()

    # * This is how we load a robot from a URDF, a workspace from a URDF, or simply a mesh object from an obj file
    # Notice that the pybullet uses *METER* by default, make sure you scale things properly!
    robot = load_pybullet(robot_path, fixed_base=True)
    workspace = load_pybullet(workspace_path, fixed_base=True)
    ee_body = create_obj(ee_path)

    # this will print all the bodies' information in your console
    dump_world()
    cprint('You just loaded a robot, a workspace (with many static objects as its link, I modeled our good old MIT 3-412 shop here), '
           + 'and an end effector (it\'s inside the robot base now)', 'green')
    wait_for_user()

    # * adjust camera pose (optional)
    # has_gui checks if the GUI mode is enabled
    if has_gui():
        camera_base_pt = (0,0,0)
        camera_pt = np.array(camera_base_pt) + np.array([1, -0.5, 0.5])
        set_camera_pose(tuple(camera_pt), camera_base_pt)

    # * each joint of the robot are assigned with an integer in pybullet
    ik_joints = get_movable_joints(robot)
    ik_joint_names = get_joint_names(robot, ik_joints)
    cprint('Joint {} \ncorresponds to:\n{}'.format(ik_joints, ik_joint_names), 'green')
    robot_start_conf = [0,-1.65715,1.71108,-1.62348,0,0]
    cprint("This is before updating pose", 'yellow')
    wait_for_user()
    # * set joint configuration, the robot's pose will be updated
    set_joint_positions(robot, ik_joints, robot_start_conf)
    cprint("This is after set joint pose: {}".format(robot_start_conf), 'green')
    wait_for_user()

    tool_attach_link_name = 'ee_link'
    tool_attach_link = link_from_name(robot, tool_attach_link_name)

    # * attach the end effector
    ee_link_pose = get_link_pose(robot, tool_attach_link)
    set_pose(ee_body, ee_link_pose)
    ee_attach = create_attachment(robot, tool_attach_link, ee_body)
    # we need to call "assign()" to update the attachment to the current end effector pose
    ee_attach.assign()

    # let's load a bar element (obj) and a box (pybullet primitive shape) into the world
    attached_bar_body = create_obj(attach_obj_path)
    box_body = create_obj(obstacle_obj_path)
    cprint('We loaded a box to our scene!', 'green')
    wait_for_user()

    # * attach the bar
    ee_link_from_tcp = Pose(point=(0.094, 0, 0))
    set_pose(attached_bar_body, multiply(ee_link_pose, ee_link_from_tcp))
    bar_attach = create_attachment(robot, tool_attach_link, attached_bar_body)
    bar_attach.assign()
    cprint('The bar element is attached to the robot', 'green')
    wait_for_user()

    attachments = [ee_attach, bar_attach]

    # * Let's do some collision checking
    # * specify disabled link pairs for collision checking (because they are adjacent / impossible to collide)
    # link name corresponds to the ones specified in the URDF
    # again, each robot link is assigned with an integer index in pybullet
    robot_self_collision_disabled_link_names = [('base_link', 'shoulder_link'),
        ('ee_link', 'wrist_1_link'), ('ee_link', 'wrist_2_link'),
        ('ee_link', 'wrist_3_link'), ('forearm_link', 'upper_arm_link'),
        ('forearm_link', 'wrist_1_link'), ('shoulder_link', 'upper_arm_link'),
        ('wrist_1_link', 'wrist_2_link'), ('wrist_1_link', 'wrist_3_link'),
        ('wrist_2_link', 'wrist_3_link')]
    self_collision_links = get_disabled_collisions(robot, robot_self_collision_disabled_link_names)
    cprint('self_collision_links disabled: {}'.format(self_collision_links), 'yellow')

    extra_disabled_link_names = [('base_link', 'MIT_3412_robot_base_plate'),
                                 ('shoulder_link', 'MIT_3412_robot_base_plate')]
    extra_disabled_collisions = get_body_body_disabled_collisions(robot, workspace, extra_disabled_link_names)
    cprint('extra disabled: {}'.format(extra_disabled_collisions), 'yellow')

    print('#'*10)
    cprint('Checking robot links self-collision', 'green')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links)
    conf = [-1.029744, -1.623156, 2.844887, -0.977384, 1.58825, 0.314159]
    # self collision, this should return true
    # this function will first set the robot's joint configuration (together with the attachment)
    # and check collision
    # notice that turning on the diagnosis flag here will show you where the collision is happening
    cprint('Notice that the diagnosis mode will zoom the camera to where the collision is detected. Move the camera around if you can\'t see what\'s going on there.', 'yellow')
    assert collision_fn(conf, diagnosis=True)

    print('#'*10)
    cprint('Checking robot links - holding attachment self-collision', 'green')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    conf = [0.03500, -2.26900, 2.44300, 1.117, 1.6579, 0.105]
    assert collision_fn(conf, diagnosis=True)
    print('\n')

    print('#'*10)
    cprint('Checking robot links to obstacles (w/o links) collision', 'green')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[box_body],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    conf = [-0.105, -0.76800000000000002, 1.292, -0.61099999999999999, 1.484, 0.105]
    assert collision_fn(conf, diagnosis=True)
    print('\n')

    print('#'*10)
    cprint('Checking robot links to multi-link obstacle collision', 'green')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[workspace],
                                    attachments=[], self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    conf = [-0.17499999999999999, -3.194, 0.33200000000000002, -1.6579999999999999, 1.431, 0.105]
    assert collision_fn(conf, diagnosis=True)
    print('\n')

    print('#'*10)
    cprint('Checking attachment to obstacles (w/o links) collision', 'green')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[workspace, box_body],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    conf = [-2.8100000000000001, -1.484, -1.9199999999999999, -1.6579999999999999, 1.431, 0.105]
    assert collision_fn(conf, diagnosis=True)
    print('\n')

    print('#'*10)
    cprint('Checking attachment to multi-link obstacle collision', 'green')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[workspace],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    conf = [-0.17499999999999999, -2.4780000000000002, 0.33200000000000002, -1.6579999999999999, 1.431, 0.105]
    assert collision_fn(conf, diagnosis=True)
    print('\n')

    # * collision checking exoneration
    print('#'*10)
    print('self-link collision disable')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[],
                                    attachments=[], self_collisions=False)
    conf = [-1.029744, -1.623156, 2.844887, -0.977384, 1.58825, 0.314159]
    assert not collision_fn(conf, diagnosis=True)
    print('\n')

    print('#'*10)
    cprint('robot links to obstacle collision exoneration', 'green')
    cprint('In this example, the first collision function will check collision between the robot and the box, '+
           'but the second collision function will ignore it.', 'yellow')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[box_body],
                                            attachments=[], self_collisions=True,
                                            disabled_collisions=self_collision_links,
                                            )
    collision_fn_disable = get_collision_fn(robot, ik_joints, obstacles=[box_body],
                                            attachments=[], self_collisions=True,
                                            disabled_collisions=self_collision_links,
                                            extra_disabled_collisions=extra_disabled_collisions.union(
                                                [((robot, link_from_name(robot, 'forearm_link')),
                                                  (box_body, BASE_LINK))]),
                                            )
    conf = [-3.2639999999999998, -2.6880000000000002, -0.85499999999999998, -1.536, 3.0369999999999999, -0.070000000000000007]
    assert collision_fn(conf, diagnosis=True)
    assert not collision_fn_disable(conf, diagnosis=True)
    print('\n')

    # * joint value overflow checking & exoneration
    cprint('joint value overflow checking & exoneration', 'green')
    cprint('collision_fn also checks for robot\'s joint limit as well. We can also exonerate it by passing custom_limits into the collision_fn', 'yellow')
    def get_custom_limits_from_name(robot, joint_limits):
        return {joint_from_name(robot, joint): limits
                for joint, limits in joint_limits.items()}
    custom_limits = get_custom_limits_from_name(robot, {'shoulder_pan_joint':(-7.9, 0), 'elbow_joint':(-8.0, 0)})
    collision_fn = get_collision_fn(robot, ik_joints)
    collision_fn_disable = get_collision_fn(robot, ik_joints, custom_limits=custom_limits)
    conf = [-7.8450000000000002, -2.1469999999999998, -7.99, -0.92500000000000004, 1.78, 0.105]
    assert collision_fn(conf, diagnosis=True)
    assert not collision_fn_disable(conf, diagnosis=True)
    print('\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-nv', '--noviewer', action='store_true', help='Enables the viewer during planning, default True')
    parser.add_argument('-d', '--demo', default='UR', choices=TUTORIALS, \
        help='The name of the demo')
    parser.add_argument('-db', '--debug', action='store_true', help='Debug mode')
    args = parser.parse_args()
    print('Arguments:', args)

    if args.demo == 'UR':
        ur_demo(viewer=not args.noviewer)
    elif args.demo == 'DUCK':
        duck_demo(viewer=not args.noviewer)
    elif args.demo == 'RFL':
        rfl_demo(viewer=not args.noviewer)
    elif args.demo == 'Assembly':
        assembly_demo(viewer=not args.noviewer, debug=args.debug)
    else:
        raise NotImplementedError()


if __name__ == '__main__':
    main()

from __future__ import division, print_function

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, JointTrajectory
import dynamixel
import itertools
import numpy as np
import reflexxes
import rospy
# import yaml
import actionlib


class EUAController(object):
    def __init__(self):
        self.loop_rate_hz = 100.0

        # Order the supplied device ID's according to this sequence of joint
        # names which corresponds to the kinematic chain
        self.joint_names = ('roll', 'wrist', 'jaw1', 'jaw2')
        self.joint_device_mapping = rospy.get_param('~joint_device_mapping')
        self.joint_device_mapping.update({v: k for k, v in self.joint_device_mapping.iteritems()})  # reverse mapping
        self.device_ids = tuple(self.joint_device_mapping[n] for n in self.joint_names)  # same order as joint_names

        # Find Dynamixel servo devices
        self.chain = dynamixel.Chain(rospy.get_param('~port'), rospy.get_param('~baud_rate'), self.device_ids)

        if not self.chain.devices:
            raise RuntimeError("No devices found")

        for dev in self.chain.devices:
            rospy.loginfo("{} -> '{}'".format(dev, self.joint_device_mapping[dev.id]))
            # print(yaml.dump(dev.dump_cc(), default_flow_style=False))

        if len(self.joint_names) != len(self.chain.devices) != 4:
            raise RuntimeError("Not all devices found")

        # Transmission/coupling matrix K
        t0 = 1.5     # motor-to-disc transmission (for all motors)
        t1 = 1.5632  # roll disc-to-joint transmission
        t2 = 1.0186  # wrist disc-to-joint transmission
        t3 = 1.2177  # grasper jaw disc-to-joint transmission (the same for both jaws)
        w = 0.6089   # coupling between wrist and grasper jaws (the same for both jaws)
        self.K = t0 * np.array([
            [  t1,  0,     0,   0   ],
            [  0,   t2,    0,   0   ],
            [  0,  -t2*w,  t3,  0   ],
            [  0,   t2*w,  0,  -t3  ],
        ])

        # Calibration offset
        self.servo_calibration_offset = np.array(rospy.get_param('/eua/servo_calibration_offset', [0, 0, 0, 0]))
        rospy.loginfo("Calibration offset (servo): {}".format(self.servo_calibration_offset))

        # Trajectory generation
        self.tg = reflexxes.extra.PositionTrajectoryGenerator(
            4,                        # number of DOF
            1.0 / self.loop_rate_hz,  # period
            [2 * np.pi] * 4,          # max vel.
            [8 * np.pi] * 4,          # max acc.
            [80 * np.pi] * 4          # max jerk
        )
        self.tg.flags.SynchronizationBehavior = reflexxes.RMLPositionFlags.NO_SYNCHRONIZATION

        servo_state = self.read_servo_state()
        joint_state = self.compute_joint_state(servo_state)

        self.tg.current_position = joint_state.position
        self.tg.current_velocity.fill(0)
        self.tg.target_position = joint_state.position
        self.tg.target_velocity.fill(0)

        self.trajectory = None
        self.path_itr = None

        self.pub_joint = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.pub_servo = rospy.Publisher('servo_states', JointState, queue_size=1)

        self.subscribers = [
            rospy.Subscriber('command', JointTrajectory, controller.on_path_command),
            # rospy.Subscriber('stop_joint', JointState, controller.stop_trajectory, lambda m: self.stop_trajectory(m.name))
        ]

        self.follow_path_action_server = actionlib.SimpleActionServer(
            'follow_joint_trajectory',
            FollowJointTrajectoryAction,
            auto_start=False)
        self.follow_path_action_server.register_goal_callback()
        self.follow_path_action_server.register_preempt_callback()

    def read_servo_state(self):
        s = JointState(position=np.empty(4), velocity=np.empty(4), effort=np.empty(4))
        s.header.stamp = rospy.Time.now()

        for i, dev in enumerate(self.chain.devices):
            if isinstance(dev, dynamixel.device.AX12):
                d = dev.read_params(['present_position', 'present_speed', 'present_load'])
                s.position[i] = d['present_position']
                s.velocity[i] = d['present_speed']
                s.effort[i] = d['present_load']
            elif isinstance(dev, dynamixel.device.MX28_2):
                d = dev.read_params(['present_load', 'present_velocity', 'present_position'])
                s.position[i] = d['present_position']
                s.velocity[i] = d['present_velocity']
                s.effort[i] = d['present_load']
            else:
                raise RuntimeError("Unknown device type '{}' in chain".format(type(dev)))

        # Subtract calibration offset from servo positions
        s.position -= self.servo_calibration_offset

        return s

    def compute_joint_state(self, servo_state):
        s = JointState()
        s.header.stamp = servo_state.header.stamp
        s.name = self.joint_names
        s.position = self.K.dot(servo_state.position)
        s.velocity = self.K.dot(servo_state.velocity)
        s.effort = self.K.dot(servo_state.effort)
        return s

    def stop_trajectory(self):
        self.tg.target_position[:] = self.tg.current_position
        self.tg.target_velocity.fill(0)
        self.trajectory = self.tg.trajectory()

    def init_trajectory(self, joint_names, target_position, target_velocity=None):
        """Set a joint-space trajectory setpoint for servos given in joint_names

        If velocity is 'false', set the target velocity to zero.
        """
        if not joint_names or not position:
            rospy.logwarn("Bad input")
            return

        # Map names to servo IDs
        msg_dev_id = [self.joint_device_mapping[n] for n in joint_names]

        # Cache these values (converted from C++ types)
        tg_target_position = self.tg.target_position
        tg_target_position = self.tg.target_velocity

        for i, dev in enumerate(self.chain.devices):
            if dev.id in msg_dev_id:
                j = msg_dev_id.index(dev.id)
                tg_target_position[i] = target_position[j]
                tg_target_position[i] = target_velocity[j] if target_velocity else 0

        # Iterator to the target state of the trajectory generator
        self.trajectory = self.tg.trajectory()
        # rospy.loginfo("New target: {} ({})".format(target_position, np.linalg.solve(self.K, target_position)))

    def step_trajectory(self):
        try:
            pos_joint_next = self.trajectory.next()[0]
            pos_servo_next = np.linalg.solve(self.K, pos_joint_next)

            # Add calibration offset to servo command
            pos_servo_next += self.servo_calibration_offset

            # Write goal position to each servo (even if there is no trajectory
            # for that particular joint), as some DOFs are coupled
            for dev, pos in itertools.izip(self.chain.devices, pos_servo_next):
                dev.write_param_single('goal_position', pos)
        except StopIteration:
            self.trajectory = None

            if self.path_itr:
                self.step_path()

    def step_path(self):
        try:
            x = next(self.path_itr)
            self.init_trajectory(self.path_names, x.positions, x.velocities)
        except StopIteration:
            self.path_itr = None
            self.follow_path_action_server.set_succeeded()

    def on_path_command(self, msg):
        if self.follow_path_action_server.is_active():
            self.self.follow_path_action_server.set_aborted()

        if not msg.joint_names:
            self.stop_trajectory()
        else:
            self.path_names = msg.joint_names
            self.path_itr = iter(msg.points)
            self.step_path()

    def on_follow_trajectory_goal_preempt(self):
        self.follow_path_action_server.set_preempted()
        self.path_itr = None
        self.stop_trajectory()

    def on_follow_trajectory_goal(self):
        goal = self.follow_path_action_server.accept_new_goal()
        self.path_names = goal.trajectory.joint_names
        self.path_itr = iter(goal.trajectory.points)
        self.step_path()

    def run(self):
        rospy.loginfo("Starting servo loop")
        self.follow_path_action_server.start()
        rate = rospy.Rate(self.loop_rate_hz)

        try:
            while not rospy.is_shutdown():
                servo_state = self.read_servo_state()
                joint_state = self.compute_joint_state(servo_state)

                if self.trajectory:
                    self.step_trajectory()

                self.pub_servo.publish(servo_state)
                self.pub_joint.publish(joint_state)

                rate.sleep()  # FIXME: check that loop rate is kept
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            pass
        finally:
            # disable servo torque
            for dev in self.chain.devices:
                dev.write_param_single('torque_enable', False)

from __future__ import division, print_function

from sensor_msgs.msg import JointState
import dynamixel
import itertools
import numpy as np
import reflexxes
import rospy
# import yaml


class EUAController(object):
    def __init__(self):
        self.loop_rate_hz = 100.0

        # We order the supplied device ID's according to this sequence of joint
        # names which corresponds to the kinematic chain
        prefix = rospy.get_param('~prefix', '')
        self.joint_names = [prefix + n for n in ('roll', 'pitch', 'yaw1', 'yaw2')]
        self.joint_device_mapping = {dev_id: prefix + name for name, dev_id in rospy.get_param('~joint_device_mapping').iteritems()}
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
            [  0,   t2*w, -t3,  0   ],
            [  0,  -t2*w,  0,   t3  ],
        ])

        # Calibration offset (default to zero at servo center position)
        self.servo_limits_lower = np.array([dev.read_param_single('cw_angle_limit') for dev in self.chain.devices])
        self.servo_limits_upper = np.array([dev.read_param_single('ccw_angle_limit') for dev in self.chain.devices])
        servo_center = self.servo_limits_lower + (self.servo_limits_upper - self.servo_limits_lower) / 2
        # self.servo_calibration_offset = np.array(rospy.get_param('/eua/servo_calibration_offset', servo_center.tolist()))  # rosparam doesn't like ndarrays
        self.servo_calibration_offset = np.array(rospy.get_param('/eua/servo_calibration_offset', [0]*4))  # rosparam doesn't like ndarrays
        self.servo_limits_lower -= self.servo_calibration_offset
        self.servo_limits_upper -= self.servo_calibration_offset
        rospy.loginfo("Calibration offset (servo space): {} deg.".format(np.degrees(self.servo_calibration_offset)))

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

        self.pub_joint = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.pub_servo = rospy.Publisher('servo_states', JointState, queue_size=1) if rospy.get_param('~publish_servo_states', False) else None
        self.subscribers = [
            rospy.Subscriber('move_joint', JointState, self.init_trajectory),
            rospy.Subscriber('servo_joint', JointState, self.set_joint_goal_direct),
        ]

        self.state_update_callbacks = []
        self.trajectory_end_callback = None

    def read_servo_state(self):
        s = JointState(position=np.empty(4), velocity=np.empty(4), effort=np.empty(4))
        s.header.stamp = rospy.Time.now()

        for i, dev in enumerate(self.chain.devices):
            s.name.append(str(dev.id))

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

    def write_joint_goal(self, pos_joint_next):
        # Compute corresponding servo angles through coupling/gearing
        pos_servo_next = np.linalg.solve(self.K, pos_joint_next)

        # Add calibration offset to servo command
        pos_servo_next += self.servo_calibration_offset

        # Write goal position to each servo even if there is no trajectory
        # for that particular joint, as some DOFs are coupled (thus servos may
        # have to move to maintain a non-moving joint angle)
        for dev, pos in itertools.izip(self.chain.devices, pos_servo_next):
            dev.write_param_single('goal_position', pos)

    def set_trajgen_target(self, m):
        # Map names to servo IDs
        msg_dev_id = [self.joint_device_mapping[n] for n in m.name]

        # Cache reference to these values while iterating (they are C++ types
        # exposed to Python opaquely)
        target_position = self.tg.target_position
        target_velocity = self.tg.target_velocity

        for i, dev in enumerate(self.chain.devices):
            if dev.id in msg_dev_id:
                j = msg_dev_id.index(dev.id)
                target_position[i] = m.position[j]
                target_velocity[i] = m.velocity[j] if m.velocity else 0

        # rospy.loginfo("New target:\n\t{} (servo)\n\t{} (joint)".format(
        #     np.linalg.solve(self.K, target_position),
        #     np.array(target_position)))

    def set_joint_goal_direct(self, m):
        """Set servo set-point directly without trajectory generation
        """
        if self.trajectory:
            rospy.logwarn("Ignoring direct joint goal: already moving along trajectory")
            return

        if not m.name or not m.position:
            rospy.logwarn("Bad input (name or position fields empty)")
            return

        # Update trajectory generator target so that future trajectories starts
        # from the correct initial position
        self.set_trajgen_target(m)

        # Write the target directly to servos without generating the actual
        # trajectory
        self.write_joint_goal(self.tg.target_position.tolist())

    def init_trajectory(self, m, trajectory_end_callback=None):
        """Set a joint-space trajectory setpoint for the given servos
        """
        if not m.name or not m.position:
            rospy.logwarn("Bad input (name or position fields empty)")
            return

        self.set_trajgen_target(m)
        self.trajectory_end_callback = trajectory_end_callback
        self.trajectory = self.tg.trajectory()

    def step_trajectory(self):
        try:
            pos_joint_next = self.trajectory.next()[0]
            self.write_joint_goal(pos_joint_next)
        except StopIteration:
            self.trajectory = None

            if self.trajectory_end_callback:
                self.trajectory_end_callback()
                self.trajectory_end_callback = None

    def stop_trajectory(self):
        self.tg.target_position[:] = self.tg.current_position
        self.tg.target_velocity.fill(0)
        self.trajectory = self.tg.trajectory()

    def run(self):
        rospy.loginfo("Starting servo loop")
        rate = rospy.Rate(self.loop_rate_hz)

        try:
            while not rospy.is_shutdown():
                servo_state = self.read_servo_state()
                joint_state = self.compute_joint_state(servo_state)

                if self.trajectory:
                    self.step_trajectory()

                if self.pub_servo is not None:
                    self.pub_servo.publish(servo_state)

                self.pub_joint.publish(joint_state)

                for f in self.state_update_callbacks:
                    f(servo_state, joint_state)

                rate.sleep()  # FIXME: check that loop rate is kept
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            pass
        finally:
            # disable servo torque
            for dev in self.chain.devices:
                dev.write_param_single('torque_enable', False)

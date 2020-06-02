from __future__ import division, print_function

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
import itertools
import numpy as np
import reflexxes
import rospy
import threading

# We can still use the node in simulated mode if dynamixel module is not available
try:
    import dynamixel
except ImportError:
    pass


class EUA1Transmission(object):
    def __init__(self):
        t0 = 1.5     # motor-to-disc transmission (for all motors)
        t1 = 1.5632  # roll disc-to-joint transmission
        t2 = 1.0186  # wrist disc-to-joint transmission
        t3 = 1.2177  # grasper jaw disc-to-joint transmission (the same for both jaws)
        w = 0.6089   # coupling between wrist and grasper jaws (the same for both jaws)

        # Transmission/coupling matrix K
        self.K = t0 * np.array([
            [  t1,  0,     0,   0   ],
            [  0,   t2,    0,   0   ],
            [  0,   t2*w, -t3,  0   ],
            [  0,  -t2*w,  0,   t3  ],
        ])

    def servo_to_joint(self, servo_angles):
        return self.K.dot(servo_angles)

    def joint_to_servo(self, joint_angles):
        return np.linalg.solve(self.K, joint_angles)


class EUAController(object):
    def __init__(self):
        self.loop_rate_hz = 100.0
        self.simulated = rospy.get_param('~simulated', False)

        # We order the supplied device ID's according to this sequence of joint
        # names which corresponds to the kinematic chain
        prefix = rospy.get_param('~prefix', '')
        self.joint_names = [prefix + n for n in ('roll', 'pitch', 'yaw1', 'yaw2')]
        self.joint_device_mapping = {dev_id: prefix + name for name, dev_id in rospy.get_param('~joint_device_mapping').iteritems()}
        self.joint_device_mapping.update({v: k for k, v in self.joint_device_mapping.iteritems()})  # reverse mapping
        self.device_ids = tuple(self.joint_device_mapping[n] for n in self.joint_names)  # same order as joint_names

        self.transmission = EUA1Transmission()

        # Trajectory generation
        self.tg = reflexxes.extra.PositionTrajectoryGenerator(
            4,                        # number of DOF
            1.0 / self.loop_rate_hz,  # period
            [2 * np.pi] * 4,          # max vel.
            [8 * np.pi] * 4,          # max acc.
            [80 * np.pi] * 4          # max jerk
        )
        self.tg.flags.SynchronizationBehavior = reflexxes.RMLPositionFlags.NO_SYNCHRONIZATION
        self.trajectory = None

        if self.simulated:
            # Current state of the simulated servos
            self.sim_current_position_servo = np.zeros(4)
            self.sim_current_velocity_servo = np.zeros(4)
            self.sim_current_effort_servo = np.zeros(4)

            # Calibration offset (default to zero at servo center position)
            self.set_servo_calibration_offset(np.zeros(4))

            rospy.loginfo("Starting in simulated mode")
        else:
            # Find Dynamixel servo devices
            self.chain = dynamixel.Chain(rospy.get_param('~port'), rospy.get_param('~baud_rate'), self.device_ids)

            if not self.chain.devices:
                raise RuntimeError("No devices found")

            for dev in self.chain.devices:
                rospy.loginfo("{} -> '{}'".format(dev, self.joint_device_mapping[dev.id]))
                # import yaml
                # print(yaml.dump(dev.dump_cc(), default_flow_style=False))

            if len(self.joint_names) != len(self.chain.devices) != 4:
                raise RuntimeError("Not all devices found")

            self.set_servo_calibration_offset(np.array(rospy.get_param('~servo_calibration_offset', [0]*4)))  # rosparam doesn't like ndarrays

        self.pub_joint = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.pub_servo = rospy.Publisher('servo_states', JointState, queue_size=1) if rospy.get_param('~publish_servo_states', False) else None
        self.subscribers = [
            rospy.Subscriber('move_joint', JointState, self.init_trajectory),
            rospy.Subscriber('servo_joint', JointState, self.set_joint_goal_direct),
            rospy.Service('calibrate', Trigger, self.calibrate)
        ]

        self.state_update_callbacks = [self.publish]

    def reset_trajectory_generator(self):
        servo_state = self.read_servo_state()
        joint_state = self.compute_joint_state(servo_state)

        self.tg.current_position = joint_state.position
        self.tg.current_velocity.fill(0)
        self.tg.target_position = joint_state.position
        self.tg.target_velocity.fill(0)

        self.trajectory = None

    def set_servo_calibration_offset(self, calibration_offset):
        if self.trajectory is not None:
            raise RuntimeError("Cannot change calibration offset while trajectory is active")

        self.servo_calibration_offset = np.asarray(calibration_offset)
        rospy.set_param('~servo_calibration_offset', self.servo_calibration_offset.tolist())
        rospy.loginfo("Set calibration offset (in servo space) to: {} deg.".format(np.degrees(self.servo_calibration_offset)))
        self.reset_trajectory_generator()

    def read_servo_state(self):
        s = JointState(position=np.empty(4), velocity=np.empty(4), effort=np.empty(4))
        s.header.stamp = rospy.Time.now()

        if self.simulated:
            s.name = [str(dev_id) for dev_id in self.device_ids]
            np.copyto(s.position, self.sim_current_position_servo)
            np.copyto(s.velocity, self.sim_current_velocity_servo)
            np.copyto(s.effort, self.sim_current_effort_servo)
        else:
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
        s.position = self.transmission.servo_to_joint(servo_state.position)
        s.velocity = self.transmission.servo_to_joint(servo_state.velocity)
        s.effort = self.transmission.servo_to_joint(servo_state.effort)
        return s

    def write_joint_goal(self, pos_joint_next):
        # Compute corresponding servo angles through coupling/gearing
        pos_servo_next = self.transmission.joint_to_servo(pos_joint_next)

        # Add calibration offset to servo command
        pos_servo_next += self.servo_calibration_offset

        if self.simulated:
            self.sim_current_position_servo = pos_servo_next
        else:
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

        if self.simulated:
            for i, dev_id in enumerate(self.device_ids):
                if dev_id in msg_dev_id:
                    j = msg_dev_id.index(dev_id)
                    target_position[i] = m.position[j]
                    target_velocity[i] = m.velocity[j] if m.velocity else 0
        else:
            for i, dev in enumerate(self.chain.devices):
                if dev.id in msg_dev_id:
                    j = msg_dev_id.index(dev.id)
                    target_position[i] = m.position[j]
                    target_velocity[i] = m.velocity[j] if m.velocity else 0

        # rospy.loginfo("New target:\n\t{} (servo)\n\t{} (joint)".format(
        #     self.transmission.joint_to_servo(target_position),
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

    def init_trajectory(self, m):
        """Set a joint-space trajectory setpoint for the given servos
        """
        if not m.name or not m.position:
            rospy.logwarn("Bad input (name or position fields empty)")
            return

        self.set_trajgen_target(m)
        self.trajectory = self.tg.trajectory()

    def step_trajectory(self):
        try:
            pos_joint_next = self.trajectory.next()[0]
            self.write_joint_goal(pos_joint_next)
        except StopIteration:
            self.trajectory = None

    def stop_trajectory(self):
        self.tg.target_position[:] = self.tg.current_position
        self.tg.target_velocity.fill(0)
        self.trajectory = self.tg.trajectory()

    def publish(self, servo_state, joint_state):
        if self.pub_servo is not None:
            self.pub_servo.publish(servo_state)

        self.pub_joint.publish(joint_state)

    def run(self):
        rospy.loginfo("Starting servo loop")
        rate = rospy.Rate(self.loop_rate_hz)

        try:
            while not rospy.is_shutdown():
                servo_state = self.read_servo_state()
                joint_state = self.compute_joint_state(servo_state)

                if self.trajectory:
                    self.step_trajectory()

                for f in self.state_update_callbacks:
                    f(servo_state, joint_state)

                rate.sleep()  # TODO: check that loop rate is kept
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            pass
        finally:
            if not self.simulated:
                # disable servo torque
                for dev in self.chain.devices:
                    dev.write_param_single('torque_enable', False)

    def calibrate(self, req):
        if self.simulated:
            raise RuntimeError("Will not calibrate in simulated mode")

        calibrator = EUACalibrator(self)
        threading.Thread(target=calibrator.run).start()
        return TriggerResponse(success=True)


class EUACalibrator(object):
    def __init__(self, controller):
        self.c = controller
        self.lock = threading.Lock()
        self.servo_state = None
        self.joint_state = None

    def controller_state_changed(self, servo_state, joint_state):
        with self.lock:
            self.servo_state = servo_state
            self.joint_state = joint_state

    def run(self):
        try:
            # Subscribe to controller state changes
            self.c.state_update_callbacks.append(self.controller_state_changed)

            # Wait for first state update to arrive
            def test_callback_received():
                with self.lock:
                    return self.servo_state is None

            while test_callback_received():
                rospy.sleep(0.1)

            # Save current controller state
            tg_max_velocity = self.c.tg.max_velocity.copy()
            tg_max_acceleration = self.c.tg.max_acceleration.copy()
            tg_max_jerk = self.c.tg.max_jerk.copy()
            calibration_offset = self.c.servo_calibration_offset.copy()

            with self.lock:
                position_joint = self.joint_state.position.tolist()

            self._run()
        except:
            rospy.logerr("Calibration error!")

            # Ramp down current trajectory and wait until movement has stopped
            self.c.stop_trajectory()

            while self.c.trajectory is not None:
                rospy.sleep(0.1)

            # Restore previous calibration offset
            self.c.set_servo_calibration_offset(calibration_offset)

            # Move back to initial position
            self.c.init_trajectory(JointState(name=self.c.joint_names, position=position_joint))

            while self.c.trajectory is not None:
                rospy.sleep(0.1)

            raise
        finally:
            if self.controller_state_changed in self.c.state_update_callbacks:
                self.c.state_update_callbacks.remove(self.controller_state_changed)

            # Reset trajectory generator limits
            self.c.tg.max_velocity = tg_max_velocity
            self.c.tg.max_acceleration = tg_max_acceleration
            self.c.tg.max_jerk = tg_max_jerk

    def _run(self):
        self.c.tg.max_velocity = [np.pi / 4] * 4
        self.c.tg.max_acceleration = [2 * np.pi] * 4
        self.c.tg.max_jerk = [20 * np.pi] * 4

        # Zero current calibration offset
        self.c.set_servo_calibration_offset(np.zeros(4))
        rospy.sleep(0.5)

        # The target position is the servo limits
        servo_limits_lower = [dev.read_param_single('cw_angle_limit') for dev in self.c.chain.devices]
        servo_limits_upper = [dev.read_param_single('ccw_angle_limit') for dev in self.c.chain.devices]
        move_to_upper = [True, True, False, True]
        target_position_servo = [upper - 0.01 if d else lower + 0.01 for d, lower, upper in zip(move_to_upper, servo_limits_lower, servo_limits_upper)]
        target_position = self.c.transmission.servo_to_joint(target_position_servo).tolist()

        # Start trajectory toward servo limits
        self.c.init_trajectory(JointState(name=self.c.joint_names, position=target_position))

        detected_joint_limits_in_servo_space = [None] * 4

        # Detect whether the joint limit is reached for each of the joints
        while any(x is None for x in detected_joint_limits_in_servo_space):
            if self.c.trajectory is None:
                raise RuntimeError("Trajectory reached the end (servo limit) without detecting all joint limits")

            thresholds = [0.18, 0.14, 0.15, 0.15]  # FIXME: hard coded values

            with self.lock:
                servo_state = self.servo_state
                joint_state = self.joint_state

            for i in range(4):
                if ((detected_joint_limits_in_servo_space[i] is None)
                        and ((move_to_upper[i] and servo_state.effort[i] < -thresholds[i])
                            or (not move_to_upper[i] and servo_state.effort[i] > thresholds[i]))):
                    rospy.loginfo("Found '{}' {} joint limit at {:.1f} deg (servo) / {:.1f} deg (joint) - servo load: {:.3f}".format(
                        self.c.joint_names[i],
                        'upper' if move_to_upper[i] else 'lower',
                        round(np.degrees(servo_state.position[i]), 1),
                        round(np.degrees(joint_state.position[i]), 1),
                        round(servo_state.effort[i], 3)))

                    # Save the position at the detected joint limit
                    detected_joint_limits_in_servo_space[i] = servo_state.position[i]

                    # Stop the trajectory (target <- current) for this joint
                    target_position[i] = joint_state.position[i]
                    self.c.init_trajectory(JointState(name=self.c.joint_names, position=target_position))

        # Wait for movement to end
        while self.c.trajectory is not None:
            rospy.sleep(0.1)

        # All joints are now at the hard mechanical limits (known reference
        # points), compute calibration offset
        known_joint_limits = np.radians([275, 95, 115, 115])  # FIXME: hard coded values
        known_joint_limits_in_servo_space = self.c.transmission.joint_to_servo(known_joint_limits)
        self.c.set_servo_calibration_offset(np.array(detected_joint_limits_in_servo_space) - known_joint_limits_in_servo_space)

        # # Move joints to home position
        home_position = np.radians([0, 0, 45, 45])  # FIXME: hard coded values
        self.c.init_trajectory(JointState(name=self.c.joint_names, position=home_position.tolist()))

        while self.c.trajectory is not None:
            rospy.sleep(0.1)

        rospy.loginfo("Calibration done!")

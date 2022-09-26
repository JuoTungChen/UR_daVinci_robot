import sensor_msgs.msg
import std_srvs.srv
import numpy as np
import reflexxes
import rclpy
import rclpy.node
import rclpy.time
import rclpy.qos
import threading
import dynamixel
import time
import array


class MyJointState:
    def __init__(self, stamp=None, name=None, position=None, velocity=None, effort=None):
        self.stamp = stamp if stamp is not None else rclpy.time.Time()
        self.name = name if name is not None else 4 * [None]
        self.position = np.asarray(position) if position is not None else np.zeros(4)
        self.velocity = np.asarray(velocity) if velocity is not None else np.zeros(4)
        self.effort = np.asarray(effort) if effort is not None else np.zeros(4)

    def to_msg(self):
        m = sensor_msgs.msg.JointState()
        m.header.stamp = self.stamp.to_msg()
        m.name = self.name
        m.position = array.array('d', self.position)
        m.velocity = array.array('d', self.velocity)
        m.effort = array.array('d', self.effort)
        return m

    def copy(self):
        m = MyJointState()
        m.stamp = rclpy.Time(nanoseconds=self.stamp.nanoseconds())
        m.name = self.name.copy()
        np.copyto(m.position, self.position)
        np.copyto(m.velocity, self.velocity)
        np.copyto(m.effort, self.effort)
        return m


class EUA1Transmission:
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
        return self.K @ servo_angles

    def joint_to_servo(self, joint_angles):
        return np.linalg.solve(self.K, joint_angles)


class EUA2Transmission:
    def __init__(self):
        t1 = 1.0  # roll disc-to-joint transmission
        t2 = 1.0  # wrist disc-to-joint transmission
        t3 = 1.15  # grasper jaw disc-to-joint transmission (the same for both jaws)
        w = 0.75   # coupling between wrist and grasper jaws (the same for both jaws)

        # Transmission/coupling matrix K
        self.K = np.array([
            [  t1,  0,     0,   0   ],
            [  0,   t2,    0,   0   ],
            [  0,   t2*w,  t3,  0   ],
            [  0,  -t2*w,  0,   t3  ],
        ])

    def servo_to_joint(self, servo_angles):
        return self.K @ servo_angles

    def joint_to_servo(self, joint_angles):
        return np.linalg.solve(self.K, joint_angles)


class EUAController(rclpy.node.Node):
    def __init__(self):
        super().__init__('eua_controller', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.loop_rate_hz = 100.0
        self.simulated = self.get_parameter('simulated').value

        # We order the supplied device ID's according to this sequence of joint
        # names which corresponds to the kinematic chain
        self.eua_type = self.get_parameter('type').value
        prefix = self.get_parameter('prefix').value
        self.joint_names = [prefix + n for n in ('roll', 'pitch', 'yaw1', 'yaw2')]
        self.joint_device_mapping = {p.value: prefix + name for name, p in self.get_parameters_by_prefix('joint_device_mapping').items()}
        self.joint_device_mapping.update({v: k for k, v in self.joint_device_mapping.items()})  # reverse mapping
        self.device_ids = tuple(self.joint_device_mapping[n] for n in self.joint_names)  # same order as joint_names

        if self.eua_type == 1:
            self.transmission = EUA1Transmission()
        elif self.eua_type == 2:
            self.transmission = EUA2Transmission()
        else:
            raise ValueError("Bad EUA type '{}' (must be 1 or 2)".format(self.eua_type))

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

            self.get_logger().info("Starting in simulated mode")
        else:
            port = self.get_parameter('port').value
            baud_rate = self.get_parameter('baud_rate').value

            # setserial [port] low_latency
            import subprocess
            subprocess.call(['setserial', port, 'low_latency'])

            # Find Dynamixel servo devices
            self.chain = dynamixel.Chain(port, baud_rate, self.device_ids)

            if not self.chain.devices:
                raise RuntimeError("No devices found on {}".format(self.chain.io.ser.name))

            for dev in self.chain.devices:
                self.get_logger().info("{} -> '{}'".format(dev, self.joint_device_mapping[dev.id]))
                # import yaml
                # print(yaml.dump(dev.dump_cc(), default_flow_style=False))

            if len(self.joint_names) != len(self.chain.devices) != 4:
                raise RuntimeError("Not all devices found")

            if self.device_ids != tuple(dev.id for dev in self.chain.devices):
                raise RuntimeError("Device IDs mismatch")

            self.set_servo_calibration_offset(np.array(self.declare_parameter('servo_calibration_offset', [0.0, 0.0, 0.0, 0.0]).value))

            for dev in self.chain.devices:
                if isinstance(dev, dynamixel.device.MX28):
                    # set multi-turn mode on MX28's
                    dev.write_param_single('cw_angle_limit', 4095, raw=True)
                    dev.write_param_single('ccw_angle_limit', 4095, raw=True)


        self.pub_joint = self.create_publisher(sensor_msgs.msg.JointState, 'joint_states', rclpy.qos.qos_profile_sensor_data)
        self.pub_servo = self.create_publisher(sensor_msgs.msg.JointState, 'servo_states', rclpy.qos.qos_profile_sensor_data) if self.declare_parameter('publish_servo_states', False).value else None
        self.subscribers = [
            self.create_subscription(sensor_msgs.msg.JointState, 'move_joint', self.init_trajectory, rclpy.qos.qos_profile_services_default),
            self.create_subscription(sensor_msgs.msg.JointState, 'servo_joint', self.set_joint_goal_direct, rclpy.qos.qos_profile_sensor_data),
            self.create_service(std_srvs.srv.Trigger, 'calibrate', self.calibrate)
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

        param = rclpy.parameter.Parameter(
            'servo_calibration_offset',
            rclpy.Parameter.Type.DOUBLE_ARRAY,
            self.servo_calibration_offset.tolist()
        )
        self.set_parameters([param])
        self.get_logger().info("Set calibration offset (in servo space) to: {} deg.".format(np.degrees(self.servo_calibration_offset)))
        self.reset_trajectory_generator()

    def read_servo_state(self):
        s = MyJointState()
        s.stamp = self.get_clock().now()

        if self.simulated:
            s.name = [str(dev_id) for dev_id in self.device_ids]
            np.copyto(s.position, self.sim_current_position_servo)
            np.copyto(s.velocity, self.sim_current_velocity_servo)
            np.copyto(s.effort, self.sim_current_effort_servo)
        else:
            for i, dev in enumerate(self.chain.devices):
                s.name.append(str(dev.id))

                if isinstance(dev, dynamixel.device.AX12) or isinstance(dev, dynamixel.device.MX28):
                    d = dev.read_params(['present_position', 'present_speed', 'present_load'])
                    s.position[i] = d['present_position']
                    s.velocity[i] = d['present_speed']
                    s.effort[i] = d['present_load']
                # elif isinstance(dev, dynamixel.device.MX28_2):
                #     d = dev.read_params(['present_load', 'present_velocity', 'present_position'])
                #     s.position[i] = d['present_position']
                #     s.velocity[i] = d['present_velocity']
                #     s.effort[i] = d['present_load']
                else:
                    raise RuntimeError("Unknown device type '{}' in chain".format(type(dev)))

        # Subtract calibration offset from servo positions
        s.position -= self.servo_calibration_offset
        return s

    def compute_joint_state(self, servo_state):
        s = MyJointState()
        s.stamp = servo_state.stamp
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
            for dev, pos in zip(self.chain.devices, pos_servo_next):
                dev.write_param_single('goal_position', pos)

    def set_joint_goal_direct(self, m):
        """Set servo set-point directly without trajectory generation
        """
        position = m.position.tolist()
        velocity = m.velocity.tolist()

        if self.trajectory:
            self.get_logger().warn("Ignoring direct joint goal: already moving along trajectory")
            return

        if not m.name or not position:
            self.get_logger().warn("Bad input (name or position fields empty)")
            return

        # Map names to servo IDs
        msg_dev_id = [self.joint_device_mapping[n] for n in m.name]

        # Deep copy
        target_position = self.tg.target_position.tolist()
        target_velocity = self.tg.target_velocity.tolist()

        for i, dev_id in enumerate(self.device_ids):
            if dev_id in msg_dev_id:
                j = msg_dev_id.index(dev_id)
                target_position[i] = position[j]
                target_velocity[i] = velocity[j] if m.velocity else 0

        # Update trajectory generator current so that future trajectories starts
        # from the correct initial position
        self.tg.current_position = target_position
        self.tg.current_velocity = target_velocity

        # Write the target directly to servos without generating the actual
        # trajectory
        self.write_joint_goal(target_position)

    def init_trajectory(self, m):
        """Set a joint-space trajectory setpoint for the given servos
        """
        position = m.position.tolist()
        velocity = m.velocity.tolist()

        if not m.name or not position:
            self.get_logger().warn("Bad input (name or position fields empty)")
            return

        # Map names to servo IDs
        msg_dev_id = [self.joint_device_mapping[n] for n in m.name]

        # Cache reference to these values while iterating (they are C++ types
        # exposed to Python opaquely)
        target_position = self.tg.target_position
        target_velocity = self.tg.target_velocity

        for i, dev_id in enumerate(self.device_ids):
            if dev_id in msg_dev_id:
                j = msg_dev_id.index(dev_id)
                target_position[i] = position[j]
                target_velocity[i] = velocity[j] if velocity else 0

        # rospy.loginfo("New target:\n\tjoints: {}\n\tpos:{} (servo)\n\tpos:{} (joint)".format(
        #     m.name,
        #     self.transmission.joint_to_servo(self.tg.target_position),
        #     np.array(self.tg.target_position)))

        self.trajectory = self.tg.trajectory()

    def step_trajectory(self):
        try:
            pos_joint_next = next(self.trajectory)[0]
            self.write_joint_goal(pos_joint_next)
        except StopIteration:
            self.trajectory = None

    def stop_trajectory_soft(self):
        self.tg.target_position[:] = self.tg.current_position
        self.tg.target_velocity.fill(0)
        self.trajectory = self.tg.trajectory()

    def stop_trajectory_hard(self):
        self.tg.target_position[:] = self.tg.current_position
        self.tg.target_velocity.fill(0)
        self.trajectory = None

    def publish(self, servo_state, joint_state):
        if self.pub_servo is not None:
            self.pub_servo.publish(servo_state.to_msg())

        self.pub_joint.publish(joint_state.to_msg())

    def run(self):
        self.get_logger().info("Starting servo loop")
        rate = self.create_rate(self.loop_rate_hz)

        try:
            while rclpy.ok():
                servo_state = self.read_servo_state()
                joint_state = self.compute_joint_state(servo_state)

                if self.trajectory:
                    self.step_trajectory()

                for f in self.state_update_callbacks:
                    f(servo_state, joint_state)

                rate.sleep()  # TODO: check that loop rate is kept
        except KeyboardInterrupt:
            pass
        finally:
            if not self.simulated:
                # disable servo torque
                for dev in self.chain.devices:
                    dev.write_param_single('torque_enable', False)

    def calibrate(self, request, response):
        if self.simulated:
            raise RuntimeError("Will not calibrate in simulated mode")

        calibrator = EUACalibrator(self)
        threading.Thread(target=calibrator.run).start()
        response.success = True
        return response


class EUACalibrationError(Exception):
    pass


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

    def wait_for_first_state_callback(self):
        def test_callback_received():
            with self.lock:
                return self.servo_state is None

        while test_callback_received():
            time.sleep(0.1)

    def wait_for_controller_trajectory_end(self):
        while self.c.trajectory is not None:
            time.sleep(0.1)

        time.sleep(0.75)

    def run(self):
        try:
            # Zero current calibration offset
            for dev in self.c.chain.devices:
                dev.write_param_single('torque_enable', False)

            time.sleep(0.2)
            self.c.set_servo_calibration_offset(np.zeros(4))
            time.sleep(0.2)

            # Subscribe to controller state changes
            self.c.state_update_callbacks.append(self.controller_state_changed)

            # Wait for first state update to arrive
            self.wait_for_first_state_callback()

            # Save current controller state
            self.tg_max_velocity = self.c.tg.max_velocity.copy()
            self.tg_max_acceleration = self.c.tg.max_acceleration.copy()
            self.tg_max_jerk = self.c.tg.max_jerk.copy()
            self.calibration_offset = self.c.servo_calibration_offset.copy()

            with self.lock:
                self.initial_joint_state = self.joint_state

            self._run()
        except EUACalibrationError as e:
            self.c.get_logger().error("Calibration error: {}".format(e))

            # Ramp down current trajectory and wait until movement has stopped
            self.c.stop_trajectory_soft()
            self.wait_for_controller_trajectory_end()

            # Restore previous calibration offset
            self.c.set_servo_calibration_offset(self.calibration_offset)

            # Move back to initial position
            self.c.init_trajectory(MyJointState(name=self.c.joint_names, position=self.initial_joint_state.position.tolist()))
            self.wait_for_controller_trajectory_end()
        finally:
            if self.controller_state_changed in self.c.state_update_callbacks:
                self.c.state_update_callbacks.remove(self.controller_state_changed)

            self.set_tg_limits_orig()

    def set_tg_limits_safe(self):
        # Set safe (low) trajectory generator limits
        self.c.tg.max_velocity = [np.pi / 4] * 4
        self.c.tg.max_acceleration = [2 * np.pi] * 4
        self.c.tg.max_jerk = [20 * np.pi] * 4

    def set_tg_limits_orig(self):
        # Reset original trajectory generator limits
        self.c.tg.max_velocity = self.tg_max_velocity
        self.c.tg.max_acceleration = self.tg_max_acceleration
        self.c.tg.max_jerk = self.tg_max_jerk

    def _run(self):
        self.set_tg_limits_safe()

        # The target position is the servo limits
        if self.c.eua_type == 1:
            servo_limits_lower = [dev.read_param_single('cw_angle_limit') for dev in self.c.chain.devices]
            servo_limits_upper = [dev.read_param_single('ccw_angle_limit') for dev in self.c.chain.devices]
        elif self.c.eua_type == 2:
            # MX28's in multi-turn mode
            servo_limits_lower = [-6*np.pi] * 4
            servo_limits_upper = [6*np.pi] * 4

        detected_positions_in_servo_space = [None for i in range(4)]
        reference_positions_in_servo_space = self.c.transmission.joint_to_servo(np.radians([0, 0, 130, 130]))  # Jaw limit is really around 110-115 deg.

        # FIXME: hard coded threshold values
        thresholds = self.c.get_parameter('homing_load_thresholds').value

        ROLL, PITCH, YAW1, YAW2 = range(4)
        LOWER, UPPER = range(2)

        def detect_limit_blocking(j, d):
            while True:
                time.sleep(1.0 / self.c.loop_rate_hz / 2)

                if self.c.trajectory is None:
                    raise EUACalibrationError("Trajectory reached the end (servo limit) without detecting the joint limit")

                with self.lock:
                    servo_state = self.servo_state
                    joint_state = self.joint_state

                if ((d == 0 and servo_state.effort[j] > thresholds[j])
                        or (d == 1 and servo_state.effort[j] < -thresholds[j])):
                    self.c.get_logger().info("Found '{}' {} joint limit at {:.1f} deg (servo) / {:.1f} deg (joint) - servo load: {:.3f}".format(
                        self.c.joint_names[j],
                        'upper' if d == 1 else 'lower',
                        round(np.degrees(servo_state.position[j]), 1),
                        round(np.degrees(joint_state.position[j]), 1),
                        round(servo_state.effort[j], 3)))

                    return servo_state.position[j], joint_state.position[j]

        def servo_to_joint_for_joint(j, j_pos_servo):
            with self.lock:
                target_position = self.servo_state.position.copy()

            target_position[j] = j_pos_servo
            return self.c.transmission.servo_to_joint(target_position)

        # Find grasper joint limits
        for j in (YAW1, YAW2):
            self.c.get_logger().info("Calibrating joint '{}'".format(self.c.joint_names[j]))

            # Move toward servo limit in the "grasper open"-direction (opposite
            # directions for each grasper jaw)
            if self.c.eua_type == 1:
                d = LOWER if j == YAW1 else UPPER
            elif self.c.eua_type == 2:
                d = UPPER

            target_position = servo_to_joint_for_joint(j, servo_limits_lower[j] + 0.01 if d == LOWER else servo_limits_upper[j] - 0.01)
            self.c.init_trajectory(MyJointState(name=[self.c.joint_names[j]], position=[target_position[j]]))

            # Wait until we hit the joint limit
            detected_positions_in_servo_space[j] = detect_limit_blocking(j, d)[0]
            self.c.stop_trajectory_hard()

            # Move back to the initial position
            self.set_tg_limits_orig()
            self.c.init_trajectory(MyJointState(name=[self.c.joint_names[j]], position=[self.initial_joint_state.position[j]]))
            self.wait_for_controller_trajectory_end()
            self.set_tg_limits_safe()

        # Find rotation and wrist lower+upper limits (the mid-points between these
        # values are then the zero position)
        for j in (ROLL, PITCH):
            self.c.get_logger().info("Calibrating joint '{}'".format(self.c.joint_names[j]))
            limits = [None, None]

            for d in (LOWER, UPPER):
                # Move toward servo lower- and upper limits
                target_position = servo_to_joint_for_joint(j, servo_limits_lower[j] + 0.01 if d == LOWER else servo_limits_upper[j] - 0.01)
                self.c.init_trajectory(MyJointState(name=[self.c.joint_names[j]], position=[target_position[j]]))

                # Wait until we hit the joint limit
                limits[d] = detect_limit_blocking(j, d)[0]
                self.c.stop_trajectory_hard()

            # Move back to the initial position
            self.set_tg_limits_orig()
            self.c.init_trajectory(MyJointState(name=[self.c.joint_names[j]], position=[self.initial_joint_state.position[j]]))
            self.wait_for_controller_trajectory_end()
            self.set_tg_limits_safe()

            # Save the detected center position
            detected_positions_in_servo_space[j] = limits[0] + (limits[1] - limits[0]) / 2

        self.c.set_servo_calibration_offset(np.array(detected_positions_in_servo_space) - reference_positions_in_servo_space)

        # Move joints to home position
        self.set_tg_limits_orig()
        home_position = np.radians([0, 0, 30, 30])  # FIXME: hard coded values
        self.c.init_trajectory(MyJointState(name=self.c.joint_names, position=home_position.tolist()))

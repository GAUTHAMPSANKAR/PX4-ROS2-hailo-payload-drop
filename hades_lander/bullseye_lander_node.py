#!/usr/bin/env python3

from mavros_msgs.srv import SetMode, CommandBool, CommandLong
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rclpy.qos
from tf_transformations import euler_from_quaternion
import math
from mavros_msgs.msg import State
from enum import Enum
from simple_pid import PID
from geometry_msgs.msg import Point 


class MissionState(Enum):
    INIT = 0
    ARMING = 1
    MISSION_FLIGHT = 2
    TARGET_ACQUIRED = 3
    STABILIZING = 4
    PAYLOAD_DROP = 5
    RTL = 6
    MISSION_RECOVERY = 7

class BullseyeDetector(Node):
    def __init__(self):
        super().__init__('bullseye_lander')
        
        self._declare_parameters()
        
        self.mission_state = MissionState.INIT

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)        
        self.cmd_vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_callback, qos)
        self.global_pos_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.global_position_callback, qos)

        self.error_sub = self.create_subscription(Point, '/bullseye/pixel_error', self.error_callback, 10)
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        while not self.command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/command service...')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')
        self.get_logger().info('All services are available.')

        self.offboard_timer = self.create_timer(0.02, self.velocity_loop)

        self._initialize_variables()
        self._log_parameters()
    
    def _log_parameters(self):
        """Log parameter values for verification"""
        self.get_logger().info(f"Control Kp XY: {self.Kp}")
        self.get_logger().info(f"Control Kp Z: {self.Kpd}")
        self.get_logger().info(f"Max Velocity: {self.mvel}")
        self.get_logger().info(f"Vision Threshold (pixels): {self.threshold}")
        self.get_logger().info(f"Mission Hold Duration: {self.hold_duration}")
        self.get_logger().info(f"Mission Descent Altitude: {self.descent_altitude}")
        self.get_logger().info(f"Post Payload Drop RTL: {self.post_drop_rtl}")

    def _declare_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[ # type: ignore[arg-type]
                # --- PID GAINS AND CLAMPS -----------
                ('hades_control.kp_xy', 1.5),    # Proportional Gain for XY
                ('hades_control.kp_z', 0.2),     # Proportional Gain for XY
                ('hades_control.ki_xy', 0.000),  # Integral Gain for XY
                ('hades_control.kd_xy', 0.00),   # Derivative Gain for XY
                ('hades_control.ki_z', 0.0),     # Integral Gain for Z
                ('hades_control.kd_z', 0.0),     # Derivative Gain for Z
                # ------------------------------------

                ('hades_control.max_velocity', 0.5),
                ('hades_vision.threshold_pixels', 0.02),
                ('hades_mission.hold_duration', 5.0),
                ('hades_mission.descent_altitude', 5.0),
                ('hades_mission.target_confirmation_frames', 20),
                ('hades_mission.post_drop_rtl', True),
            ]
        )

    def _initialize_variables(self):
        # Load parameter values
        self.Kp = self.get_parameter('hades_control.kp_xy').value
        self.Kpd = self.get_parameter('hades_control.kp_z').value

        Ki = self.get_parameter('hades_control.ki_xy').value
        Kd = self.get_parameter('hades_control.kd_xy').value
        Ki_z = self.get_parameter('hades_control.ki_z').value
        Kd_z = self.get_parameter('hades_control.kd_z').value

        self.mvel = self.get_parameter('hades_control.max_velocity').value
        self.threshold = self.get_parameter('hades_vision.threshold_pixels').value
        self.hold_duration = self.get_parameter('hades_mission.hold_duration').value
        self.descent_altitude = self.get_parameter('hades_mission.descent_altitude').value
        self.target_confirmation_frames = self.get_parameter('hades_mission.target_confirmation_frames').value
        self.post_drop_rtl = self.get_parameter('hades_mission.post_drop_rtl').value

        self.prev_mode = None
        self.prev_armed = None
        self.prev_state = None
        self.task_over = False
        self.dont_call_me_twice = False
        self.offboard_mode = False
        self.hold = False
        self.hold_start_time = None
        self.hold_started = 0
        self.stabilizing_start_time = None

        self.target_acquired = False
        self.current_error_dx = 0.0
        self.current_error_dy = 0.0

        self.armed = False
        self.initial_arm_attempt = True
        self.current_mode = None
        
        self.current_position = None
        self.current_yaw = 0.0
        self.z = 0.0
        
        self.target_acquired = False

        self.velocity_command = Twist()
        self.pid_vx = PID(self.Kp, Ki, Kd, setpoint=0)
        self.pid_vy = PID(self.Kp, Ki, Kd, setpoint=0)
        self.prev_vel = None
        
        self.pid_z = PID(self.Kpd, Ki_z, Kd_z, setpoint=self.descent_altitude)
        self.i = 0
        self.pid_vx.output_limits = (-self.mvel, self.mvel)
        self.pid_vy.output_limits = (-self.mvel, self.mvel)
        self.pid_z.output_limits = (-self.mvel, self.mvel)

        self.retry_timer = None
        self.target_count = 0
        self.waiting = 0
        self.payload_timer = None

        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.current_altitude = 0.0
        self.gps_pos = None
        self.last_known_target_pos = None

    def send_reposition_command(self):
        if abs(self.last_known_target_pos.latitude) < 0.001 and abs(self.last_known_target_pos.longitude) < 0.001:
            self.get_logger().warn("Current GPS position unknown, cannot send reposition command.") 
            self.set_mode("AUTO.LOITER")
            return False


        req = CommandLong.Request()
        req.command = 192
        req.confirmation = 0
        req.param1 = float(1)
        req.param2 = 1.0
        req.param3 = 0.0
        req.param4 = float((-self.current_yaw)) 
        req.param5 = float(self.last_known_target_pos.latitude) if self.last_known_target_pos else float(self.current_latitude)
        req.param6 = float(self.last_known_target_pos.longitude) if self.last_known_target_pos else float(self.current_longitude)
        req.param7 = float('nan')   
        self.get_logger().info(f"Sending reposition command to lat: {self.current_latitude}, lon: {self.current_longitude}, alt: {self.current_altitude }")
      
        future = self.command_client.call_async(req)
        
        def reposition_done_cb(fut):
            try:
                res = fut.result()
                if res and res.success:
                    self.get_logger().info("✓ Reposition command accepted by PX4")
                else:
                    result_code = res.result if res else "No response"
                    self.get_logger().warn(f"✗ Reposition command rejected. Result code: {result_code}")
                    # Fallback to LOITER
                    self.set_mode("AUTO.LOITER")
            except Exception as e:
                self.get_logger().error(f"Reposition command exception: {e}")
                self.set_mode("AUTO.LOITER")
        
        future.add_done_callback(reposition_done_cb)
        return True

    def global_position_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.current_altitude = msg.altitude
        self.gps_pos = msg

    def error_callback(self, msg):

        if self.mission_state in [MissionState.INIT, MissionState.ARMING, MissionState.RTL]:
            return

        if msg.z > 0:
            self.target_acquired = True
            self.current_error_dx = msg.x
            self.current_error_dy = msg.y
            self.last_known_target_pos = self.gps_pos if self.gps_pos else None
        else:
            self.target_acquired = False
            self.current_error_dx = 0.0
            self.current_error_dy = 0.0

        if self.mission_state in [MissionState.TARGET_ACQUIRED, MissionState.MISSION_RECOVERY, MissionState.MISSION_FLIGHT, MissionState.STABILIZING]:
            self.update_mission_state_from_detection()
    
    def state_callback(self, msg):
        self.current_mode = msg.mode
        self.armed = msg.armed 

        if self.current_mode != self.prev_mode or self.armed != self.prev_armed or self.prev_state != self.mission_state.name:
            self.get_logger().warn(f"Current Mode: {self.current_mode}, Armed: {self.armed}, Mission State: {self.mission_state.name} Z: {self.z:.2f}")
        self.prev_mode = self.current_mode
        self.prev_armed = self.armed
        self.prev_state = self.mission_state.name

        if self.mission_state == MissionState.INIT:
            self.mission_state = MissionState.ARMING
            
        if self.mission_state == MissionState.RTL:
            if self.task_over:
                self.get_logger().info("Task over")
                self.task_over = False
            return

        if self.current_mode == "AUTO.RTL":
            self.mission_state = MissionState.RTL
            
        if self.mission_state == MissionState.ARMING and self.current_mode == "AUTO.MISSION" and self.armed:
            self.mission_state = MissionState.MISSION_FLIGHT

        if not self.armed and not self.mission_state == MissionState.RTL:
            if self.current_mode == "AUTO.LOITER":
                if not self.retry_timer:
                    self.try_arming()
                    self.mission_state = MissionState.ARMING
                self.initial_arm_attempt = False
                return
            if self.current_mode != "AUTO.LOITER":
                self.set_mode("AUTO.LOITER")

        if self.armed:
            self.initial_arm_attempt = False
            if self.retry_timer:
                self.get_logger().info("Retrying timer canceled, vehicle is armed.")
                self.retry_timer.cancel()
                self.retry_timer = None
                
            if self.mission_state == MissionState.ARMING:
                if not self.current_mode == "AUTO.MISSION":
                    self.get_logger().info("Retrying mission")
                    self.mission_state = MissionState.MISSION_FLIGHT
                    self.try_mission_flight()

    def try_mission_flight(self):
        if self.mission_state == MissionState.MISSION_FLIGHT or self.mission_state == MissionState.MISSION_RECOVERY:
            if self.current_mode != "AUTO.MISSION":
                self.set_mode("AUTO.MISSION")
                self.mission_state = MissionState.MISSION_FLIGHT
    
    def set_mode(self, mode):
        if self.current_mode != mode:
            self.get_logger().info(f"Mode is {self.current_mode}, switching to '{mode}'...")
            req_mode = SetMode.Request()
            req_mode.custom_mode = mode
            req_mode.base_mode = 0

            future_mode = self.set_mode_client.call_async(req_mode)
            self.get_logger().info(f"Requesting mode change to '{mode}'")
            
            def mode_done_cb(fut):
                try:
                    res = fut.result()
                    if res and res.mode_sent:
                        self.get_logger().info(f"{mode} requested")
                    else:
                        self.get_logger().warn(f"PX4 denied {mode} request.")
                        self.get_logger().info(f"Current mode is still {self.current_mode}")    
                except Exception as e:
                    self.get_logger().error(f"Mode request exception: {e}")
            future_mode.add_done_callback(mode_done_cb)

    def try_arming(self):
        if not self.armed:
            self.get_logger().info("Arming vehicle...")
            req_arm = CommandBool.Request()
            req_arm.value = True
            future_arm = self.arming_client.call_async(req_arm)
            
            def arm_done_cb(fut):
                try:
                    if fut.result() and fut.result().success:
                        self.get_logger().info("Vehicle armed successfully.")
                    else:
                        self.get_logger().error("Failed to arm vehicle.")
                        self._ensure_retry_timer()
                except Exception as e:
                    self.get_logger().error(f"Arming request exception: {e}")
            future_arm.add_done_callback(arm_done_cb)
            
        if self.armed:
            if self.retry_timer:
                self.get_logger().info("Retrying timer canceled, vehicle is armed.")
                self.retry_timer.cancel()
                self.retry_timer = None
                return

    def _ensure_retry_timer(self):
        if self.retry_timer is None:
            self.get_logger().info("Arm Retry timer started")
            self.retry_timer = self.create_timer(5.0, self.try_arming)
   
    def drop_payload(self):
        if self.mission_state != MissionState.PAYLOAD_DROP or self.payload_timer is not None:
            return
            
        self.get_logger().info("Payload drop sequence initiated. Timer started.")
        
        # Reset the state counter
        self.waiting = 0 
        
        # Create a new timer that runs every 1.5 seconds (adjust as needed)
        # This timer will call our new sequence function.
        self.payload_timer = self.create_timer(1.5, self._payload_sequence_callback)

    def _payload_sequence_callback(self):


            if self.mission_state == MissionState.RTL:
                if self.payload_timer:
                    self.payload_timer.cancel()
                    self.payload_timer = None
                return

            if self.waiting == 0:
                self.get_logger().info("Payload drop (Step 1/2): Releasing payload (moving to MIN, Normalized -1.0)...")
                req = CommandLong.Request()
                req.command = 187  
                req.param1 = float(2)    
                req.param2 = float(-1.0) 
                self.command_client.call_async(req)
                self.waiting = 1 

            elif self.waiting == 1:
                self.get_logger().info("Payload drop (Step 2/2): Release command sent. Switching to RTL.")
                self.mission_state = MissionState.RTL
                if self.post_drop_rtl:
                    self.set_mode("AUTO.RTL")
                else:
                    self.set_mode("AUTO.MISSION")
                self.task_over = True
                
                if self.payload_timer:
                    self.payload_timer.cancel()
                    self.payload_timer = None
                self.waiting = 2 
    def update_mission_state_from_detection(self):
        if not self.armed:
            return

        if self.mission_state not in [MissionState.TARGET_ACQUIRED, MissionState.PAYLOAD_DROP, MissionState.RTL, MissionState.STABILIZING] and self.target_acquired:
            self.get_logger().info("Target acquired, switching to HOLD mode.")
            self.send_reposition_command()
            self.mission_state = MissionState.TARGET_ACQUIRED
            
        if self.mission_state == MissionState.TARGET_ACQUIRED and self.current_mode == "AUTO.LOITER":
            if self.hold_started == 0:
                self.hold_started = 1
                self.hold_state()
                self.target_count = 0
        self.hold_state()


        if self.mission_state == MissionState.MISSION_RECOVERY and self.dont_call_me_twice:
            self.hold_started = 0
            self.hold_start_time = None
            self.try_mission_flight()
            self.dont_call_me_twice = False
            return

        if self.current_mode == "AUTO.LOITER" and self.hold_started == 2 and not self.target_acquired:
            self.hold_state()
            self.target_count = self.target_count + 1
            if self.target_count > 10:
                if self.mission_state != MissionState.MISSION_RECOVERY:
                    self.mission_state = MissionState.MISSION_RECOVERY
                    self.dont_call_me_twice = True
                    return 
            else:
                return
                
        if self.current_mode == "AUTO.LOITER" and self.hold_started == 2 and self.target_acquired:
            self.hold_state()
            if self.mission_state != MissionState.STABILIZING:
                self.mission_state = MissionState.STABILIZING
            if self.mission_state == MissionState.STABILIZING and self.current_mode != "OFFBOARD":
                self.set_mode("OFFBOARD")
                self.hold_start_time = None
                self.hold_started = 0
                self.target_count = 0
                
        if self.current_mode == "OFFBOARD" and self.mission_state == MissionState.STABILIZING and self.target_acquired:
            self._stabilization_control()
            
        if self.current_mode == "OFFBOARD" and not self.target_acquired and self.mission_state == MissionState.STABILIZING:
            self.target_count = self.target_count + 1
            # self.get_logger().info(f"Target count: {self.target_count}")
            if self.target_count > self.target_confirmation_frames:  #type: ignore
                self.get_logger().info(f"Target lost during Stablization, switching to MISSION_RECOVERY mode: {self.target_count}")
                if self.mission_state != MissionState.MISSION_RECOVERY:
                    # self.get_logger().info(f"{self.mission_state}")
                    self.mission_state = MissionState.MISSION_RECOVERY
                    self.dont_call_me_twice = True

    def _stabilization_control(self):
        if self.current_position is None or self.mission_state != MissionState.STABILIZING:
            return

        # if self.stable_target:
        #     x, y, _ = self.stable_target
        # else:
        #     x, y, _ = self.last_known_target if self.last_known_target else (0, 0, None)
            
        # if self.image_center is None:
        #     return
            
        # dx = x - self.image_center[0]
        # dy = y - self.image_center[1]
        dx = self.current_error_dx
        dy = self.current_error_dy
        dz = self.z - self.descent_altitude #type: ignore

        # error_msg = Point()
        # error_msg.x = float(dx)
        # error_msg.y = float(dy)
        # error_msg.z = float(dz)
        # self.error_pub.publish(error_msg)
        
        if not math.isclose(dx, 0.0, abs_tol=self.threshold) or not math.isclose(dy, 0.0, abs_tol=self.threshold) or not math.isclose(dz, 0.0, abs_tol=0.5):    #type: ignore
            self.i = 0
            self.send_velocity_command(dx, dy, dz)
            self.stabilizing_start_time = None
        else:
            self.send_velocity_command(0, 0, 0)
            
        if math.isclose(dx, 0.0, abs_tol=self.threshold) and math.isclose(dy, 0.0, abs_tol=self.threshold) and math.isclose(dz, 0.0, abs_tol=0.5):      #type: ignore
            if self.stabilizing_start_time is None:
                self.stabilizing_start_time = self.get_clock().now()
            else:
                elapsed_stabilizing = (self.get_clock().now() - self.stabilizing_start_time).nanoseconds * 1e-9
                if elapsed_stabilizing >= 3.0:
                    self.get_logger().info("Stabilization complete, descending...")
                    self.mission_state = MissionState.PAYLOAD_DROP
                    self.drop_payload()
                    self.stabilizing_start_time = None
                    
    def hold_state(self):

        if self.hold_started in [0, 2]:
            return
            
        if self.hold_start_time is None:
            self.hold_start_time = self.get_clock().now()
            self.get_logger().info("Hold timer started.")
            return
            
        elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
        if elapsed >= self.hold_duration:    #type: ignore
            self.get_logger().info(f"Hold duration complete {self.z}")
            self.hold_started = 2
        else:
            return

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        self.current_position = pos
        _, _, self.current_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.z = pos.z     

    def send_velocity_command(self, dx, dy, dz):

        cmd = Twist()
        vx_body = self.pid_vx(dy)
        vy_body = self.pid_vy(dx)
        vz = self.pid_z(self.z)

        yaw = self.current_yaw
        vx = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
        vy = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)

        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz

        self.velocity_command = cmd
        

    def velocity_loop(self):
        self.cmd_vel_pub.publish(self.velocity_command)
        if self.prev_vel == self.velocity_command:
            self.i += 1
        else:
            self.i = 0
        if self.i > 30:
            self.velocity_command = Twist()
        self.prev_vel = self.velocity_command


    def destroy_node(self):
        self.get_logger().info("Shutting down bullseye lander...")
        
        if hasattr(self, 'retry_timer') and self.retry_timer:
            self.retry_timer.cancel()
                        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BullseyeDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt...")
    except Exception as e:
        node.get_logger().error(f"Node error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
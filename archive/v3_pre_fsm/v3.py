"""
Prototype v3 – Pre-FSM enhanced autonomy node.

The system still used simplified 'landing' behavior as a stand-in for the
real payload-drop mission during testing, but now includes:

- A clearer internal phase structure
- Realistic mission interaction (AUTO.MISSION → OFFBOARD → RTL)
- Stabilization timing and improved target consistency logic

This was the last major prototype before replacing the landing test behavior
with the actual payload-drop sequence in the final `hades_lander` package.
"""



from mavros_msgs.srv import SetMode, CommandBool, CommandLong
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy.qos
from tf_transformations import euler_from_quaternion
import math
from mavros_msgs.msg import State
from enum import Enum
from collections import deque


class MissionState(Enum):
    """Enhanced state machine for mission control"""
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
        
        # Declare parameters first
        self._declare_parameters()
        
        # State machine - keep your original structure
        self.mission_state = MissionState.INIT

        # ROS2 publishers and subscribers - keep your original setup
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)        
        self.cmd_vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_callback, qos)
        self.global_pos_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.global_position_callback, qos)
        
        # Service clients
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

        self.bridge = CvBridge()
        self.display_timer = self.create_timer(0.01, self.display)
        self.offboard_timer = self.create_timer(0.05, self.offboard_keepalive)

        # Initialize variables with parameters
        self._initialize_variables()
        self._log_parameters()
    
    def _log_parameters(self):
        """Log parameter values for verification"""
        self.get_logger().info(f"Control Kp XY: {self.Kp}")
        self.get_logger().info(f"Control Kp Z: {self.Kpd}")
        self.get_logger().info(f"Max Velocity: {self.mvel}")
        self.get_logger().info(f"Vision Threshold (pixels): {self.threshold}")
        self.get_logger().info(f"Vision Detection Confidence: {self.min_confidence}")
        self.get_logger().info(f"Vision Max Frames Lost: {self.max_frames_lost}")
        self.get_logger().info(f"Mission Hold Duration: {self.hold_duration}")
        self.get_logger().info(f"Mission Descent Altitude: {self.descent_altitude}")
        self.get_logger().info(f"Mission Detection History Size: {self.detection_history.maxlen}")
        self.get_logger().info(f"Control Exponential Weight: {self.eweight}")

    def _declare_parameters(self):
        """Declare ROS parameters for tuning"""
        self.declare_parameters(
            namespace='',
            parameters=[ # type: ignore[arg-type]
                ('hades_control.kp_xy', 0.009),
                ('hades_control.kp_z', 0.5),
                ('hades_control.max_velocity', 0.5),
                ('hades_vision.threshold_pixels', 5),
                ('hades_vision.detection_confidence', 0.7),
                ('hades_vision.max_frames_lost', 15),
                ('hades_mission.hold_duration', 5.0),
                ('hades_mission.descent_altitude', 10.0),
                ('hades_mission.detection_history_size', 15),
                ('hades_control.exponential_weight', -2.0),
                ('hades_mission.target_confirmation_frames', 20)
            ]
        )

    def _initialize_variables(self):
        """Initialize variables using parameter values"""
        # Load parameter values
        self.Kp = self.get_parameter('hades_control.kp_xy').value
        self.Kpd = self.get_parameter('hades_control.kp_z').value
        self.mvel = self.get_parameter('hades_control.max_velocity').value
        self.threshold = self.get_parameter('hades_vision.threshold_pixels').value
        self.min_confidence = self.get_parameter('hades_vision.detection_confidence').value
        self.max_frames_lost = self.get_parameter('hades_vision.max_frames_lost').value
        self.hold_duration = self.get_parameter('hades_mission.hold_duration').value
        self.descent_altitude = self.get_parameter('hades_mission.descent_altitude').value
        self.eweight = self.get_parameter('hades_control.exponential_weight').value
        detection_history_size = self.get_parameter('hades_mission.detection_history_size').value
        self.target_confirmation_frames = self.get_parameter('hades_mission.target_confirmation_frames').value

        # Keep your original variable names and logic
        self.prev_mode = None
        self.prev_armed = None
        self.prev_state = None
        self.task_over = False
        self.dont_call_me_twice = False
        self.offboard_mode = False
        self.hold = False
        self.hold_start_time = None
        self.hold_started = 0
        self.publishing_velocity = False
        self.stabilizing_start_time = None
        
        # Vehicle state
        self.armed = False
        self.initial_arm_attempt = True
        self.current_mode = None
        
        # Position and orientation
        self.current_position = None
        self.current_yaw = 0.0
        self.z = 0.0
        
        # Image processing parameters
        self.image_center = None        
        self.detection_history = deque(maxlen=detection_history_size)
        self.stable_target = None
        self.target_confidence = 0.0
        self.last_known_target = None
        self.frames_since_detection = 0
        self.target_acquired = False

        # Velocity smoothing memory
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vz = 0.0

        # Timers
        self.retry_timer = None
        self.cv_image = None
        self.target_count = 0


        # Add these variables in _initialize_variables
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
        req.param4 = float(self.current_yaw) 
        req.param5 = float(self.last_known_target_pos.latitude) if self.last_known_target_pos else float(self.current_latitude)
        req.param6 = float(self.last_known_target_pos.longitude) if self.last_known_target_pos else float(self.current_longitude)
        req.param7 = float('nan')   # Add 5 meters to current altitude for safety
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

    # Add this callback method
    def global_position_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.current_altitude = msg.altitude
        self.gps_pos = msg
    
    def display(self):
        """Keep your original display logic"""
        if self.cv_image is not None and self.image_center is not None:
            display_img = self.cv_image.copy()
            cv2.drawMarker(
                display_img, self.image_center, (255, 0, 0),
                cv2.MARKER_CROSS, 20, 2
            )
            
            if self.stable_target and self.target_confidence >= self.min_confidence:    #type: ignore
                x, y, r = self.stable_target
                cv2.circle(display_img, (x, y), r, (0, 255, 0), 2)
                cv2.circle(display_img, (x, y), 3, (0, 255, 0), -1)
                cv2.line(display_img, self.image_center, (x, y), (0, 255, 0), 2)
            elif self.last_known_target:
                x, y, r = self.last_known_target
                cv2.circle(display_img, (x, y), r, (255, 255, 0), 2)
                cv2.circle(display_img, (x, y), 3, (255, 255, 0), -1)
                cv2.line(display_img, self.image_center, (x, y), (255, 255, 0), 2)
                
            target_detected = bool(self.stable_target and self.target_confidence >= self.min_confidence)    #type: ignore
            cv2.putText(
                display_img, f"target_detected : {target_detected}\ntarget_confidence : {self.target_confidence}", (10, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2
            )
            
            cv2.imshow('Bullseye Landing System', display_img)
            cv2.waitKey(1)
     
    def state_callback(self, msg):
        """Keep your working state callback logic"""
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
        """Keep your original mission flight logic"""
        if self.mission_state == MissionState.MISSION_FLIGHT or self.mission_state == MissionState.MISSION_RECOVERY:
            if self.current_mode != "AUTO.MISSION":
                self.set_mode("AUTO.MISSION")
                self.mission_state = MissionState.MISSION_FLIGHT
    
    def set_mode(self, mode):
        """Keep your original set_mode logic"""
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
        """Keep your original arming logic"""
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
        """Keep your original retry timer logic"""
        if self.retry_timer is None:
            self.get_logger().info("Arm Retry timer started")
            self.retry_timer = self.create_timer(5.0, self.try_arming)
   
    def image_callback(self, msg):
        """Keep your original image callback logic"""
        if self.mission_state in [MissionState.INIT, MissionState.ARMING]:
            return
            
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            thickness = 100
            
            if self.mission_state in [MissionState.MISSION_FLIGHT, MissionState.MISSION_RECOVERY]:
                h, w = self.cv_image.shape[:2]
                mask = self.cv_image.copy()
                color = (0, 0, 0)
                cv2.rectangle(mask, (0, 0), (w, thickness), color, -1)          # top
                cv2.rectangle(mask, (0, h-thickness), (w, h), color, -1)        # bottom
                cv2.rectangle(mask, (0, 0), (thickness, h), color, -1)          # left
                cv2.rectangle(mask, (w-thickness, 0), (w, h), color, -1)     
                self.cv_image = mask   

            if self.image_center is None and self.cv_image is not None:
                height, width = self.cv_image.shape[:2]
                self.image_center = (width // 2, height // 2)
                
            if not self.mission_state == MissionState.RTL and self.z > 9.0:
                detected_target = self.detect_bullseye(self.cv_image)
                self.detection_history.append(detected_target)
                self._update_stable_target()
                
            if self.mission_state in [MissionState.TARGET_ACQUIRED, MissionState.MISSION_RECOVERY, MissionState.MISSION_FLIGHT, MissionState.STABILIZING]:
                self.update_mission_state_from_detection()

        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
 
    def update_mission_state_from_detection(self):
        """Keep your original detection state logic"""
        if not self.armed:
            return
            
        if self.stable_target and self.target_confidence >= self.min_confidence:    #type: ignore
            self.target_acquired = True
        else:
            self.target_acquired = False
            
        if self.mission_state not in [MissionState.TARGET_ACQUIRED, MissionState.PAYLOAD_DROP, MissionState.RTL, MissionState.STABILIZING] and self.target_acquired:
            self.get_logger().info("Target acquired, switching to HOLD mode.")
            # self.set_mode("AUTO.LOITER")
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
            # self.get_logger().info("Am i trouble?")
            self.try_mission_flight()
            self.dont_call_me_twice = False
            return

        if self.current_mode == "AUTO.LOITER" and self.hold_started == 2 and not self.target_acquired:
            self.hold_state()
            self.target_count = self.target_count + 1
            # self.get_logger().info(f"Target count: {self.target_count}")
            if self.target_count > 10:
                # self.get_logger().info(f"Target lost during hold, switching to MISSION_RECOVERY mode: {self.target_count}")
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
            self.publishing_velocity = False
            self.target_count = self.target_count + 1
            # self.get_logger().info(f"Target count: {self.target_count}")
            if self.target_count > self.target_confirmation_frames:  #type: ignore
                self.get_logger().info(f"Target lost during Stablization, switching to MISSION_RECOVERY mode: {self.target_count}")
                if self.mission_state != MissionState.MISSION_RECOVERY:
                    # self.get_logger().info(f"{self.mission_state}")
                    self.mission_state = MissionState.MISSION_RECOVERY
                    self.dont_call_me_twice = True

    def drop_payload(self):
        """Keep your original payload drop logic"""
        if self.mission_state == MissionState.PAYLOAD_DROP:
            self.get_logger().info("Payload drop sequence initiated.")
            self.get_logger().info("Payload dropped successfully.")
            self.mission_state = MissionState.RTL
            self.set_mode("AUTO.RTL")
            self.get_logger().info("Switching to RTL mode.")
            self.task_over = True
            
    def _stabilization_control(self):
        """Keep your original stabilization control logic"""
        if self.stable_target is None or self.current_position is None or self.image_center is None or self.mission_state != MissionState.STABILIZING:
            return

        if self.stable_target:
            x, y, _ = self.stable_target
        else:
            x, y, _ = self.last_known_target if self.last_known_target else (0, 0, None)
            
        if self.image_center is None:
            return
            
        dx = x - self.image_center[0]
        dy = y - self.image_center[1]
        dz = self.z - self.descent_altitude #type: ignore
        
        if not math.isclose(dx, 0.0, abs_tol=self.threshold) or not math.isclose(dy, 0.0, abs_tol=self.threshold) or not math.isclose(dz, 0.0, abs_tol=0.5):    #type: ignore
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
        """Keep your original hold state logic"""
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

    def _update_stable_target(self):
        """Keep your original stable target update logic"""
        valid_detections = [d for d in self.detection_history if d is not None]
        
        if not valid_detections:
            self.target_confidence = 0.0
            self.frames_since_detection += 1
            
            if self.frames_since_detection > self.max_frames_lost: #type: ignore
                self.last_known_target = self.stable_target
                self.stable_target = None
            return
            
        
        self.target_confidence = len(valid_detections) / len(self.detection_history)
        self.frames_since_detection = 0
        
        weights = np.exp(np.linspace(self.eweight, 0, len(valid_detections))) #type: ignore
        weights /= weights.sum()
        
        x_avg = sum(w * d[0] for w, d in zip(weights, valid_detections))
        y_avg = sum(w * d[1] for w, d in zip(weights, valid_detections))
        r_avg = sum(w * d[2] for w, d in zip(weights, valid_detections))
        # self.get_logger().info("updating stable target")
        
        self.stable_target = (int(x_avg), int(y_avg), int(r_avg))
        self.last_known_target_pos = self.gps_pos if self.gps_pos else None

    def detect_bullseye(self, image):
        """Keep your original bullseye detection logic"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        blurred = cv2.GaussianBlur(enhanced, (9, 9), 2)
        
        circles = cv2.HoughCircles(
            blurred, 
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=80,
            param1=80,
            param2=40,
            minRadius=10,
            maxRadius=200
        )
        
        if circles is None:
            return None
        
        circles = np.round(circles[0, :]).astype("int")
        best_circle = None
        best_score = float('inf')
        
        for (x, y, r) in circles:
            dist_from_center = math.sqrt(
                (x - self.image_center[0])**2 + (y - self.image_center[1])**2 #type: ignore
            )
            score = dist_from_center
            
            if score < best_score:
                best_score = score
                best_circle = (x, y, r)
        # self.get_logger().info("returning circle")
                
        return best_circle

    def odom_callback(self, msg):
        """Keep your original odometry callback"""
        q = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        self.current_position = pos
        _, _, self.current_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.z = pos.z     

    def send_velocity_command(self, dx, dy, dz):
        """Keep your original velocity command logic"""
        self.publishing_velocity = True
        cmd = Twist()

        # --- Smoothed proportional control ---
        alpha = 0.3  # smoothing factor (0.0 = very smooth, 1.0 = instant)
        vx_body_raw = -dy * self.Kp
        vy_body_raw = -dx * self.Kp
        vz_raw = -dz * self.Kpd

        # Exponential smoothing for smoother motion
        vx_body = alpha * vx_body_raw + (1 - alpha) * self.prev_vx
        vy_body = alpha * vy_body_raw + (1 - alpha) * self.prev_vy
        vz = alpha * vz_raw + (1 - alpha) * self.prev_vz

        # Store previous velocities
        self.prev_vx, self.prev_vy, self.prev_vz = vx_body, vy_body, vz
        # --- End smoothing ---

        yaw = self.current_yaw
        vx = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
        vy = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)

        vx = max(min(vx, self.mvel), -self.mvel) # Clamp to max velocity#type: ignore
        vy = max(min(vy, self.mvel), -self.mvel)    #type: ignore
        vz = max(min(vz, self.mvel), -self.mvel)    #type: ignore
        
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def offboard_keepalive(self):
        """Keep your original offboard keepalive"""
        if not self.publishing_velocity:
            self.cmd_vel_pub.publish(Twist())

    def destroy_node(self):
        """Safe destruction with error handling"""
        self.get_logger().info("Shutting down bullseye lander...")
        
        # Cancel timers
        if hasattr(self, 'retry_timer') and self.retry_timer:
            self.retry_timer.cancel()
            
        # Safe window destruction
        try:
            if cv2.getWindowProperty('Bullseye Landing System', cv2.WND_PROP_VISIBLE) >= 0:
                cv2.destroyAllWindows()
        except:
            pass  # Ignore window errors during shutdown
            
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
"""
Prototype v0 – Monolithic autonomy node (landing used as a test behavior).

Note:
This version attempted a simplified “landing on the target” behavior ONLY
to validate core concepts such as:

- Pixel-error → velocity mapping
- Mode switching (LOITER → OFFBOARD)
- Basic alignment over a visual target
- Descent logic execution in OFFBOARD mode

The real mission objective was always payload drop, not landing.
Landing here was a convenient way to exercise the control system during
the earliest prototyping phase.

This file is preserved solely as a record of the engineering evolution.
"""



from mavros_msgs.srv import SetMode, CommandBool
import rclpy


from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy.qos
from tf_transformations import euler_from_quaternion
import math
from mavros_msgs.msg import State

class BullseyeDetector(Node):
    def __init__(self):
        super().__init__('bullseye_lander')

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        

        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_callback, qos)

        self.hold = True
        self.bullseye_detected = False
        self.initial_arm_attempt  = True
        self.bridge = CvBridge()
        self.armed = False
        self.current_mode = None
        self.retry_timer = None
        self.idle_vel_timer = None
        self.image_center = None
        self.stable_bullseye_center = None
        self.r = 0.0
        self.min_detections = 7
        self.Kp = 0.009
        self.current_yaw = 0.0
        self.threshold = 3
        self.mvel = 0.5
        self.offboard_started = False
        self.bullseye_center = None
        self.detection_count = 0
        self.dectect_accuracy = 30
        self.mission_started = False
        self.loiter_started = False

        self.hold_position = False
        self.hold_start_time = None
        self.hold_duration = 3.0
        self.current_position = None
        self.z = 0.0
        self.descent_altitude = 10
        self.Kpd = 0.5
        self.lost_detection_count = 0
        self.max_lost_detections = 10 

      
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')
        
        



    
    def try_arming(self):


        if self.current_mode != "AUTO.LOITER" and not self.armed:
            # Switch mode first
            self.get_logger().warn(f"Mode is {self.current_mode}, switching to AUTO.LOITER...")
            req_mode = SetMode.Request()
            req_mode.custom_mode = "AUTO.LOITER"
            req_mode.base_mode = 0

            future_mode = self.set_mode_client.call_async(req_mode)

            def mode_done_cb(fut):
                try:
                    if fut.result() and fut.result().mode_sent:
                        self.get_logger().info("AUTO.LOITER requested - waiting for confirmation...")
                    else:
                        self.get_logger().error("Failed to switch to AUTO.LOITER.")
                        self._ensure_retry_timer()
                except Exception as e:
                    self.get_logger().error(f"Mode request exception: {e}")
            future_mode.add_done_callback(mode_done_cb)

        if self.armed:
            self.get_logger().info("Drone already armed.")
            if self.retry_timer:
                self.retry_timer.cancel()
                self.retry_timer = None
            return
        else:
            self.get_logger().info("Drone not armed yet.")  
            self._ensure_retry_timer()
        # If already in AUTO.LOITER, try arming directly
        if self.current_mode == "AUTO.LOITER" and not self.armed:
            self.get_logger().info("Attempting to arm the drone...")
            self._send_arm_command()



    def _send_arm_command(self):
        req = CommandBool.Request()
        req.value = True
        future_arm = self.arming_client.call_async(req)

        def arm_done_cb(fut):
            try:
                res = fut.result()
                if res and getattr(res, "success", False):
                    self.armed = True
                    self.get_logger().info(f"Drone armed: {self.armed}, current mode: {self.current_mode}")
                    if self.retry_timer:
                        self.retry_timer.cancel()
                        self.retry_timer = None
                else:
                    self.get_logger().warn("Arming failed. Retrying...")
                    self._ensure_retry_timer()
            except Exception as e:
                self.get_logger().error(f"Arming request exception: {e}")
                self._ensure_retry_timer()

        future_arm.add_done_callback(arm_done_cb)



    def _ensure_retry_timer(self):
        if self.retry_timer is None:
            self.retry_timer = self.create_timer(1.0, self.try_arming)

    
    def state_callback(self, msg):
        self.current_mode = msg.mode
        self.armed = msg.armed  

        
        if self.initial_arm_attempt :
            self.try_arming()
            self.initial_arm_attempt  = False
        
        if msg.mode == "OFFBOARD" and not self.offboard_started and self.armed:
            self.offboard_started = True
            self.mission_started = False
            self.loiter_started = False
            self.get_logger().info("OFFBOARD mode ACTUALLY active!")
        if msg.mode == "AUTO.MISSION" and not self.mission_started and self.armed:
            self.mission_started = True
            self.offboard_started = False
            self.loiter_started = False
            self.get_logger().info("AUTO.MISSION mode ACTUALLY active!")
        if msg.mode == "AUTO.LOITER" and not self.loiter_started and not self.armed:
            self.loiter_started = True
            self.mission_started = False
            self.offboard_started = False
            self.get_logger().info("AUTO.LOITER mode ACTUALLY active!")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        self.current_position = pos
        _, _, self.current_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.z = pos.z
        if self.armed and not self.mission_started and not self.offboard_started:
            self.get_logger().info("Armed and in LOITER, switching to MISSION...")
            self.start_mission()
    
    
    def start_mission(self):
        if self.current_mode != "AUTO.MISSION":
            # self.get_logger().info("Starting mission by switching to AUTO.MISSION...")
            req = SetMode.Request()
            req.custom_mode = "AUTO.MISSION"
            req.base_mode = 0
            future = self.set_mode_client.call_async(req)

            def mission_mode_done_cb(fut):
                if fut.result() and fut.result().mode_sent:
                    pass
                    # self.get_logger().info("AUTO.MISSION requested - waiting for confirmation...")
                else:
                    self.get_logger().error("Failed to switch to AUTO.MISSION.")
            future.add_done_callback(mission_mode_done_cb)
    
    def image_callback(self, msg):
        if self.armed:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                height, width = cv_image.shape[:2]
                self.image_center = (width // 2, height // 2)
                # camera_tilt_angle = math.radians(60)  # 30 degrees tilt
                # camera_fov_vertical = math.radians(42)  # Adjust to your camera's actual vertical FOV

                # ground_offset = self.z * math.tan(camera_tilt_angle)
                # angular_offset = math.atan(ground_offset / self.z) if self.z > 0 else 0
                # pixels_per_radian_vertical = height / camera_fov_vertical
                # pixel_offset_y = angular_offset * pixels_per_radian_vertical

                # cx = width // 2
                # cy = (height // 2) + int(pixel_offset_y)
                # cy = max(0, min(cy, height - 1))  # Clamp to image boundaries

                # self.image_center = (cx, cy)
                self.detect_bullseye(cv_image)

                

                if self.stable_bullseye_center:
                    cv2.circle(cv_image, self.stable_bullseye_center, 2, (0, 255, 0), -1)
                    cv2.circle(cv_image, self.stable_bullseye_center, self.r, (0, 255, 0), 2) # type: ignore

                cv2.line(cv_image, (self.image_center[0]-20, self.image_center[1]),
                        (self.image_center[0]+20, self.image_center[1]), (255, 0, 0), 1)
                cv2.line(cv_image, (self.image_center[0], self.image_center[1]-20),
                        (self.image_center[0], self.image_center[1]+20), (255, 0, 0), 1)
                cv2.imshow('Bullseye Detector', cv_image)
                cv2.waitKey(1)

                
                if self.bullseye_detected:
                    dx = self.stable_bullseye_center[0] - self.image_center[0] # type: ignore
                    dy = self.stable_bullseye_center[1] - self.image_center[1] # type: ignore
                    # self.get_logger().info(f"Offset: dx={dx}, dy={dy}")
                    cv2.line(cv_image, (self.image_center[0], self.image_center[1]),
                        (self.stable_bullseye_center[0], self.stable_bullseye_center[1]), (0, 255, 0), 2) # type: ignore
                    
                    
                
                    if not self.offboard_started:
                        if not self.idle_vel_timer:
                            self.idle_vel_timer = self.create_timer(0.1, self.send_idle_velocity)
                            self.get_logger().info("Starting idle velocity timer...")  
                            self.hold_position = True 
                            
                        self.switch_to_offboard()
                        self.get_logger().info("Switching to OFFBOARD mode...")

                    
                if self.offboard_started and self.hold_position:
                        self.hold_position_mode()
                        return  
                
                if self.offboard_started and not self.hold_position and self.bullseye_detected:
                    dz = self.z-self.descent_altitude
                    if abs(dx) > self.threshold or abs(dy) > self.threshold or abs(dz) > 0.1:
                        self.send_velocity_command(dx, dy, dz)
                        # self.get_logger().info(f"Aligning to target: dz = {self.z}")
                    else:
                        self.send_velocity_command(0, 0, 0)    
                elif self.offboard_started and not self.bullseye_detected and not self.hold_position:
                    self.lost_detection_count += 1
                    self.get_logger().warn(f"Bullseye lost! Count: {self.lost_detection_count}/{self.max_lost_detections}")
                    
                    if self.lost_detection_count >= self.max_lost_detections:
                        self.get_logger().error("Bullseye lost for too long! Returning to MISSION...")
                        self.start_mission()
                        self.lost_detection_count = 0
                else:
                    # Reset counter when bullseye is detected
                    if self.stable_bullseye_center and self.lost_detection_count > 0:
                        self.lost_detection_count = 0
                                

                                
            except Exception as e:
                self.get_logger().error(f'Error processing image: {e}')
        

    def hold_position_mode(self):
        if self.hold_start_time is None:
            self.hold_start_time = self.get_clock().now()
            elapsed = 0.0
        else:
            elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds * 1e-9
        if elapsed < self.hold_duration:
            # Still holding - idle velocity is published by timer
            self.get_logger().info(f"Holding position... {elapsed:.1f}/{self.hold_duration}s")
            return
        else:
            # Hold complete, start alignment
            self.hold_position = False
            self.get_logger().info("Hold complete. Starting target alignment...")
            self.hold_start_time = None

        if self.idle_vel_timer:
            self.idle_vel_timer.cancel()
            self.idle_vel_timer = None
            self.get_logger().info("Stopping idle velocity timer...")
    

    def switch_to_offboard(self):
        if self.offboard_started:
            return
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        req.base_mode = 0
        future = self.set_mode_client.call_async(req)

        def callback(fut):
            if fut.result() and fut.result().mode_sent:
                self.get_logger().info("OFFBOARD requested - waiting for confirmation...")
            else:
                self.get_logger().error("Failed to switch to OFFBOARD.")
        future.add_done_callback(callback)



    def detect_bullseye(self, image):
        if self.armed:
        # self.get_logger().error("detect")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                                    param1=50, param2=40, minRadius=10, maxRadius=200)

            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                x, y, self.r = circles[0]
                
                if self.bullseye_center is None or \
                (abs(x - self.bullseye_center[0]) < self.dectect_accuracy and abs(y - self.bullseye_center[1]) < self.dectect_accuracy):
                    self.detection_count += 1
                else:
                    self.detection_count = 1
                self.bullseye_center = (x, y)
                if self.detection_count >= self.min_detections:
                    self.bullseye_detected = True
                    self.stable_bullseye_center = (x, y)
            else:
                self.bullseye_detected = False
                self.detection_count = 0
                self.bullseye_center = None
                self.stable_bullseye_center = None
                self.r = 0.0
            
        else:
            return

        

    def send_velocity_command(self, dx, dy, dz):
        cmd = Twist()

        # Proportional control in image frame
        vx_body = -dy * self.Kp
        vy_body = -dx * self.Kp
        vz = -dz * self.Kpd

        # Apply yaw correction (rotate body-frame to world-frame velocity)
        yaw = self.current_yaw
        vx = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
        vy = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)

        # Clip velocities
        vx = max(min(vx, self.mvel), -self.mvel)
        vy = max(min(vy, self.mvel), -self.mvel)
        


        vu = max(min(vz, self.mvel), -self.mvel)

        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vu
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)


    def send_idle_velocity(self):
        if self.idle_vel_timer:
            self.cmd_vel_pub.publish(Twist())
            # self.get_logger().error(f"i am being called{self.i}")
            # self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = BullseyeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
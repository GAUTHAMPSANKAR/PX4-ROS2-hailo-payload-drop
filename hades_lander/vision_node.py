import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys # <-- Make sure sys is imported
from collections import deque
from collections import Counter # <-- NEW: Import Counter for counting
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration # <-- NEW: Import Duration for timed saving
from .vision_utils import BullseyeProcessor

class VisionNode(Node):
    def __init__(self):
        super().__init__('bullseye_vision_node')
        self.declare_parameters(
            namespace='',
            parameters=[ #type: ignore [arg]
                ('hades_vision.detection_confidence', 0.7),
                ('hades_vision.max_frames_lost', 15),
                ('hades_mission.detection_history_size', 30),
                ('hades_control.exponential_weight', -6.0),
                ('hades_vision.save_detections', False),
                ('hades_vision.save_path', '/tmp/hades_detections'),
                ('hades_vision.save_mode', 'target_only'), 
                ('hades_vision.save_interval_sec', 1.0) 
            ]
        )
        
        self.CLASS_LABELS = ['empty','circle']
        
        self.min_confidence = self.get_parameter('hades_vision.detection_confidence').value
        self.max_frames_lost = self.get_parameter('hades_vision.max_frames_lost').value
        detection_history_size = self.get_parameter('hades_mission.detection_history_size').value
        self.eweight = self.get_parameter('hades_control.exponential_weight').value
        
        self.save_detections = self.get_parameter('hades_vision.save_detections').value
        self.save_path = self.get_parameter('hades_vision.save_path').value
        self.frame_count = 0
        
        
        self.save_mode = self.get_parameter('hades_vision.save_mode').value
        save_interval_sec = self.get_parameter('hades_vision.save_interval_sec').value
        self.save_interval = Duration(seconds=save_interval_sec)
        self.last_save_time = self.get_clock().now() 
        if self.save_detections:
            os.makedirs(self.save_path, exist_ok=True)

        self.get_logger().info("Bullseye Vision Node starting...")
        video_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            video_qos_profile)
        self.error_pub = self.create_publisher(Point, '/bullseye/pixel_error', 10)
        self.debug_pub = self.create_publisher(Image, '/bullseye/debug_image', 1)


        self.bridge = CvBridge()
        self.image_center = None
        self.detection_history = deque(maxlen=detection_history_size)
        self.stable_target = None
        self.target_confidence = 0.0
        self.last_known_target = None
        self.frames_since_detection = 0

        TARGET_NAME = "circle"
        try:
            target_idx = self.CLASS_LABELS.index(TARGET_NAME)
        except ValueError:
            self.get_logger().error(f"FATAL: Target '{TARGET_NAME}' not in CLASS_LABELS list!")
            sys.exit(1) 

        self.vision_processor = BullseyeProcessor(
            target_index=target_idx,  
            min_confidence=self.min_confidence,
        )

        self.vision_processor.start()
        
        self.get_logger().info(f"Hailo processor started. Looking for '{TARGET_NAME}' (Index: {target_idx}).")
    
    def image_callback(self, msg):
        current_frame_time = self.get_clock().now()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        if self.image_center is None:
            height, width = cv_image.shape[:2]
            self.image_center = (width // 2, height // 2)
        detected_target, all_raw_detections = self.vision_processor.detect_bullseye(cv_image)
        
        self.detection_history.append(detected_target)
        self._update_stable_target()
        
        if cv_image is not None and self.image_center is not None:
            display_img = cv_image.copy()
            cv2.drawMarker(
                display_img, self.image_center, (255, 0, 0),
                cv2.MARKER_CROSS, 20, 2
            )
            
            if all_raw_detections:                
                for det in all_raw_detections:                    
                    class_id, (x1, y1, x2, y2), conf = det
                    
                    if conf < 0.5: 
                        continue 

                    try:
                        label_name = self.CLASS_LABELS[class_id]
                    except IndexError:
                        label_name = "UNKNOWN"

                    color = (0, 255, 255) 
                    if class_id == self.vision_processor.target_index: 
                        color = (0, 165, 255)
                        
                    cv2.rectangle(display_img, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(
                        display_img, f"{label_name} {conf:.2f}", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
                    )
                
                label_ids = [det[0] for det in all_raw_detections if det[2] >= 0.25]
                label_names = []
                for class_id in label_ids:
                    try:
                        label_names.append(self.CLASS_LABELS[class_id])
                    except IndexError:
                        label_names.append("UNKNOWN")
                
                label_counts = Counter(label_names) 
                
                y_offset = 30 
                for label, count in label_counts.items():
                    text = f"{label}: {count}"
                    cv2.putText(display_img, text, (10, y_offset), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    y_offset += 25 

            if self.stable_target and self.target_confidence >= self.min_confidence:
                x, y, r = self.stable_target
                cv2.circle(display_img, (x, y), r, (0, 255, 0), 2)
                cv2.circle(display_img, (x, y), 3, (0, 255, 0), -1)
                cv2.line(display_img, self.image_center, (x, y), (0, 255, 0), 2)
            elif self.last_known_target:
                x, y, r = self.last_known_target
                cv2.circle(display_img, (x, y), r, (255, 255, 0), 2)
  
            debug_msg = self.bridge.cv2_to_imgmsg(display_img, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
            
            if self.save_detections and all_raw_detections:
                save_this_frame = False
                
                if self.save_mode == 'all':
                    save_this_frame = True
                
                elif self.save_mode == 'target_only':
                    if detected_target is not None:
                        save_this_frame = True
                
                elif self.save_mode == 'interval':
                    if (current_frame_time - self.last_save_time) >= self.save_interval:
                        save_this_frame = True
                        self.last_save_time = current_frame_time 

                if save_this_frame:
                    filename = os.path.join(self.save_path, f"detection_{self.frame_count:06d}.png")
                    cv2.imwrite(filename, display_img)
                    self.frame_count += 1

        error_msg = Point()
        if self.stable_target and self.target_confidence >= self.min_confidence:
            x, y, _ = self.stable_target
            dx = float(x - self.image_center[0])
            dy = float(y - self.image_center[1])

            dx_norm = dx / cv_image.shape[1]
            dy_norm = dy / cv_image.shape[0]
            
            error_msg.x = dx_norm
            error_msg.y = dy_norm
            error_msg.z = 1.0
        else:
            error_msg.x = 0.0
            error_msg.y = 0.0
            error_msg.z = -1.0
            
        self.error_pub.publish(error_msg)

    def _update_stable_target(self):
        valid_detections = [d for d in self.detection_history if d is not None]
        
        if not valid_detections:
            self.target_confidence = 0.0
            self.frames_since_detection += 1
            
            if self.frames_since_detection > self.max_frames_lost:
                self.last_known_target = self.stable_target
                self.stable_target = None
            return
            
        self.target_confidence = len(valid_detections) / len(self.detection_history)
        self.frames_since_detection = 0
        
        weights = np.exp(np.linspace(self.eweight, 0, len(valid_detections)))
        weights /= weights.sum()
        
        x_avg = sum(w * d[0] for w, d in zip(weights, valid_detections))
        y_avg = sum(w * d[1] for w, d in zip(weights, valid_detections))
        r_avg = sum(w * d[2] for w, d in zip(weights, valid_detections))
        
        self.stable_target = (int(x_avg), int(y_avg), int(r_avg))


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.get_logger().info("Shutting down Hailo processor...")
        vision_node.vision_processor.shutdown()
        vision_node.get_logger().info("Destroying node...")
        vision_node.destroy_node()
        rclpy.shutdown()
        print("Vision node shut down successfully.")

if __name__ == '__main__':
    main()
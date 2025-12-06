import cv2
import numpy as np
import threading
import time
import sys

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

try:
    import hailo
    hailo_object_t = hailo.hailo_object_t
    print("'hailo' binding loaded.")
except ImportError:
    print("'hailo' binding not found. Make sure hailo-tappas-core-python-binding is installed.", file=sys.stderr)
    sys.exit(1)

# # --- YOUR LABELS LIST ---
# # This list maps the integer index (0, 1, ...) to your string names.
# CLASS_LABELS = [
#     'flood',  # Index 0
#     'bike'    # Index 1
# ]


class BullseyeProcessor:
    """
    This class handles all GStreamer and Hailo-8L interaction.
    It provides a 'detect_bullseye' function that returns both the
    best target (for control) and a list of all detections (for visualization).
    
    (This class logic is unchanged and still uses 'target_index')
    """
    def __init__(self,
                 target_index, 
                 min_confidence=0.90,
                 net_width=640,
                 net_height=640,
                 postproc_so="/usr/lib/aarch64-linux-gnu/hailo/tappas/post_processes/libyolo_hailortpp_post.so",
                 hef_path="/home/nyx/yolov8s_circle.hef"):
        
        print(f"Initializing BullseyeProcessor for target index: {target_index}")
        
        self.target_index = int(target_index) 
        self.min_confidence = float(min_confidence)
        self.net_w = int(net_width)
        self.net_h = int(net_height)
        self.hef_path = hef_path
        self.postproc_so = postproc_so

        self._det_lock = threading.Lock()
        self._latest_det = None
        
        self._all_detections = []

        Gst.init(None)
        
        self.pipeline_string = f"""
            appsrc name=src is-live=true format=time do-timestamp=true max-bytes=0 !
            videoconvert !
            videoscale !
            video/x-raw,format=RGB,width={self.net_w},height={self.net_h},pixel-aspect-ratio=1/1 !
            queue leaky=2 max-size-buffers=2 !
            hailonet hef-path={self.hef_path} force-writable=true !
            queue leaky=2 max-size-buffers=2 !
            hailofilter so-path={self.postproc_so} function-name=filter !
            queue leaky=2 max-size-buffers=2 !
            appsink name=yolo_sink emit-signals=true max-buffers=1 drop=true sync=false
        """
        
        print("Creating GStreamer pipeline...")
        self._pipeline = Gst.parse_launch(self.pipeline_string)
        
        if not self._pipeline:
            raise RuntimeError("Failed to create Hailo pipeline from string.")
            
        print("Pipeline created successfully.")
        
        self._appsrc = self._pipeline.get_by_name("src")
        self._appsink = self._pipeline.get_by_name("yolo_sink")
        
        if not self._appsrc or not self._appsink:
            raise RuntimeError("Failed to get 'src' or 'yolo_sink' from pipeline.")

        caps_str = f"video/x-raw,format=RGB,width={self.net_w},height={self.net_h},framerate=0/1"
        caps = Gst.Caps.from_string(caps_str)
        self._appsrc.set_property("caps", caps)

        self._bus = None
        self._loop = None
        self._loop_thread = None
        self._t0 = time.time() 


    def start(self):
        print("Starting GStreamer pipeline thread...")
        
        self._appsink.connect("new-sample", self._on_new_sample)
        self._bus = self._pipeline.get_bus()
        self._bus.add_signal_watch()
        self._bus.connect("message", self._on_bus_message)

        self._pipeline.set_state(Gst.State.PLAYING)
        self._loop = GLib.MainLoop()
        self._loop_thread = threading.Thread(target=self._loop.run, daemon=True)
        self._loop_thread.start()
        
        print("Pipeline is running.")
        self._t0 = time.time()

    def _on_bus_message(self, bus, message):
        """Handles messages from the GStreamer bus (e.g., errors)."""
        mtype = message.type
        
        if mtype == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"[GStreamer Bus] ERROR: {err} ({debug})", file=sys.stderr)
        elif mtype == Gst.MessageType.EOS:
            print("[GStreamer Bus] End-Of-Stream (EOS) received.")
        
        return True

    def _on_new_sample(self, appsink):
        """
        Callback function for the 'new-sample' signal from the appsink.
        This is where we get our detections from the Hailo.
        """
        
        sample = appsink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        if not buf:
            return GDst.FlowReturn.OK

        try:
            roi = hailo.get_roi_from_buffer(buf)
            detections = roi.get_objects_typed(hailo_object_t.HAILO_DETECTION)
        except Exception as e:
            print(f"[Hailo Parse] Could not get ROI from buffer: {e}", file=sys.stderr)
            return Gst.FlowReturn.OK

        best_detection = None
        best_confidence = 0.0
        current_frame_all_detections = []

        for det in detections:
            try:
                class_id = det.get_class_id()
                
                confidence = float(det.get_confidence())
                bbox = det.get_bbox()
                
                cx_norm = bbox.xmin() + (bbox.width() / 2.0)
                cy_norm = bbox.ymin() + (bbox.height() / 2.0)
                w_norm = bbox.width()
                h_norm = bbox.height()

                current_frame_all_detections.append(
                    (class_id, cx_norm, cy_norm, w_norm, h_norm, confidence)
                )
                
                if class_id == self.target_index and confidence >= self.min_confidence:
                    
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_detection = (cx_norm, cy_norm, w_norm, h_norm, confidence)
                        
            except Exception as e:
                print(f"Error parsing a detection: {e}", file=sys.stderr)
                continue

        with self._det_lock:
            self._latest_det = best_detection
            self._all_detections = current_frame_all_detections
            
        return Gst.FlowReturn.OK

    def detect_bullseye(self, image):
            if image is None:
                return (None, [])
                
            H, W = image.shape[:2]
            resized_image = cv2.resize(image, (self.net_w, self.net_h), 
                                    interpolation=cv2.INTER_LINEAR)
            rgb = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
            
            data = rgb.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)

            now_ns = int((time.time() - self._t0) * 1e9)
            buf.pts = now_ns
            buf.dts = now_ns
            buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 30)
            
            try:
                self._appsrc.emit("push-buffer", buf)
            except Exception as e:
                print(f"[GStreamer] push-buffer failed: {e}", file=sys.stderr)
                return (None, [])
            with self._det_lock:
                det_best = self._latest_det
                all_dets_norm = self._all_detections

            pixel_target = None
            if det_best is not None:
                cx_norm, cy_norm, w_norm, h_norm, conf = det_best
                x_px = int(cx_norm * W)
                y_px = int(cy_norm * H)
                r_px = int(0.5 * min(w_norm * W, h_norm * H))
                pixel_target = (x_px, y_px, max(1, r_px))

            pixel_all_detections = []
            for det in all_dets_norm:
                class_id, cx_norm, cy_norm, w_norm, h_norm, conf = det
                
                x_center_px = int(cx_norm * W)
                y_center_px = int(cy_norm * H)
                width_px = int(w_norm * W)
                height_px = int(h_norm * H)
                
                x1 = int(x_center_px - width_px / 2)
                y1 = int(y_center_px - height_px / 2)
                x2 = int(x_center_px + width_px / 2)
                y2 = int(y_center_px + height_px / 2)

                pixel_all_detections.append(
                    (class_id, (x1, y1, x2, y2), conf)
                )

            return (pixel_target, pixel_all_detections)

    def shutdown(self):
        print("Shutting down pipeline...")
        try:
            if self._pipeline:
                self._pipeline.set_state(Gst.State.NULL)
            if self._bus:
                self._bus.remove_signal_watch()
            if self._loop and self._loop.is_running():
                self._loop.quit()
        except Exception as e:
            print(f"Error during shutdown: {e}")

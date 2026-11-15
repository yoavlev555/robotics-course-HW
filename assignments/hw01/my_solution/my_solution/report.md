# HW01 Report: Camera-Guided Reaching with a Two-Joint Arm

## 1. Marker Detection

The incoming RGB frame is converted from BGR to the HSV color space using cv2.cvtColor. HSV was chosen over BGR because it separates color identity (Hue) from illumination (Value), making the detection robust to lighting variation and partial shadowing in the scene - a property BGR does not offer, since all three of its channels change jointly under lighting changes.

Because OpenCV represents Hue on a circular scale of 0–179, the color red - which lies at the wrap-around point of the hue wheel - cannot be captured by a single contiguous range. Two ranges are therefore defined, one near each end of the scale ([0,10] and [160,179]), each with a minimum saturation/value of 100 to reject washed-out, non-vivid pixels. The two resulting binary masks are combined with a logical OR to produce the final red-pixel mask.

The marker’s pixel location is computed as the centroid of all pixels flagged in the mask: np.nonzero(mask) returns the (row, column) indices of every non-zero pixel, and their mean gives the center-of-mass coordinates (u, v). This is mathematically equivalent to a moments-based centroid, since every mask pixel carries equal weight (0 or 255). If no pixel is flagged (len(xs) == 0), the frame is treated as “marker not visible”: a warning is logged and the callback returns immediately, preventing a division-by-zero or downstream processing of an undefined pixel location.

## 2. Depth Lookup & 3D Reconstruction

A 2D pixel location alone does not determine a 3D position, since it conflates distance and size - a nearby small object and a distant large one can project to the same pixel. The corresponding depth frame supplies the missing dimension: the distance from the camera to whatever is visible at each pixel.

The marker’s depth is read from the depth image at (v_px, u_px) - the row/column indices obtained by rounding and clamping (u, v) to the image bounds, which prevents an out-of-range index from a rounding error. The camera’s intrinsic parameters (fx, fy, cx, cy), published in CameraInfo.K, are then used with the standard pinhole camera model to back-project the pixel into a 3D point in the camera’s optical frame:

Z = depth_value           
X = (u - cx) * Z / fx           
Y = (v - cy) * Z / fy

Before this projection is trusted, the depth reading is validated: it must be finite (depth sensors report NaN/Inf when a measurement fails, e.g. on reflective surfaces), strictly positive, and below a scene-specific sanity bound (MAX_VALID_DEPTH = 5.0 m), which rejects sensor noise or out-of-range artifacts. If the reading fails any check, the frame is discarded before it can propagate an invalid position downstream to the TF transform or the arm’s motion command.


## 3. TF Direction & Timing

The transform is looked up as lookup_transform('base_link', camera_frame, stamp), following the tf2 convention lookup_transform(target_frame, source_frame, time): since the marker’s position is expressed in the camera’s optical frame and needs to be expressed relative to the robot’s base for motion planning, base_link is the target and the camera frame is the source. The lookup uses the image’s own timestamp rather than the current time, so that the returned transform reflects the robot’s pose at the moment the measurement was taken, not its current (possibly different) pose; a short timeout (0.1 s) absorbs minor delivery jitter in the TF buffer. LookupException, ConnectivityException, and ExtrapolationException are caught together, since each represents the same outcome from the caller’s perspective - no valid transform is currently available - and is handled by discarding the frame.

Two independent staleness checks guard against acting on outdated data. observation_age compares the simulation clock to the message timestamp, rejecting observations whose underlying measurement is too old relative to simulated time. camera_data_age instead compares real wall-clock time to the last time any camera frame was received, independent of the simulation clock. The second check is necessary because if simulation time itself stalls (e.g. the simulator pauses), a sim-time-based staleness check cannot detect it - both “now” and the stored timestamp freeze together - whereas the wall-clock check continues to advance and correctly flags the stall.

## 4. Failure Handling

Beyond the specific validity checks on depth and TF transforms (covered above), the node includes several additional layers of defense against failure.

Image conversion from the ROS message to OpenCV format is wrapped in a try/except CvBridgeError block: a single malformed or unexpectedly-encoded frame is logged and skipped, rather than raising an unhandled exception that would crash the entire node.

Since camera callbacks fire continuously and independently of how long a previous arm motion takes to complete, a self.motion_busy flag (combined with an arm.ready() check) guards against issuing overlapping or conflicting motion commands while a previous one is still in flight. This flag is cleared unconditionally in the on_motion_done completion callback - regardless of whether the motion succeeded or failed - since clearing it only on success would permanently lock the robot out of future commands after any single motion failure.

Finally, shutdown is handled explicitly in main(): a try/except KeyboardInterrupt around rclpy.spin is paired with a finally block that cancels any in-flight arm motion and tears down the node (destroy_node, rclpy.try_shutdown()), ensuring that an interrupt (e.g. Ctrl+C) does not leave the arm mid-motion or raise a shutdown error from a redundant rclpy.shutdown() call.

## 5. Results

### Preset 1

With the simulation launched under `preset:=1` and `check_reach --preset 1` running
alongside `reach_node`, the pipeline produced the following:

- **Detected marker pixel:** (u=202.1, v=105.0)
- **3D position in `camera_optical_frame`:** (X=0.143, Y=-0.051, Z=0.940) m
- **3D position in `base_link`:** (X=0.351, Y=-0.143, Z=0.060) m
- **Commanded hover target (after +0.08 m offset):** z ≈ 0.140 m, matching the
  fixed tip height required by the arm.
- **`check_reach` result:** `PASS - tip error 0.0029 m, held 2 simulated seconds;
  live contact streams clear`

The tip settled within 3 mm of the required hover point, well under the 0.02 m
tolerance, with no platform or marker contact detected throughout the hold period.

### Preset 2

With the simulation restarted under `preset:=2` and `check_reach --preset 2`
running alongside `reach_node`:

- **Detected marker pixel:** (u=156.0, v=94.0)
- **3D position in `camera_optical_frame`:** (X=-0.014, Y=-0.088, Z=0.941) m
- **3D position in `base_link`:** (X=0.388, Y=0.014, Z=0.059) m
- **`check_reach` result:** `PASS — tip error 0.0022 m, held 2 simulated seconds;
  live contact streams clear`

**Robustness observations from this run:**

1. *Transient depth outlier absorbed by the busy guard.* Between two consecutive
   detections at the same pixel, the looked-up depth briefly jumped from 0.941 m
   to 0.791 m (base-frame Z from 0.059 m to 0.209 m) — most likely the arm itself
   momentarily passing through that pixel's line of sight during motion. Because
   `motion_busy` was `True` at that instant, this outlier reading was never used
   to command a new goal, demonstrating that the single-motion guard also
   incidentally protects against transient depth noise during arm movement.
2. *Expected self-occlusion after arrival.* Once the tip settled above the
   marker, subsequent frames logged `no red marker pixels detected in this
   frame` — the arm itself blocks the overhead camera's view of the marker it is
   now hovering over. This is correctly handled as a missing-detection case
   (warning logged, frame skipped) rather than as an error.

### Preset 3

With the simulation restarted under `preset:=3` and `check_reach --preset 3`
running alongside `reach_node`:

- **Detected marker pixel (stabilized):** (u=115.0, v=110.0)
- **3D position in `camera_optical_frame`:** (X=-0.153, Y=-0.034, Z=0.940) m
- **3D position in `base_link`:** (X=0.334, Y=0.153, Z=0.060) m
- **`check_reach` result:** `PASS — tip error 0.0021 m, held 2 simulated seconds;
  live contact streams clear`

The same transient depth outlier pattern observed in Preset 2 (Z momentarily
reading ~0.765 m instead of ~0.940 m at an unchanged pixel) occurred twice
during this run as well, and was again absorbed without effect by the
`motion_busy` guard — suggesting this is a systematic, momentary self-occlusion
artifact of the arm passing through the camera's line of sight, rather than a
one-off fluke.

### Summary

| Preset | Detected pixel (u,v) | base_link position (X,Y,Z) [m] | check_reach result |
|---|---|---|---|
| 1 | (202.1, 105.0) | (0.351, -0.143, 0.060) | PASS, tip error 0.0029 m |
| 2 | (156.0, 94.0)  | (0.388, 0.014, 0.059)  | PASS, tip error 0.0022 m |
| 3 | (115.0, 110.0) | (0.334, 0.153, 0.060)  | PASS, tip error 0.0021 m |

All three presets passed with tip error well under the required 0.02 m
tolerance, using the same unmodified implementation and no preset-specific
logic - confirming the pipeline derives the target purely from camera data.

## 6. Build & Run Commands

### a. Build the shared environment (once)

cd ~/robotics-course-HW/assignments/hw01/environment           
colcon build --symlink-install --base-paths src --executor sequential           
source install/setup.bash

### b. Build my_solution (once)
cd ~/robotics-course-HW/assignments/hw01/my_solution           
colcon build --symlink-install

### c. Run (repeated for preset 1, 2, and 3)

#### Terminal 1 - simulation:

cd ~/robotics-course-HW/assignments/hw01/environment           
source install/setup.bash           
ros2 launch hw01_sim simulation.launch.py preset:=<1|2|3>

#### Terminal 2 - public checker (started before our node, per preset):

cd ~/robotics-course-HW/assignments/hw01/environment           
source install/setup.bash           
ros2 run hw01_sim check_reach --preset <1|2|3>

#### Terminal 3 - our node:

cd ~/robotics-course-HW/assignments/hw01/environment           
source install/setup.bash           
cd ~/robotics-course-HW/assignments/hw01/my_solution           
source install/setup.bash           
ros2 run my_solution reach_node           

Between presets: Ctrl+C in Terminal 3, then Ctrl+C in Terminal 1, wait for
full exit, then relaunch Terminal 1 with the next preset before repeating
Terminals 2–3.



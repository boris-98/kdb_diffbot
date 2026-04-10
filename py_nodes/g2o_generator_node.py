#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros
import numpy as np
from math import sin, cos, atan2, sqrt, pi
import yaml
import os 
import time


def wrap_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

def yaw_to_quat(yaw):
    qx = 0.0
    qy = 0.0
    qz = sin(yaw / 2.0)
    qw = cos(yaw / 2.0)
    return (qx, qy, qz, qw)

def quat_to_yaw(q):
    return atan2(
        2.0*(q.w*q.z + q.x*q.y),
        1.0 - 2.0*(q.y*q.y + q.z*q.z)
    )

def rpy_to_quat(roll, pitch, yaw):
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return (qx, qy, qz, qw)

def tf_to_matrix(translation, rotation):
    tx, ty, tz = translation
    qx, qy, qz, qw = rotation

    # Convert quaternion to rotation matrix
    R = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])

    M = np.eye(4)
    M[0:3, 0:3] = R
    M[0:3, 3] = [tx, ty, tz]

    return M

def tfmsg_to_matrix(t):
    tr = t.transform.translation
    rot = t.transform.rotation
    return tf_to_matrix(
        (tr.x, tr.y, tr.z),
        (rot.x, rot.y, rot.z, rot.w)
    )

def invert_homogen(M):
    R = M[:3, :3]
    t = M[:3, 3]
    M_inv = np.eye(4)
    M_inv[:3, :3] = R.T
    M_inv[:3, 3] = -R.T @ t
    
    return M_inv


class G2oGeneratorNode(Node):
    def __init__(self):
        super().__init__('g2o_generator_node')
        self.declare_parameter('tag_map_yaml', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('chassis_frame', 'chassis')
        self.declare_parameter('camera_optical_frame', 'camera_link_optical')   
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('use_tf_for_tags', True)
        self.declare_parameter('default_meas_var_xy', 0.01) # m^2
        self.declare_parameter('default_meas_var_theta', 0.05) # rad^2
        self.declare_parameter('alpha1', 0.01)  # motion noise related to forward velocity
        self.declare_parameter('alpha2', 0.01)  # motion noise related to angular velocity
        self.declare_parameter('alpha3', 0.01)  # motion noise related to forward velocity
        self.declare_parameter('alpha4', 0.01)  # motion noise related to angular velocity
        
        
        # G2o state
        self.mu = np.zeros(3)  # [x, y, theta] initial pose estimate
        self.pose_id = 0
        self.prev_pose_id = None
        self.prev_mu = None
        self.landmark_ids = {}  # tag_id -> landmark_id (vertex_id) in g2o
        self.next_landmark_id = 10000  # start landmark IDs from 10000 to avoid collision with pose IDs
        self.g2o_file = open('generated_graph.g2o', 'w')  # Output g2o file
        # Write first pose as fixed prior
        self.write_vertex_se2(self.pose_id, self.mu)
        self.g2o_file.write(f'FIX {self.pose_id}\n')
        self.prev_pose_id = self.pose_id
        self.prev_mu = self.mu.copy()
        # Following is for ground truth from Gazebo [SIM]
        self.gtruth_pose = None
        self.gtruth_file = open('groundtruth.txt', 'w')

        # Frames
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value    # ili get_parameter_value().string_value
        self.chassis_frame = self.get_parameter('chassis_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # Last odom
        self.last_odom_msg = None
        self.last_odom_time = None

        # Motion noise parameters
        self.alpha1 = self.get_parameter('alpha1').value
        self.alpha2 = self.get_parameter('alpha2').value
        self.alpha3 = self.get_parameter('alpha3').value
        self.alpha4 = self.get_parameter('alpha4').value

        # Measurement default variances
        self.default_var_xy = float(self.get_parameter('default_meas_var_xy').value)
        self.default_var_theta = float(self.get_parameter('default_meas_var_theta').value)

        # Detected tags storage (capturing TF at detection time to avoid timing skew)
        self.pending_measurements = []        # list of (tag_id, TransformStamped)
        self.processed_tags_per_pose = set()  # (pose_id, tag_id) pairs already written
        # Landmark outlier rejection state
        self.landmark_observations_global = {}  # tag_id -> list of predicted global [x,y] positions
        self.pending_landmarks = {}             # tag_id -> (z, predicted_global, margin, pose_id) buffered first obs
        
        # # Load global poses for tags from YAML file
        # tag_map_yaml = self.get_parameter('tag_map_yaml').value
        # self.get_logger().info(f'Loading tag map from: {tag_map_yaml}')
        # self.tag_map = {} # tag_id -> 4x4 homogenous transform matrix (map->tag)
        # if tag_map_yaml and os.path.isfile(tag_map_yaml):
        #     with open(tag_map_yaml, 'r') as f:
        #         tag_map_data = yaml.safe_load(f)
        #         # expecting format: { id: {x:..., y:..., z:..., yaw:...} }
        #         for tid, info in tag_map_data.items():
        #             x = float(info.get('x', 0.0))
        #             y = float(info.get('y', 0.0))
        #             z = float(info.get('z', 0.0))
        #             yaw = float(info.get('yaw', 0.0))
        #             roll = float(info.get('roll', 0.0))
        #             pitch = float(info.get('pitch', 0.0))
        #             self.get_logger().info(f'Loaded tag {tid} at x:{x} y:{y} z:{z} yaw:{yaw}')
        #             q = rpy_to_quat(roll, pitch, yaw)
        #             self.tag_map[str(tid)] = tf_to_matrix((x, y, z), q)     # str(id) jer si u yaml stavio kao stringove kljuceve (tid_0, itd.)
        #             #self.get_logger().info(f'tag_map[{str(tid)}]: {self.tag_map[str(tid)]}')
        #     self.get_logger().info(f'Loaded {len(self.tag_map)} tags from {tag_map_yaml}')
        # else:
        #     self.get_logger().info('No tag YAML provided or file not found - will try TF lookups for map->tag (if available)')
        
        # TF listeners / broadcasters
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_static_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers and Publishers
        self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 50)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detections_callback, 10)
        # Following is for ground truth from Gazebo [SIM]
        self.create_subscription(PoseStamped, '/model/my_bot/pose', self.gtruth_callback, 10)

        # Main timer
        self.timer = self.create_timer(1, self.timer_callback) # 1 Hz SLAM Graph generation (first param is timer period)

    def odom_callback(self, msg: Odometry):
        # Store the last odometry message (velocities se integrale)
        self.last_odom_msg = msg
        # Extract twist from odom_msg and update mu according to motion model
        t_now = self.last_odom_msg.header.stamp
        sec = t_now.sec + t_now.nanosec * 1e-9
        
        if self.last_odom_time is None:
            self.last_odom_time = sec
            return
        dt = sec - self.last_odom_time
        
        if dt <= 0:
            return
        self.last_odom_time = sec

        # Need only forward vel + yaw for 2D diff bot
        v = self.last_odom_msg.twist.twist.linear.x
        omega = self.last_odom_msg.twist.twist.angular.z

        theta = self.mu[2]
        if abs(omega) < 1e-6:
            # very small rotation -> straight line
            dx = v * dt * cos(theta)
            dy = v * dt * sin(theta)
            dtheta = 0.0
        else:
            # better behaved version (handles negative v naturally)
            dtheta = omega * dt
            dx = (v / omega) * (sin(theta + dtheta) - sin(theta))
            dy = (v / omega) * (-cos(theta + dtheta) + cos(theta))

        # Update mu
        self.mu[0] += dx
        self.mu[1] += dy
        self.mu[2] =  wrap_angle(self.mu[2] + dtheta)
        #self.get_logger().info(f'Predicted mu = [{self.mu[0]:.2f}, {self.mu[1]:.2f}, {self.mu[2]:.2f}] from odom v={v:.2f} omega={omega:.2f} dt={dt:.2f}')
        # self.last_odom_msg = msg
        # p = msg.pose.pose.position
        # q = msg.pose.pose.orientation
        # self.mu[0] = p.x
        # self.mu[1] = p.y
        # self.mu[2] = quat_to_yaw(q)

    def detections_callback(self, msg: AprilTagDetectionArray):
        if len(msg.detections) == 0:
            self.pending_measurements = []
            return
        
        # Reject measurements during fast rotation
        # because the TF chain base_link->camera and camera->tag may be from different instants
        if self.last_odom_msg is not None:
            omega = abs(self.last_odom_msg.twist.twist.angular.z)
            if omega > 0.3:  # rad/s threshold
                self.get_logger().info(f'Skipping tag detections during fast rotation (omega={omega:.2f})', throttle_duration_sec=1.0)
                self.pending_measurements = []
                return

        measurements = []        
        for det in msg.detections:
            tag_id = f"tag_{det.id}"
            try:
                # Capture TF now at detection time, not later in the timer.
                # This ensures base_link->camera and camera->tag are from the same instant.
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, tag_id, rclpy.time.Time())
                measurements.append((tag_id, t, det.decision_margin))
                if (det.decision_margin < 30):
                    self.get_logger().info(f'Low confidence detection for {tag_id} with margin {det.decision_margin:.2f}', throttle_duration_sec=2.0)
            except Exception as e:
                self.get_logger().info(f"TF lookup failed for {tag_id} in detection callback: {e}")
        self.pending_measurements = measurements
    
    # listens on /model/my_bot/pose published by gazebo plugin for ground truth    
    def gtruth_callback(self, msg: PoseStamped):
        self.gtruth_pose = msg

    # Main loop
    def timer_callback(self):
        # 1. Odometry
        if self.last_odom_msg is not None:
            self.generate_from_odom()
        else:
            self.get_logger().warning('No odometry messages received yet!')


        # 2. Measurement
        if not self.pending_measurements:
            self.get_logger().info('No tags detected for correction step.', throttle_duration_sec=2.0)
            self.get_logger().info(f'mu = [{self.mu[0]:.2f}, {self.mu[1]:.2f}, {self.mu[2]:.2f}]', throttle_duration_sec=2.0)
        else:
            for tag_id, t, margin in self.pending_measurements:
                # Skip duplicate observations from the same pose
                key = (self.pose_id, tag_id)
                if key in self.processed_tags_per_pose:
                    continue
                self.processed_tags_per_pose.add(key)

                T_base_tag = tfmsg_to_matrix(t)
                    
                # Relative measurement in robot frame (SE2: x, y, theta)
                x_rel = T_base_tag[0, 3]
                y_rel = T_base_tag[1, 3]
                theta_rel = atan2(T_base_tag[1, 0], T_base_tag[0, 0])
                z = np.array([x_rel, y_rel, theta_rel])
                
                # Predict where this observation places the landmark in global frame
                theta = self.mu[2]
                R = np.array([[cos(theta), -sin(theta)],
                                [sin(theta), cos(theta)]])
                predicted_global_xy = self.mu[0:2] + R @ z[0:2]
                predicted_global_theta = wrap_angle(theta + theta_rel)
                predicted_global_pos = predicted_global_xy  # XY only for consistency checks
                                
                # if tag_id not in self.landmark_ids: # create landmark vertex if tag is first time seen
                #     # If margin is too low, don't use this detection to create a landmark (it will likely be an outlier and can mess up the graph optimization)
                #     if margin < 20:
                #         self.get_logger().info(f'Skipping low confidence detection for {tag_id} with margin {margin:.2f}', throttle_duration_sec=2.0)
                #         continue
                #     landmark_vertex_id = self.next_landmark_id
                #     self.landmark_ids[tag_id] = self.next_landmark_id
                #     self.next_landmark_id += 1
                    
                #     self.landmark_observations_global[tag_id] = [predicted_global_pos.copy()]    # save global position estimate for outlier rejection in future detections
                #     self.write_vertex_xy(landmark_vertex_id, predicted_global_pos)
                if tag_id not in self.landmark_ids:
                    # --- UNSEEN LANDMARK: use pending buffer to require 2 agreeing observations ---
                    if margin < 20:
                        self.get_logger().info(f'Skipping low confidence detection for {tag_id} with margin {margin:.2f}', throttle_duration_sec=2.0)
                        continue
                    
                    if tag_id not in self.pending_landmarks:
                        # First ever observation - buffer it, don't create vertex yet
                        self.pending_landmarks[tag_id] = (z.copy(), predicted_global_pos.copy(), margin, self.pose_id)
                        self.get_logger().info(f'Buffered first observation of {tag_id} from pose {self.pose_id}, waiting for confirmation')
                        continue
                    else:
                        # Second+ observation - check if it agrees with the buffered first
                        first_z, first_predicted, first_margin, first_pose_id = self.pending_landmarks[tag_id]
                        # Require minimum pose gap to avoid confirming from same viewpoint
                        # (PnP flip produces same wrong answer from same viewing angle)
                        if self.pose_id - first_pose_id < 3:
                            continue
                        disagreement = np.linalg.norm(predicted_global_pos - first_predicted)
                        
                        if disagreement > 0.5:
                            # First and second disagree - replace buffer with the new one
                            self.pending_landmarks[tag_id] = (z.copy(), predicted_global_pos.copy(), margin, self.pose_id)
                            self.get_logger().info(
                                f'Observation of {tag_id} from pose {self.pose_id} disagrees with buffered '
                                f'(from pose {first_pose_id}) by {disagreement:.2f}m - replacing buffer')
                            continue
                        
                        # First and second agree - create the landmark
                        del self.pending_landmarks[tag_id]
                        landmark_vertex_id = self.next_landmark_id
                        self.landmark_ids[tag_id] = self.next_landmark_id
                        self.next_landmark_id += 1
                        
                        # Use whichever observation was closer (more accurate) for vertex position
                        first_dist = np.linalg.norm(first_z[0:2])
                        curr_dist = np.linalg.norm(z[0:2])
                        if curr_dist < first_dist:
                            init_xy = predicted_global_xy
                            init_theta = predicted_global_theta
                        else:
                            init_xy = first_predicted
                            first_theta_global = wrap_angle(self.mu[2] + first_z[2])  # approximate
                            init_theta = first_theta_global
                        
                        self.write_vertex_se2(landmark_vertex_id, [init_xy[0], init_xy[1], init_theta])
                        self.landmark_observations_global[tag_id] = [first_predicted.copy(), predicted_global_pos.copy()]
                        
                        # Write the buffered first observation's edge
                        first_var_xy = self.default_var_xy + 0.02 * first_dist**2 + max(0, (50 - first_margin)) * 0.01
                        first_var_th = self.default_var_theta + 0.05 * first_dist**2
                        first_info_matrix = np.diag([1.0 / first_var_xy, 1.0 / first_var_xy, 1.0 / first_var_th])
                        self.write_edge_se2(first_pose_id, landmark_vertex_id, first_z, first_info_matrix)
                        
                        self.get_logger().info(
                            f'Created landmark {tag_id} (vertex {landmark_vertex_id}) confirmed by '
                            f'poses {first_pose_id} and {self.pose_id}')
                    
                else:
                    # --- EXISTING LANDMARK ---
                    # Consistency check: compare against median of all previous observations
                    prev_obs = np.array(self.landmark_observations_global[tag_id])
                    median_pos = np.median(prev_obs, axis=0)
                    disagreement = np.linalg.norm(predicted_global_pos - median_pos)
                    
                    if disagreement > 0.5:
                        self.get_logger().info(
                            f'Rejecting {tag_id} obs from pose {self.pose_id}: '
                            f'predicted ({predicted_global_pos[0]:.2f}, {predicted_global_pos[1]:.2f}) '
                            f'vs median ({median_pos[0]:.2f}, {median_pos[1]:.2f}), '
                            f'disagreement={disagreement:.2f}m')
                        continue
                    self.landmark_observations_global[tag_id].append(predicted_global_pos.copy())
                    landmark_vertex_id = self.landmark_ids[tag_id]
                    
                # Add observation edge between current pose and landmark
                #info_matrix = np.diag([1.0 / self.default_var_xy, 1.0 / self.default_var_xy])  # Information matrix is inverse of covariance
                dist = sqrt(x_rel**2 + y_rel**2)
                # Scale variance: grows with distance**2, shrinks with margin base variance + distance penalty + low-margin penalty
                var_xy = self.default_var_xy + 0.02 * dist**2 + max(0, (50 - margin)) * 0.01
                var_th = self.default_var_theta + 0.05 * dist**2
                info_matrix = np.diag([1.0 / var_xy, 1.0 / var_xy, 1.0 / var_th])
                self.write_edge_se2(self.pose_id, landmark_vertex_id, z, info_matrix)
                

    def generate_from_odom(self):
        # Get absolute motion
        theta_i = self.prev_mu[2]
        R_i_T = np.array([
            [cos(theta_i), sin(theta_i)],
            [-sin(theta_i), cos(theta_i)]
        ])
        delta_global = np.array([
            self.mu[0] - self.prev_mu[0],
            self.mu[1] - self.prev_mu[1]
        ])
        
        # Check if the motion is significant enough to add a new vertex and edge
        motion_threshold = 0.01  # meters
        if np.linalg.norm(delta_global) < motion_threshold:
            self.get_logger().info('Motion below threshold, skipping vertex/edge creation.', throttle_duration_sec=2.0)
            return
        
        # Calculate relative motion in robot frame
        delta_local = R_i_T @ delta_global
        
        # Create new vertex for current pose
        self.pose_id += 1
        current_pose_id = self.pose_id
        
        self.write_vertex_se2(current_pose_id, self.mu)
        self.write_groundtruth(current_pose_id)

        # Create edge from previous pose to current pose with relative motion as measurement
        # dx_rel = self.mu[0] - self.prev_mu[0]
        # dy_rel = self.mu[1] - self.prev_mu[1]
        # dtheta_rel = wrap_angle(self.mu[2] - self.prev_mu[2])
        # measurement = np.array([dx_rel, dy_rel, dtheta_rel])
        dx_rel = delta_local[0]
        dy_rel = delta_local[1]
        dtheta_rel = wrap_angle(self.mu[2] - self.prev_mu[2])
        measurement = np.array([dx_rel, dy_rel, dtheta_rel])
        # Simple diagonal information matrix
        info_matrix = np.diag([
            1.0 / 0.05,
            1.0 / 0.05,
            1.0 / 0.1
        ])

        self.write_edge_se2(self.prev_pose_id, current_pose_id, measurement, info_matrix)

        self.prev_pose_id = current_pose_id
        self.prev_mu = self.mu.copy()
        
    def write_vertex_se2(self, vertex_id, pose):
        x, y, theta = pose
        self.g2o_file.write(f"VERTEX_SE2 {vertex_id} {x} {y} {theta}\n") 
        
    # def write_vertex_xy(self, vertex_id, position):
    #     x, y = position
    #     self.g2o_file.write(f"VERTEX_XY {vertex_id} {x} {y}\n")

    def write_edge_se2(self, from_id, to_id, measurement, info_matrix):
        dx, dy, dtheta = measurement
        I = info_matrix
        self.g2o_file.write(
                f"EDGE_SE2 {from_id} {to_id} {dx} {dy} {dtheta} "
                f"{I[0,0]} {I[0,1]} {I[0,2]} "
                f"{I[1,1]} {I[1,2]} "
                f"{I[2,2]}\n"
        )   # g2o stores only upper triangular of information matrix.

    # def write_edge_se2_xy(self, pose_id, landmark_id, measurement, info_matrix):
    #     mx, my = measurement
    #     I = info_matrix
    #     self.g2o_file.write(
    #         f"EDGE_SE2_XY {pose_id} {landmark_id} {mx} {my} "
    #         f"{I[0,0]} {I[0,1]} "
    #         f"{I[1,1]}\n"
    #     )   # g2o stores only upper triangular of information matrix.
        
    def write_groundtruth(self, vertex_id):
        if self.gtruth_pose is None:
            self.get_logger().info("No ground truth pose yet")
            return

        p = self.gtruth_pose.pose.position
        q = self.gtruth_pose.pose.orientation

        theta = quat_to_yaw(q)

        self.gtruth_file.write(
            f"{vertex_id} {p.x} {p.y} {theta}\n"
        )


def main(args=None):
    rclpy.init(args=args)
    node = G2oGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.g2o_file.close()
    node.gtruth_file.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
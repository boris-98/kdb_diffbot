#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
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

class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')
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

        # Frames
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value    # ili get_parameter_value().string_value
        self.chassis_frame = self.get_parameter('chassis_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # EKF State
        self.mu = np.zeros(3)  # initial pose at map origin [0, 0, 0]
        self.Sigma = np.eye(3) * 0.01  # initial small uncertainty          TUNE ME LATER
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

        # Detected tags storage
        self.detected_tag_ids = []
        
        # Load global poses for tags from YAML file
        tag_map_yaml = self.get_parameter('tag_map_yaml').value
        self.get_logger().info(f'Loading tag map from: {tag_map_yaml}')
        self.tag_map = {} # tag_id -> 4x4 homogenous transform matrix (map->tag)
        if tag_map_yaml and os.path.isfile(tag_map_yaml):
            with open(tag_map_yaml, 'r') as f:
                tag_map_data = yaml.safe_load(f)
                # expecting format: { id: {x:..., y:..., z:..., yaw:...} }
                for tid, info in tag_map_data.items():
                    x = float(info.get('x', 0.0))
                    y = float(info.get('y', 0.0))
                    z = float(info.get('z', 0.0))
                    yaw = float(info.get('yaw', 0.0))
                    roll = float(info.get('roll', 0.0))
                    pitch = float(info.get('pitch', 0.0))
                    self.get_logger().info(f'Loaded tag {tid} at x:{x} y:{y} z:{z} yaw:{yaw}')
                    q = rpy_to_quat(roll, pitch, yaw)
                    self.tag_map[str(tid)] = tf_to_matrix((x, y, z), q)     # str(id) jer si u yaml stavio kao stringove kljuceve (tid_0, itd.)
                    #self.get_logger().info(f'tag_map[{str(tid)}]: {self.tag_map[str(tid)]}')
            self.get_logger().info(f'Loaded {len(self.tag_map)} tags from {tag_map_yaml}')
        else:
            self.get_logger().info('No tag YAML provided or file not found - will try TF lookups for map->tag (if available)')
        
        # TF listeners / broadcasters
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_static_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers and Publishers
        self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 50)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detections_callback, 10)
        self.ekf_pub = self.create_publisher(Odometry, '/ekf/odom', 10)

        # Main timer
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz

        # Initially publish map->odom transform where map->base is identity
        self.publish_map_odom_initial()

    def publish_map_odom_initial(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.odom_frame, rclpy.time.Time())
            T_odom_base = tf_to_matrix(
                (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
                (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
            )
        except:
            # Assume identity if lookup fails
            T_odom_base = np.eye(4)

        # map->base is mu (initial): mu = 0 -> T_map_base = identity
        T_map_base = np.eye(4)
        T_map_odom = T_map_base @ invert_homogen(T_odom_base)  # moglo je i samo sledece umesto poziva one funkcije: np.linalg.inv(T_odom_base)
        self.send_map_odom_tf(T_map_odom)

    def send_map_odom_tf(self, T_map_odom):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = float(T_map_odom[0, 3])
        t.transform.translation.y = float(T_map_odom[1, 3])
        t.transform.translation.z = float(T_map_odom[2, 3])

        # Convert rotation matrix back to quaternion
        R = T_map_odom[:3, :3]
        # Ako nekad zatreba 3D (u drugoj se podrazumeva samo yaw), onda ide sledece zakomentarisano
        # qw = sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
        # qx = (R[2, 1] - R[1, 2]) / (4 * qw)
        # qy = (R[0, 2] - R[2, 0]) / (4 * qw)
        # qz = (R[1, 0] - R[0, 1]) / (4 * qw)
        # t.transform.rotation.x = qx
        # t.transform.rotation.y = qy
        # t.transform.rotation.z = qz
        # t.transform.rotation.w = qw
        yaw = atan2(R[1,0], R[0,0])
        q = yaw_to_quat(yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_ekf_odom(self):
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame     # "map"
        msg.child_frame_id = self.base_frame     # "base_link"

        # Pose from mu
        msg.pose.pose.position.x = float(self.mu[0])
        msg.pose.pose.position.y = float(self.mu[1])
        msg.pose.pose.position.z = 0.0

        # Yaw -> quaternion
        qz = sin(self.mu[2] / 2.0)
        qw = cos(self.mu[2] / 2.0)

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        # Flatten covariance into 6x6 matrix
        cov = np.zeros((6, 6))

        cov[0, 0] = self.Sigma[0, 0]  # x-x
        cov[0, 1] = self.Sigma[0, 1]  # x-y
        cov[1, 0] = self.Sigma[1, 0]  # y-x
        cov[1, 1] = self.Sigma[1, 1]  # y-y

        cov[5, 5] = self.Sigma[2, 2]  # yaw-yaw

        msg.pose.covariance = cov.flatten().tolist()

        self.ekf_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        # Store the last odometry message (velocities se integrale)
        self.last_odom_msg = msg

    def detections_callback(self, msg: AprilTagDetectionArray):
        if len(msg.detections) == 0:
            self.detected_tag_ids = []
            return

        self.detected_tag_ids = [f"tag_{det.id}" for det in msg.detections]

    # Main loop
    def timer_callback(self):
        # 1. Prediction
        if self.last_odom_msg is not None:
            self.predict_from_odom(self.last_odom_msg)
        else:
            self.get_logger().warning('No odometry messages received yet for prediction step!')


        # 2. Correction
        if not self.detected_tag_ids:
            self.get_logger().info('No tags detected for correction step.', throttle_duration_sec=2.0)
            self.get_logger().info(f'Prior mu = [{self.mu[0]:.2f}, {self.mu[1]:.2f}, {self.mu[2]:.2f}]', throttle_duration_sec=2.0)
            self.publish_ekf_odom()
        else:
            for tag_id in self.detected_tag_ids:
                if tag_id not in self.tag_map:
                    self.get_logger().warning(f'Detected tag {tag_id} not in tag map, skipping.', throttle_duration_sec=2.0)
                    continue

                tag_frame = tag_id

                try:
                    t = self.tf_buffer.lookup_transform(self.base_frame, tag_frame, rclpy.time.Time())
                    T_base_tag = tfmsg_to_matrix(t)
                    
                    # Get T_map_tag from tag_map
                    T_map_tag = self.tag_map[tag_id]
                    
                    # Final estimate
                    T_map_base = T_map_tag @ invert_homogen(T_base_tag)
                    
                    # Extract measurement z = [x, y, theta] from T_map_cam
                    meas_x = T_map_base[0, 3]
                    meas_y = T_map_base[1, 3]
                    meas_theta = atan2(T_map_base[1, 0], T_map_base[0, 0])
                    z = np.array([meas_x, meas_y, meas_theta])
                    
                    # logs
                    self.get_logger().info(f'Found TF {self.odom_frame} -> {tag_frame}', throttle_duration_sec=1.0)
                    self.get_logger().info(f'T_base_tag:\n{T_base_tag}', throttle_duration_sec=1.0)
                    self.get_logger().info(f'Inverted T_tag_odom:\n{invert_homogen(T_base_tag)}', throttle_duration_sec=1.0)
                    self.get_logger().info(f'map->tag {tag_id}:\n{T_map_tag}', throttle_duration_sec=1.0)
                    self.get_logger().info(f'Estimated T_map_odom from tag {tag_id}:\n{T_map_base}', throttle_duration_sec=1.0)
                    self.get_logger().info(f'Correction from tag {tag_id}: z = [{meas_x:.2f}, {meas_y:.2f}, {meas_theta:.2f}]', throttle_duration_sec=1.0)
                    self.get_logger().info(f'Prior mu = [{self.mu[0]:.2f}, {self.mu[1]:.2f}, {self.mu[2]:.2f}]', throttle_duration_sec=1.0)

                    # Measurement prediction h(mu)
                    h_mu = self.mu.copy()  

                    # Measurement residual
                    y_k = z - h_mu
                    y_k[2] = wrap_angle(y_k[2])

                    # Measurement covariance R
                    R = np.diag([self.default_var_xy, self.default_var_xy, self.default_var_theta])

                    # Kalman Gain
                    S = self.Sigma + R
                    K = self.Sigma @ np.linalg.inv(S)

                    # Update state
                    self.mu = self.mu + K @ y_k
                    self.mu[2] = wrap_angle(self.mu[2])

                    # Update covariance
                    self.Sigma = (np.eye(3) - K) @ self.Sigma

                    # Publish updated odometry for visualization
                    self.publish_ekf_odom()

                except Exception as e:
                    self.get_logger().info(f"TF failed for {tag_id}: {e}")
                    continue


        # 3. Publish map->odom transform
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
            T_odom_base = tfmsg_to_matrix(t)
        except Exception:
            # fallback identity (nije idealno, proveri kasnije)
            T_odom_base = np.eye(4)

        # T_map_base from mu
        T_map_base = np.eye(4)
        T_map_base[0,3] = float(self.mu[0])
        T_map_base[1,3] = float(self.mu[1])
        T_map_base[:3,:3] = np.array([
            [cos(self.mu[2]), -sin(self.mu[2]), 0],
            [sin(self.mu[2]),  cos(self.mu[2]), 0],
            [0, 0, 1]
        ])
        T_map_odom = T_map_base @ invert_homogen(T_odom_base)
        self.send_map_odom_tf(T_map_odom)


    def predict_from_odom(self, odom_msg: Odometry):
        # Extract twist from odom_msg and update mu according to motion model
        t_now = odom_msg.header.stamp
        sec = t_now.sec + t_now.nanosec * 1e-9
        
        if self.last_odom_time is None:
            self.last_odom_time = sec
            return
        dt = sec - self.last_odom_time
        
        if dt <= 0:
            return
        self.last_odom_time = sec

        # Need only forward vel + yaw for 2D diff bot
        v = odom_msg.twist.twist.linear.x
        omega = odom_msg.twist.twist.angular.z

        theta = self.mu[2]
        if abs(omega) < 1e-6:
            # Straight line motion
            dx = v * dt 
            dy = 0.0
            dtheta = 0.0
        else:
            # Circular motion
            dx = -(v / omega) * sin(theta) + (v / omega) * sin(theta + omega * dt)
            dy = (v / omega) * cos(theta) - (v / omega) * cos(theta + omega * dt)
            dtheta = omega * dt

        # Update mu
        self.mu[0] += dx
        self.mu[1] += dy
        self.mu[2] =  wrap_angle(self.mu[2] + dtheta)
        #self.get_logger().info(f'Predicted mu = [{self.mu[0]:.2f}, {self.mu[1]:.2f}, {self.mu[2]:.2f}] from odom v={v:.2f} omega={omega:.2f} dt={dt:.2f}')

        # Jacobians G (w.r.t. state) and V (w.r.t. control noise)
        G = np.array([[1, 0, -dy],                                  # -dy ce dati isti izraz kao i izracunat izvod (a 0 svakako ako je straight line motion)
                      [0, 1,  dx if  abs(omega) >= 1e-6 else 0],    # za liniju iznad sa -dy nije problem jer je raniji if resio, a ovde ipak dx ne bi dao 0 ako je straight line motion
                      [0, 0,   1]])
        
        if abs(omega) >= 1e-6:
            V = np.array([[(-sin(theta) + sin(theta + omega * dt)) / omega, v * (sin(theta) - sin(theta + omega * dt)) / (omega ** 2) + v * cos(theta + omega * dt) * dt / omega],
                        [(cos(theta) - cos(theta + omega * dt)) / omega, -v * (cos(theta) - cos(theta + omega * dt)) / (omega ** 2) + v * sin(theta + omega * dt) * dt / omega],
                        [0, dt]])
        else:   # ako omega -> 0, onda je motion model: x = x + v*dt*cos(theta), y = y + v*dt*sin(theta), theta = theta, pa izvodi budu:
            V = np.array([
                        [cos(theta) * dt,  0],
                        [sin(theta) * dt,  0],
                        [0,               dt]])
        
        # Control noise covariance M
        M = np.array([[self.alpha1 * v**2 + self.alpha2 * omega**2, 0],
                      [0, self.alpha3 * v**2 + self.alpha4 * omega**2]])
        
        # Update covariance Sigma
        self.Sigma = G @ self.Sigma @ G.T + V @ M @ V.T


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import open3d as o3d
import numpy as np
import copy
import time

from transforms3d.euler import mat2euler
from transforms3d.euler import euler2mat

import torch
from cv_bridge import CvBridge

import rospy as ros

from sensor_msgs.msg import Image
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

from lab_group_w.msg import ImageData

#
#   Class to estimate the rotation of the objects
#
class PoseEstimator() : 
    """
    Class to estimate the rotation of the objects.
    """
    def __init__(self, object_points, object_label, path_to_models) :
      
        """
        Initialize the PoseEstimator.

        Args:
            object_points (list): List of object points.
            object_label (str): Label of the object.
            path_to_models (str): Path to the .stl meshes.
        """

        ## detected object points from the depth sensor
        self.object_points = object_points
        ## object name
        self.object_label = object_label
        ## absolute path to the mesh folder
        self.path_to_models = path_to_models

        ## Set True to see intermediate point cloud plots
        self.do_draw = False

        ## scaling factor to tune functions with
        self.voxel_size = 0.003  # means 3mm for the dataset
        
        self.source, self.target, self.source_down, self.target_down, self.source_fpfh, self.target_fpfh = \
            self.prepare_data(self.voxel_size, object_points, path_to_models, object_label)
        

    def draw_registration_result(self, source, target, transformation):
        """
        Draw the registration result.

        Args:
            source (PointCloud): Source point cloud (detected one).
            target (PointCloud): Target point cloud (artificial extracted from the mesh).
            transformation (ndarray): Transformation matrix between the 2 point clouds.
        """
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])


    def preprocess_point_cloud(self, pcd, voxel_size):
        """
        Preprocess the point cloud.

        Args:
            pcd (PointCloud): Point cloud.
            voxel_size (float): Voxel size to scale hyper parameters.

        Returns:
            tuple: Tuple containing the downsampled point cloud and FPFH features.
        """
        pcd_down = o3d.geometry.PointCloud.voxel_down_sample(pcd, voxel_size)

        radius_normal = voxel_size * 2
        o3d.geometry.PointCloud.estimate_normals(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh


    def prepare_data(self, voxel_size, object_points, path_to_models, object_label):
        """
        Generate the 2 point clouds as open3d friendly pointclouds and preprocess them by:
        downsampling, estimate normals, extract FPFH features.

        Args:
            voxel_size (float): Voxel size to scale hyper parameters.
            object_points (list): List of object points.
            path_to_models (str): Path to the .stl mesh.
            object_label (str): Label of the object.

        Returns:
            tuple: Tuple containing the source, target, downsampled source and target, and FPFH features.
        """

        # Convert points to a numpy array
        object_points = np.asarray(object_points)
        # Convert the arrays into Open3D point cloud objects
        source = o3d.cuda.pybind.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(object_points)

        # Read the mesh data from the STL file
        mesh = o3d.io.read_triangle_mesh(path_to_models+"/"+object_label+".stl")
        # Sample points uniformly from the mesh to create a point cloud
        n_points = int(np.size(object_points) * 1.5)
        print("sampled points from mesh : ", n_points)

        mesh_points = mesh.sample_points_uniformly(number_of_points=n_points)
        target = o3d.cuda.pybind.geometry.PointCloud()
        target.points = mesh_points.points

        if (self.do_draw) :
            self.draw_registration_result(source, target, np.identity(4))

        source_down, source_fpfh = self.preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target, voxel_size)
        return source, target, source_down, target_down, source_fpfh, target_fpfh

    def execute_global_registration(self, source_down, target_down, source_fpfh,
                                    target_fpfh, voxel_size):
        """
        Execute global registration using RANSAC.

        Args:
            source_down (PointCloud): Downsampled source point cloud.
            target_down (PointCloud): Downsampled target point cloud.
            source_fpfh (ndarray): Source FPFH features.
            target_fpfh (ndarray): Target FPFH features.
            voxel_size (float): Voxel size.

        Returns:
            RegistrationResult: Result of the global registration.
        """
        distance_threshold = voxel_size * 1.5
        
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True ,distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 500))
        return result


    def refine_registration(self, source, target, result_ransac, voxel_size):
        """
        Refine the registration using ICP.

        Args:
            source (PointCloud): Source point cloud.
            target (PointCloud): Target point cloud.
            result_ransac (RegistrationResult): Result of the global registration.
            voxel_size (float): Voxel size.

        Returns:
            RegistrationResult: refined registration result.
        """
        distance_threshold = voxel_size * 0.4

        result = o3d.pipelines.registration.registration_icp(
            source, target, distance_threshold, result_ransac.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        return result


    def run_pose_estimation(self) :
        """
        Run the pose estimation algorithm.

        Returns:
            ndarray: Rotation matrix of the object.
        """
        start = time.time()
        result_ransac = self.execute_global_registration(self.source_down, self.target_down,
                                                    self.source_fpfh, self.target_fpfh,
                                                    self.voxel_size)

        print("\nGlobal registration took %.3f sec.\n" % (time.time() - start))
        if (self.do_draw) :
            self.draw_registration_result(self.source_down, self.target_down,
                                        result_ransac.transformation)

        result = self.refine_registration(self.source_down, self.target_down, result_ransac, self.voxel_size)
        
        R = result.transformation[:3, :3].copy()

        return R


class imageProcessor() :
    """
    Class to detect and classify objects using YOLO5.
    """
    def __init__(self) :
        """
        Initialize the ImageProcessor.

        """

        #
        # DEFINE CLASS VARIABLES
        #

        ## image captured from the camera left eye
        self.img_L = np.zeros((1920,1080,3), np.uint8)
        # captured point cloud
        self.point_cloud = PointCloud2()

        # detected class names
        self.classes = []
        # detected bounding boxes (in input image scale 1920x1080)
        self.boxes = []
        # detected confidences
        self.confidences = []
        # detected class ids (yolov5 format)
        self.class_ids = []
        # detection counter
        self.number_of_detections = 0

        ## Object points respect to object's center
        self.objects_relative_points = []
        ## Object points relative to pixel coordinate system
        self.objects_relative_pixels = []

        ## list of rotation matrices from World to Object coordinates
        self.objects_rotations = []
        ## list of traslation vectors from World to Object coordinates
        self.objects_positions = []

        #
        # DEFINE CLASS COSTANTS
        # 
        # N.B: some might change based on camera calibration
        #

        ## Rotation matrix from Camera to World frame
        self.w_R_c = np.array([[ 0.0,      -0.49948, 0.86632],
                                [-1.0,       0.0,       0.0],
                                [-0.0,      -0.86632, -0.49948]])
        ## Traslation vector from Robot to Camera frame
        self.x_c = np.array([-0.9,   0.24, -0.35])

    
        # Set true if we're in real world
        if (False) :
            # Outputs of camera calibration
            robot_to_cam = np.array([ -1.00663661, 0.2125323, -0.35139947])
            rvec_robot_to_cam = np.array([0.02507289002211772, 0.4040827583973359, -0.020770899252596686])

            r_R_c = euler2mat(rvec_robot_to_cam[0], rvec_robot_to_cam[1], rvec_robot_to_cam[2])
            w_R_r = np.array([[1,  0,  0],
                              [0,  1,  0],
                              [0,  0,  1]])
            
            self.x_c = robot_to_cam
            self.w_R_c = r_R_c.dot(w_R_r)


        ## Translation vector from World to Robot base frame
        self.base_offset = np.array([0.5, 0.35, 1.75])

        #
        # Camera instrinsic parameters
        #
        
        horizontal_fov = 1.7633 
        image_width = 1920  
        image_height = 1080 
        
        fx = (image_width) / (2 * np.tan(horizontal_fov / 2))
        
        ## intrisic parameters of the camera 
        self.cameraMatrix = np.array([[fx,  0.0, image_width/2],
                                        [0.0, fx,  image_height/2],
                                        [0.0, 0.0, 1.0]])
        ## intrinsic distortion coefficients of the camera
        self.distCoeffs = np.array([[-0.043693598, 0.0146164996, 0.006573319, 0.000216900, 0.000084328]])


    def receive_pointcloud(self, msg):
        """
        Receive and store the point cloud data.

        Args:
            msg (PointCloud2): Point cloud message.
        """
        self.point_cloud = msg

    # LEFT CAMERA
    def receive_image_L(self, msg):
        """
        Receive the left camera image.

        Args:
            msg (Image): Image message.
        """
        #save and convert the image
        bridge = CvBridge()
        self.img_L = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def get_pointCloud_region(self, x1, y1, x2, y2) :
        """
        Retrieve the point cloud region.

        Args:
            x1 (int): x-coordinate of the top-left corner.
            y1 (int): y-coordinate of the top-left corner.
            x2 (int): x-coordinate of the bottom-right corner.
            y2 (int): y-coordinate of the bottom-right corner.

        Returns:
            tuple: points regarding the specified region in camera coordinates, same points in pixel coordinates.
        """

        # Round values : point_cloud2.read_points uvs accepts INT only
        # Round tight for better precision
        # (more likely to remove some pixels that don't belong to the object)
        x1 = int(np.ceil(x1))
        y1 = int(np.ceil(y1))
        x2 = int(np.floor(x2))
        y2 = int(np.floor(y2))

        # Define a list of pixel coordinates within the bounding box
        uvs = [(x, y) for y in range(y1, y2+1) for x in range(x1, x2+1)]

        # Extract the points within the bounding box
        points_region = []
        for data in point_cloud2.read_points(self.point_cloud, field_names=['x', 'y', 'z'], skip_nans=False, uvs=uvs) :
            points_region.append([data[0], data[1], data[2]])
        
        return points_region, uvs

    def object_detection (self, path_to_model, path_to_weights):
        """
        Detect objects in the image and store the results in class variables:

        boxes : bounding boxes of the objects.
        confidences : confidence score of each detection.
        class_ids : classes associated yolo indexes.
        classes : class names in string format.
        number_of_detections : number of detected objects.

        Args:
            path_to_model (string): absolute path to the Yolov5 model.
            path_to_weights (string) : absolute path to the weights file.
        """

        if torch.cuda.is_available():
            device = torch.device('cuda')
            print("CUDA available, device = GPU\n")
        else:
            device = torch.device('cpu')
            print("CUDA not available, device = CPU\n")

        model = torch.hub.load(path_to_model, 'custom', path=path_to_weights, source='local', force_reload=False)  # local repo
        
        model.conf = 0.80  # set confidence threshold
        model.iou = 0.45  # set threshold for Non-maximum suppression
        
        # Move the model to the selected device
        model = model.to(device)

        image = cv2.cvtColor(self.img_L, cv2.COLOR_BGR2RGB)

        # Inference
        start = time.time()
        results = model(image) # pass the image through the model
        print("Image detection took %.3f sec.\n" % (time.time() - start))

        self.boxes = results.pandas().xyxy[0][['xmin','ymin','xmax','ymax']].values.tolist()
        
        self.confidences = results.pandas().xyxy[0]['confidence'].values.tolist()
        self.class_ids = results.pandas().xyxy[0]['class'].values.tolist()
        self.classes = results.pandas().xyxy[0]['name'].values.tolist()
        
        self.number_of_detections = len(self.class_ids)


    def show_detections (self) :
        """
        Show the image with detections on using openCV libraries
        """
        img_cpy = self.img_L.copy()

        # Iterate over the detections, draw rectangles and show class names
        for i in range(0, self.number_of_detections):
            x1, y1, x2, y2 = self.boxes[i]
            x1 = int(np.ceil(x1))
            y1 = int(np.ceil(y1))
            x2 = int(np.floor(x2))
            y2 = int(np.floor(y2))

            text_print = self.classes[i] + " : " + str(round( self.confidences[i], 2))

            cv2.rectangle(img_cpy, (x1, y1), (x2, y2), (0, 255, 0), 1)  # Display bbox
            cv2.putText(img_cpy, text_print, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 1)  # Display name above the bbox

        # Display the image with the detected objects
        cv2.imshow('Object Detection', img_cpy)

        # Uncomment to store the image
        #cv2.imwrite('detection.png' , img_cpy)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def pose_estimation(self, models_dir):
        """
        Instantiate a PoseEstimater object for each detection and run pose estimation on it

        Args:
            models_dir (string) : Absolute path to .stl mesh folder
        """
        # Iterate over the detected objects and estimate their pose
        for i in range(0, self.number_of_detections) :
            box = self.boxes[i]
            x1, y1, x2, y2 = box
            label = self.classes[i]
            confidence = self.confidences[i]

            # Extract points regarding the wanted object
            # Expressed in camera frame
            point_region, point_region_pixels = self.get_pointCloud_region(x1, y1, x2, y2)

            # Transform from Camera to World coordinate system
            object_points = []
            for point in point_region :
                object_points.append(self.w_R_c.dot(point) + self.x_c + self.base_offset)
              
            # Filter ponts below 0.866 of height (they belong to the table and only add noise)
            object_points_filtered = []
            point_region_pixels_filtered = []
            for i in range(0, len(object_points)) :
                if(object_points[i][2] > 0.867) :
                    object_points_filtered.append(object_points[i])
                    point_region_pixels_filtered.append(point_region_pixels[i])

            # Get middle point by averaging all points
            point_middle = np.mean(object_points_filtered, axis=0)
            print(label+" Center : ", point_middle)
            
            # Apply offset to Object center
            object_points_filtered_traslated = []
            for point in object_points_filtered :
                object_points_filtered_traslated.append((point - point_middle))

            # Get the rotation matrix and translation vector from the detected pointcloud and the associated .stl model
            poseEstimator = PoseEstimator(object_points_filtered_traslated, label, models_dir)
            object_rotation_matrix = poseEstimator.run_pose_estimation()

            print(label, " rotation : ", np.degrees(mat2euler(object_rotation_matrix)), "\n ------------------------------------------------------")

            # Apply rotation to all object points
            # Now object_points are expressed in Object coordinate systemm
            object_points_filtered_traslated_rotated = []
            for point in object_points_filtered_traslated :
                object_points_filtered_traslated_rotated.append(object_rotation_matrix.dot(point))
            
            # Value to show pose estimation
            self.objects_relative_points.append(object_points_filtered_traslated_rotated)
            self.objects_relative_pixels.append(point_region_pixels_filtered)

            # Values to publish on topic
            self.objects_positions.append(point_middle)
            self.objects_rotations.append(object_rotation_matrix)

    def show_objects_poses (self):
        """
        Display the detected rotation on the objects as cartesian axes with openCV

        """
        img_cpy = self.img_L.copy()
        for i in range(0, self.number_of_detections):
            
            # Load matching Points (in object coordinate system) and Pixels (image coordinate system)
            object_points = np.array([tuple(x) for x in self.objects_relative_points[i]], dtype=np.float32)
            point_region_pixels = np.array([tuple(x) for x in self.objects_relative_pixels[i]], dtype=np.float32)

            #
            # Returns the rotation and the translation vectors that transform a
            # 3D point expressed in the object coordinate frame to the camera coordinate frame
            #
            _, rvec, tvec = cv2.solvePnP(object_points, point_region_pixels, self.cameraMatrix, self.distCoeffs)

            # Draw the axis on the image
            axis_length = 0.1 # length of axis lines in meters
            cv2.drawFrameAxes(img_cpy, self.cameraMatrix, self.distCoeffs, rvec, tvec, axis_length)

        
        # Uncomment to store the image
        #cv2.imwrite('poses.png', img_cpy)

        # Display
        cv2.imshow('Detected poses', img_cpy)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
def talker(p):
    """
    Initialize the node, run object detection and pose estimation, send results to "task_planner" node as msg type (ImageData)
    """
    
    # Initialize the ROS node.
    ros.init_node("imageProcessor")

    # Subscribe to pointcloud and image publishers
    sub_pointcloud = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, callback= p.receive_pointcloud, queue_size=1)
    sub_image_l = ros.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, callback= p.receive_image_L, queue_size=1)
    pub = ros.Publisher('/imageProcessor/processed_data', ImageData, queue_size = p.number_of_detections)

    # Run object detection and classification
    path_to_model = "/home/luca/Desktop/yolo5_training/yolov5"
    path_to_weights = '/home/luca/Desktop/yolo5_training/yolov5/epoch10.pt'
    p.object_detection(path_to_model, path_to_weights)
    p.show_detections()

    # Run pose estimation
    path_to_meshes = "/home/luca/ros_ws/src/locosim/robot_control/lab_exercises/lab_group_w/scripts/models"
    p.pose_estimation(path_to_meshes)
    p.show_objects_poses()
    
    # loop frequency
    ur5rate = 0.001     #from params.py
    rate = ros.Rate(1 / ur5rate)

    for i in range(0, p.number_of_detections):
        msg = ImageData()
        msg.class_type = p.class_ids[i]
        msg.position = p.objects_positions[i]
        msg.rotation = np.array(p.objects_rotations[i]).flatten().tolist()
                       
        pub.publish(msg)
        print('Published message, object type : ', msg.class_type)
        rate.sleep()

    while not ros.is_shutdown() :
        ros.sleep(1)

if __name__ == '__main__':

    # Initialize imageProcessor class
    p = imageProcessor()
    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
    




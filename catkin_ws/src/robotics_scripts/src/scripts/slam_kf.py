#!/usr/bin/env python3
import numpy as np
import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from robotics_project.msg import custom_msg
from bresenham import bresenham

def p2l(p):
	"""
		           p(x)
	 l(x) = log ----------
		         1 - p(x)

	"""
	return np.log(p / (1 - p))

def l2p(l):
	"""
	 		         1
	 p(x) = 1 - ---------------
		         1 + exp(l(x))
	"""
	return 1 - 1 / (1 + np.exp(l))
    

class grid:

    def __init__(self,center_x, center_y, grid_size_x, grid_size_y, grid_resolution, min_angle,max_angle, laser_resolution, range_max, range_min, p_occ, p_free,p_prior):
        """
        (center_x , center_y)      : the center of the map grid [in meters].
        (grid_width , grid_height) : size of the map [in meters].
        grid_resolution  : meters/cell.
        min_angle        : the start angle of laser [in radian].
        max_angle        : the end "max" angle laser can reach [in radian].
        laser_resolution : angle step "degrees per step(increment)" [in radian].
        range_max        : maximum range value [in meters].
        p_occ            : probability that cell is occupied with fully confidence.
        p_free           : probability that cell is free with fully confidence.
        p_prior          : prior probability for all the cells.

        """
        # set the class parameters
        self.grid_center_x = center_x          
        self.grid_center_y = center_y          
        self.grid_size_x = grid_size_x              
        self.grid_size_y = grid_size_y              
        self.grid_resolution = grid_resolution      
        self.laser_min_angle = min_angle    
        self.laser_max_angle = max_angle    
        self.laser_resolution = laser_resolution  
        self.laser_max_range = range_max   
        self.laser_min_range = range_min  
        self.sensor_model_l_occ = p2l(p_occ)
        self.sensor_model_l_free = p2l(p_free)
        self.sensor_model_l_prior = p2l(p_prior)


        # get the number of rows and columns of the grid
        num_rows = int(grid_size_y / grid_resolution)
        num_cols = int(grid_size_x / grid_resolution)

        # initialize the grid with inverse p_prior
        self.grid = self.sensor_model_l_prior * np.ones((num_rows, num_cols))
        self.hit_miss = np.zeros((num_rows, num_cols))
    
    
    def to_meters (self, i, j):
        """
        input  : (j,i) for specific cell in the grid.
        output : (x,y) position in meters for the corresponding cell.

        """
        x = j * self.grid_resolution + self.grid_center_x
        y = i * self.grid_resolution  + self.grid_center_y
        return x, y

    def to_cell (self, x, y):
        """
        input  : (x,y) position in meters.
        output : (j,i) represents the corresponding cell.

        """
        i = (y - self.grid_center_y) / self.grid_resolution
        j = (x - self.grid_center_x) / self.grid_resolution
        return i, j

    def is_inside (self, i, j):
        return i<self.grid.shape[0] and j<self.grid.shape[1] and i>=0 and j>=0

    def ray_casting(self, robot_x, robot_y, theta, distance):
        """
        inputs: 
                 (x,y) : robot position in meters.
                 theta : angle of the laser baem with reference to +x axis.
                 distance : at which the laser beam hit an obstacle.

        functionality: 
                    based on (distance) --> (1) -INF means it hits object that very close to the sensor.
                                            (2)  NAN means errors in the readings at this time stamp.
                                            (3) +INF means it reached the max range and didn't hit objects.
                                            (4)  min range < distance < max_range, distance at which it hits an object.
                    get the (x,y) positions of the object and its corresponding cell in the grid.
                    using bresenham return the points on the line starting from robot position and 
                    ending with the object.

        output : updated grid cells.
        """
        if distance < self.laser_min_range or distance > self.laser_max_range:
            return
        # if np.isinf(distance) and np.sign(distance) == +1:
        #     distance = self.laser_max_range
        elif np.isinf(distance) or np.isnan(distance):
            return
        
        # get the object position (x,y) in meters
        object_x = robot_x + distance * np.cos(theta)
        object_y = robot_y + distance * np.sin(theta)

        # get the corresponding cells in the grid 
        robot_i, robot_j   = self.to_cell(robot_x, robot_y)    # cell holding the robot
        object_i, object_j = self.to_cell(object_x, object_y)  # cell holding the object

        num_cells = distance / self.grid_resolution

        ip, jp = self.bresenham(robot_i, robot_j, object_i, object_j, num_cells)
        if not np.isnan(distance) and distance != self.laser_max_range and self.is_inside(int(ip),int(jp)):
            # Hit!
            self.hit_miss[int(ip),int(jp)] -=1
            if self.hit_miss[int(ip),int(jp)] <= -5: 
                self.grid[int(ip),int(jp)] += self.sensor_model_l_occ - self.sensor_model_l_prior
        return
    
    
    def bresenham (self, i0, j0, i1, j1, steps):   # i0, j0 (starting point)
        """
        inputs: 
                (j0,i0): the starting point.
                (j1,i1): the ending point.
                steps  : number of steps or cells between start and end.

        functionality:                                          y                         ___ x
                    Note: bresenham algorithm assuming the axes |___ x (BUT) map grid is |       
                    (rows accessed with (y = i) and cols with x = j).                    y
                    
        output : updated grid cells that line passes through.
        """

        points = list(bresenham(int(j0),int(i0),int(j1),int(i1)))
        length = len(points)
        for i in range(length - 1):
            jp = points[i][0]
            ip = points[i][1]
            if (jp == j1 and ip == i1) or (np.sqrt((jp-j0)**2+(ip-i0)**2) >= steps) or not self.is_inside(ip, jp):
                return ip, jp
            elif self.grid[int(ip),int(jp)] >= 4 * self.sensor_model_l_occ:
                return ip, jp
            # self.hit_miss[int(ip),int(jp)] +=1
            # if self.hit_miss[int(ip),int(jp)] >= 7:
            self.grid[int(ip),int(jp)] += self.sensor_model_l_free - self.sensor_model_l_prior
        return points[length-1][1],points[length-1][0]

    def update(self, x, y, robot_theta, scan):
        """
        inputs: 
                (x,y) : robot position in meters.
                robot_theta : robot orientation.
                scan : list of scan (angle,distance) data per time stamp.

        functionality: 
                - for each laser beam data in scan list update the laser theta with respect to the robot orientation.
                - Note : since list starting with index 0 --> this is not the min angle of the laser, so we add the min angle
                to the theta then multiply the index by the resolution (which is the step between beam and the next).
                for ex :
                    assume robot orientation = 0 degree , min range = -180 degree , resolution = 2 degree
                    index 0 --- > theta = 0 + (-180) + 0 * 2 = -180
                    index 1 --- > theta = 0 + (-180) + 1 * 2 = -178 
                theta = robot_orientation + laser_min_range + index * step (laser_resolution).

        output : call ray casting to generate laser beam from robot to the distance (obj).
        """
        for angle, distance in enumerate(scan):
            theta = robot_theta + self.laser_min_angle + angle * self.laser_resolution
            self.ray_casting(x, y, theta, distance)
        return self.grid



class slam_kf:

    def __init__(self):
        rospy.init_node('slam_kf', anonymous=True)

        self.initialized = False
        self.first_reading = True
        self.map_last_publish = rospy.Time()

        # initialize robot position
        self.prev_robot_x = []
        self.prev_robot_y = []
        self.prev_robot_theta = []

        self.prevTime = rospy.Time(0)
        self.x_old = 0
        self.y_old = 0
        self.prevTheta = 0
        self.qx_old = 0
        self.qy_old = 0
        self.qz_old = 0
        self.qw_old = 0

        # read the parameters from the launch file
        self.sensor_model_p_occ   = rospy.get_param('~sensor_model_p_occ', 0.75)
        self.sensor_model_p_free  = rospy.get_param('~sensor_model_p_free', 0.45)
        self.sensor_model_p_prior = rospy.get_param('~sensor_model_p_prior', 0.5)
        self.robot_frame          = rospy.get_param('~robot_frame', 'robot_base_link')
        self.map_frame            = rospy.get_param('~map_frame', 'robot_map')
        self.map_center_x         = rospy.get_param('~map_center_x', -50.0)
        self.map_center_y         = rospy.get_param('~map_center_y', -50.0)
        self.map_size_x           = rospy.get_param('~map_size_x', 100.0)
        self.map_size_y           = rospy.get_param('~map_size_y', 100.0)
        self.map_resolution       = rospy.get_param('~map_resolution', 0.08)
        self.map_publish_freq     = rospy.get_param('~map_publish_freq', 1.0)
        self.update_movement      = rospy.get_param('~update_movement', 0.08)
        # initialize occupancy grid message for the map
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size_y / self.map_resolution)
        self.map_msg.info.height = int(self.map_size_x / self.map_resolution)
        self.map_msg.info.origin.position.x = self.map_center_x
        self.map_msg.info.origin.position.y = self.map_center_y
        
        # Publishers and Subscribers
        self.laser_subscriber = rospy.Subscriber("/laser_odom", custom_msg, self.call_back, queue_size=1)
        self.map_publisher = rospy.Publisher('/slam_kf', OccupancyGrid, queue_size=1)

    def initialize_grid(self, min_angle, max_angle,max_range,min_range, laser_resolution):
        self.gridmapping = grid(self.map_center_x, self.map_center_y, self.map_size_x, self.map_size_y, self.map_resolution, min_angle, max_angle, laser_resolution,max_range,min_range, self.sensor_model_p_occ, self.sensor_model_p_free, self.sensor_model_p_prior)
        self.initialized = True

    def to_yaw(self, qx, qy, qz, qw):
        """
        input : quaternion data (x,y,x,w)

        functionality: 
            sin = 2(w * z + x * y)
            cos = (1-2(y^2 + z^2))
            yaw = atan2(sin,cos)

        output : yaw "rotation around z"

        """
        return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    def publish_occupancygrid(self, data, header_stamp):
        """
            input : grid
            functionality:
                        convert grid map to occupancy grid map which hold data of type int8[]
                        probabilities are in range 0--100 
                        unknown cells are set to -1
                        
            output: published occupancy grid map.
        """
        temp_data = l2p(data)
        data = (temp_data*100).astype(dtype=np.int8)
        mask = (temp_data == self.sensor_model_p_prior)  # for setting unknown cells to -1
        data[mask] = -1  
        # Publish occupancy grid map
        self.map_msg.data = data
        self.map_msg.header.frame_id = 'robot_map'
        self.map_msg.header.stamp = header_stamp
        self.map_publisher.publish(self.map_msg)
        rospy.loginfo_once("Published map!")

    def call_back(self, data):

        laser = data.laser_msg
        odom = data.odom_msg

        if not self.initialized:
            self.initialize_grid(laser.angle_min, laser.angle_max, laser.range_max,laser.range_min, laser.angle_increment)

        now = rospy.Time(0)
        try:
            if self.first_reading:
                # observation
                obs_robotX = odom.pose.pose.position.x
                obs_robotY = odom.pose.pose.position.y
                obs_robotOrientation = self.to_yaw(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
                self.x_old = obs_robotX
                self.y_old = obs_robotY
                self.prevTheta = obs_robotOrientation
                self.first_reading = False

            # observation
            obs_robotX = odom.pose.pose.position.x
            obs_robotY = odom.pose.pose.position.y
            obs_robotOrientation = self.to_yaw(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
            
            # prediction 
            vX = odom.twist.twist.linear.x 
            vY = odom.twist.twist.linear.y
            vTheta = odom.twist.twist.angular.z 
            dt = (odom.header.stamp - self.prevTime).to_sec()

            #calculate kalman gain
            kX = 0.8
            kY = 0.8
            kTheta = 0.8
            
            # calc the new position based on the velocity
            pred_robotOrientation = self.prevTheta + vTheta * dt
            pred_robotX = self.x_old + vX * dt * math.cos(pred_robotOrientation) - vY * dt * math.sin(pred_robotOrientation)
            pred_robotY = self.y_old + vX * dt * math.sin(pred_robotOrientation) + vY * dt * math.cos(pred_robotOrientation)

            # correction
            robotX = pred_robotX + kX * (obs_robotX - pred_robotX)
            robotY = pred_robotY + kY * (obs_robotY - pred_robotY)
            robotOrientation = pred_robotOrientation + kTheta * (obs_robotOrientation - pred_robotOrientation)
            

            
            # update the map if the robot moved > specifc value 
            # movement = (robotX - self.x_old)**2 + (robotY - self.y_old)**2
            # if ( movement >= self.update_movement**2 ):
            
            gridmap = self.gridmapping.update(robotX, robotY, robotOrientation, laser.ranges).flatten()


            # publish map (with the specified frequency)
            if (self.map_last_publish.to_sec() + 1.0/self.map_publish_freq < rospy.Time.now().to_sec() ):
                self.map_last_publish = rospy.Time.now()
                self.publish_occupancygrid(gridmap, now)

            # update old pose 
            self.x_old = robotX
            self.y_old = robotY
            self.prevTheta = robotOrientation
            self.prevTime = odom.header.stamp
        except Exception as e:
            rospy.logerr(e)
            pass


se = slam_kf()
while not rospy.is_shutdown():
    rospy.spin()
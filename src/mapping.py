#!/usr/bin/env python
""" Simple occupancy-grid-based mapping without localization.

Subscribed topics                      :
/scan

Published topics                       :
/map
/map_metadata

Author                                 : Nathan Sprague
Version                                : 2/13/14
"""
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf
from nav_msgs.msg import Odometry
import time
import numpy as np

class Map(object)                      :
    """
    The Map class stores an occupancy grid as a two dimensional
    numpy array.

    Public instance variables          :

        width -- Number of columns in the occupancy grid.
        height -- Number of rows in the occupancy grid.
        resolution -- Width of each grid square in meters.
        origin_x -- Position of the grid cell (0,0) in
        origin_y -- in the map coordinate system.
        grid -- numpy array with height rows and width columns.


    Note that x increases with increasing column number and y increases
    with increasing row number.
    """

    def __init__(self, origin_x=-20, origin_y=-20, resolution=0.1,
                 width=500, height=500):
        """ Construct an empty occupancy grid.

        Arguments                      : origin_x,
                   origin_y -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells
                                in meters.
                   width,
                   height -- The grid will have height rows and width
                                columns cells. width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.

         The default arguments put (0,0) in the center of the grid.

        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width))

    def to_message(self)               :
        """ Return a nav_msgs/OccupancyGrid representation of this map. """

        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message.
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        grid_msg.data = list(np.round(flat_grid))
        return grid_msg

    def set_cell(self, x, y, val)      :
        """ Set the value of a cell in the grid.

        Arguments                      :
            x, y - This is a point in the map coordinate frame.
            val - This is the value that should be assigned to the
                    grid cell that contains (x,y).

        This would probably be a helpful method! Feel free to throw out
        point that land outside of the grid.
        """
        pass

class Mapper(object)                   :
    """
    The Mapper class creates a map from laser scan data.
    """

    def __init__(self)                 :

        """ Start the mapper. """

        rospy.init_node('mapper')
        self._map = Map()

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages. This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans. Better to just drop
        # old scans and always work with the most recent available.
        self.position = [0,0]
        rospy.Subscriber('odom',
                         Odometry, self.odom_callback, queue_size=1)
        time.sleep(0.2)
        rospy.Subscriber('base_scan_1',
                         LaserScan, self.scan_callback, queue_size=1)

        # Latched publishers are used for slow changing topics like
        # maps. Data will sit on the topic until someone reads it.
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True)
        self._map_data_pub = rospy.Publisher('map_metadata',
                                             MapMetaData, latch=True)

        rospy.spin()

    def odom_callback(self,odom):
        # global myOdom = odom
        pos =  odom.pose.pose.position
        self.position[0] = pos.x
        self.position[1] = pos.y


    def scan_callback(self, scan)      :
        """ Update the map on every scan callback. """

        # Fill some cells in the map just so we can see that something is
        # being published.
        Lresol = 10
        r = scan.ranges[0]
        xt = [self.position[0]+20, self.position[1]+20, np.pi/4]
        # for k in range(0,len(scan.ranges)-1):
        scanAngles = np.linspace(1/2*np.pi,-1/2*np.pi,len(scan.ranges))
        lidar_local = np.array([xt[0]+scan.ranges*np.cos(scanAngles+xt[2]), xt[1]-(scan.ranges*np.sin(scanAngles+xt[2]))])

        # print len(lidar_local[1])
        xtg = [np.ceil(xt[1]*Lresol),np.ceil(xt[2]*Lresol)]
        for k in range(0,len(scan.ranges)-1):
            if scan.ranges[k]<scan.range_max:
                rtl = np.ceil(lidar_local[:,k]*Lresol)
                self._map.grid[rtl[1]][rtl[0]]=1

        theta = scan.angle_min+0*scan.angle_increment
        print r
        print theta
        l = bresenham((0,0),(3,4))
        print l.path
        # self._map.grid[0, 0] = 1.0
        # self._map.grid[0, 1] = .9
        # self._map.grid[0, 2] = .7
        # self._map.grid[1, 0] = .5
        # self._map.grid[2, 0] = .3


        # Now that the map is updated, publish it!
        rospy.loginfo("Scan is processed, publishing updated map.")
        self.publish_map()


    def publish_map(self):
        """ Publish the map. """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


class bresenham:
	def __init__(self, start, end):
		self.start = list(start)
		self.end = list(end)
		self.path = []

		self.steep = abs(self.end[1]-self.start[1]) > abs(self.end[0]-self.start[0])

		if self.steep:
			print 'Steep'
			self.start = self.swap(self.start[0],self.start[1])
			self.end = self.swap(self.end[0],self.end[1])

		if self.start[0] > self.end[0]:
			print 'flippin and floppin'
			_x0 = int(self.start[0])
			_x1 = int(self.end[0])
			self.start[0] = _x1
			self.end[0] = _x0

			_y0 = int(self.start[1])
			_y1 = int(self.end[1])
			self.start[1] = _y1
			self.end[1] = _y0

		dx = self.end[0] - self.start[0]
		dy = abs(self.end[1] - self.start[1])
		error = 0
		derr = dy/float(dx)

		ystep = 0
		y = self.start[1]

		if self.start[1] < self.end[1]: ystep = 1
		else: ystep = -1

		for x in range(self.start[0],self.end[0]+1):
			if self.steep:
				self.path.append((y,x))
			else:
				self.path.append((x,y))

			error += derr

			if error >= 0.5:
				y += ystep
				error -= 1.0

		print start
		print end
		print
		print self.start
		print self.end

	def swap(self,n1,n2):
		return [n2,n1]
"""
l = bresenham([8,1],[6,4])
print l.path

map = []
for x in range(0,15):
	yc = []
	for y in range(0,15):
		yc.append('#')
	map.append(yc)

for pos in l.path:
	map[pos[0]][pos[1]] = '.'

for y in range(0,15):
	for x in range(0,15):
		print map[x][y],
	print
"""

if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass

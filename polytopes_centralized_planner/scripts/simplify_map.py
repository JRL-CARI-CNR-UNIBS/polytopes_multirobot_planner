#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped, Point32, Pose
from polytopes_centralized_planner_msgs.srv import MapWithBoundingBoxes


import numpy as np
import cv2
import copy

class SimplifyMap(Node):
  sub_topic = "/sweepee_2/global_costmap/costmap"
  # pub_topic = "/simplified_map_points"
  srv_name = "/simplify_map"
  def __init__(self):
    super().__init__('simplify_map')
    self.subscription = self.create_subscription(OccupancyGrid, self.sub_topic, self.save_map, 1)
    # self.publisher = self.create_publisher(PolygonStamped, self.pub_topic, 10)
    self.srv = self.create_service(MapWithBoundingBoxes, self.srv_name, self.simplification)
    self.map_saved: bool = False
    print("setup ok")

  def save_map(self, msg: OccupancyGrid):
    self.map: OccupancyGrid = msg
    self.width:int = msg.info.width
    self.height:int = msg.info.height
    self.map_grid = msg.data
    self.resolution:float = msg.info.resolution
    self.map_pose: Pose = msg.info.origin
    self.map_saved: bool = True

  def simplification(self, request, response):
    if not self.map_saved:
      rclpy.spin_once(self)
    np_data = np.asarray(self.map_grid, dtype="uint8")
    gray = np_data.reshape(self.height, self.width)
    gray = np.round(gray * 255./100).astype(np.uint8)
    ret,thresh = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
    contours,h = cv2.findContours(thresh,1,2)
    bounding_box=[];
      
    for cnt in contours:
      x,y,w,h = cv2.boundingRect(cnt)
      if w < 5 and h < 5:
          continue
      bounding_box.append(np.array([x,y,x+w,y+h]))

    bounding_box_in_map_frame = []
    for bb in bounding_box:
      bounding_box_in_map_frame.append(np.array([bb[0], self.height - bb[1], bb[2], self.height - bb[3]])*self.resolution + self.resolution/2)

    # img = copy.copy(gray)
    # [cv2.rectangle(img, (bb[0],bb[1]), (bb[2], bb[3]), 127, 2) for bb in bounding_box]
    # cv2.namedWindow('img',cv2.WINDOW_NORMAL)
    # cv2.imshow('img',img)
    # cv2.waitKey(-1)
    # cv2.destroyAllWindows()

    polygon_list = []
    polygon_out = PolygonStamped()
    polygon_out.header.frame_id = "map"
    polygon_out.header.stamp = self.get_clock().now().to_msg()
    for bb in bounding_box_in_map_frame:
      polygon_out.polygon.points.clear()
      polygon_out.polygon.points.append(Point32(x = float(bb[0]), y = float(bb[1]), z = 0.0))
      polygon_out.polygon.points.append(Point32(x = float(bb[0]), y = float(bb[3]), z = 0.0))
      polygon_out.polygon.points.append(Point32(x = float(bb[2]), y = float(bb[3]), z = 0.0))
      polygon_out.polygon.points.append(Point32(x = float(bb[2]), y = float(bb[1]), z = 0.0))
      polygon_list.append(copy.copy(polygon_out))
      response.map = self.map
    response.polygon_list = polygon_list
    return response


    


def main(args=None):
  rclpy.init(args=args)
  sm = SimplifyMap()
  rclpy.spin(sm)

  sm.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()

import rospy
import matplotlib.pyplot as plt
import numpy as np

#from geometry_msgs import Polygon
from std_msgs.msg import Float64MultiArray

# DEBUG
import logging

def set_rospy_log_lvl(log_level):
    logger = logging.getLogger('rosout')
    logger.setLevel(rospy.impl.rosout._rospy_to_logging_levels[log_level])

class ScenePlotter():
  def __init__(self):
    fig, ax = plt.subplots()
    self.fig = fig
    self.ax = ax
    self.ax.grid(True)
    self.config = []
    self.ready_to_plot = False
    self.parameter_loaded = False
  
  def plot(self, data, color='k'):
    shape = data.shape
    if len(shape) == 1 or shape[1] == 1:
      self.ax.plot(data[0], data[1],  marker="x", markersize=20, color=color)
    elif shape[1] == 2:
      self.ax.plot([data[0,0], data[0,1]],
                   [data[1,0], data[1,1]],
                   marker=".", color=color)
    elif shape[1] > 2:
      for idx in range(shape[1]):
        self.ax.plot([data[0, idx], data[0, (idx+1) % shape[1]]],
                   [data[1, idx], data[1, (idx+1) % shape[1]]],
                   marker=".", linestyle="-", color=color)

  def load_parameters(self):
    self.robots = rospy.get_param('robot_vertices/robot_to_load')
    self.scene_bound = np.array(rospy.get_param('scene_bound')).T
    self.cube_1 = np.array(rospy.get_param('cube_1')).T
    self.cube_2 = np.array(rospy.get_param('cube_2')).T
    self.init = np.array(rospy.get_param('init')).T
    self.goal = np.array(rospy.get_param('goal')).T
    self.bounds = np.array(rospy.get_param('bounds')).T

    object_vertices = rospy.get_param('object_vertices')
    self.object_vertices = np.array(object_vertices).T
    grasping_vertices_in_obj = rospy.get_param('grasping_vertices_in_obj')
    self.grasping_vertices_in_obj = np.array(grasping_vertices_in_obj).T
    grasping_from_center_in_robot = rospy.get_param('grasping_from_center_in_robot')
    self.grasping_from_center_in_robot = np.array(grasping_from_center_in_robot).T
    for idx in range(self.grasping_from_center_in_robot.shape[1]):
      # Normalize
      self.grasping_from_center_in_robot[:,idx] = self.grasping_from_center_in_robot[:,idx]/np.linalg.norm(self.grasping_from_center_in_robot[:,idx])
    robot_vertices = rospy.get_param('robot_vertices')
    self.robot_vertices = [np.array(robot_vertices[name]).T for name in robot_vertices.keys() if name != 'robot_to_load']
    self.parameter_loaded = True
    rospy.loginfo("Parameter loaded")
    
  def plot_scene(self, data):
    self.config = np.array(data.data)
#    self.config = self.init
    if not self.parameter_loaded:
      self.load_parameters()

    # Plot obstacles
    self.plot(self.scene_bound, color="r")
    self.plot(self.cube_1)
    rospy.loginfo(self.cube_1)
    self.plot(self.cube_2)

    # Plot obj
    obj_world = []
    for idx in range(self.object_vertices.shape[1]):
      obj_world.append(self.vertex_to_world_from_obj(self.object_vertices[:,idx]))
    rospy.loginfo(obj_world)
    self.plot(np.column_stack(tuple(obj_world)), 'b')

    # Plot grasp obj
    for idx in range(len(self.robots)):
      self.plot(self.vertex_to_world_from_obj(self.grasping_vertices_in_obj[:,idx]), "orange")

    # Plot grasp robot
    for idx in range(len(self.robots)):
      grasp_in_world = []
      grasp_in_world.append(self.vertex_to_world_from_robot(self.grasping_from_center_in_robot[:,idx].reshape(-1,1) * self.config[3+idx], idx))
      grasp_in_world.append(self.vertex_to_world_from_robot(np.zeros((3,1)), idx))
      self.plot(np.column_stack(grasp_in_world), ["red","blue"][idx])

    # Plot robots
    for idx,_ in enumerate(self.robots):
      robot_world = []
      for jdx in range(self.robot_vertices[idx].shape[1]):
        rospy.logdebug(f"robot_vertex:\n{self.object_vertices[:,jdx]}")
        robot_world.append(self.vertex_to_world_from_robot(self.robot_vertices[idx][:,jdx], idx))
        rospy.logdebug(f"robot_vertex:\n{robot_world[jdx]}")
      self.plot(np.column_stack(robot_world), 'g')

    self.ax.set_aspect('equal', adjustable='box')
    self.ready_to_plot = True

  def vertex_to_world_from_obj(self, x: np.array):
    x.resize(3,1)
    t = np.array([self.config[0], self.config[1], 0])
    t.resize(3,1)
    rot = lambda x: np.array([[np.cos(x), -np.sin(x), 0],
                               [np.sin(x), np.cos(x), 0],
                               [0, 0, 1]])
    return t + rot(self.config[2]) @ x

  def vertex_to_world_from_robot(self, x: np.array, robot: int):
    x = x.reshape(-1,1)
    t = np.array([self.config[0], self.config[1], 0]).reshape(-1,1)
    rot = lambda x: np.array([[np.cos(x), -np.sin(x), 0],
                               [np.sin(x), np.cos(x), 0],
                               [0, 0, 1]])
    ret = t + rot(self.config[2]) @ (
          self.grasping_vertices_in_obj[:,robot].reshape(-1,1)
          + rot(self.config[3 + len(self.robots) + robot]) @ (
                x - self.config[3 + robot] * self.grasping_from_center_in_robot[:,robot].reshape(-1,1)
              )
          )
    return ret


def main():
  rospy.init_node("area_plotter")
  set_rospy_log_lvl(rospy.DEBUG)
  plotter = ScenePlotter()
  rospy.Subscriber("/formation_result", Float64MultiArray, plotter.plot_scene)
  plotter.load_parameters()

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    if plotter.ready_to_plot:
      rospy.loginfo_once("Plotting scene")
      plt.show()
    rate.sleep()

if __name__ == "__main__":
  main()

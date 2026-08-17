from tf.transformations import quaternion_from_euler,euler_from_quaternion
import geometry_msgs.msg as geometry_msgs
class Point():
  def __init__(self,x,y,angle):
    self.x = x
    self.y = y
    self.angle = angle
  x=int()
  y=int()
  angle=float()

class Coordinates():
  @classmethod
  def Grid(cls,originX,originY,posex,posey,resolution):
    gridPose=Point(0,0,0)
    gridPose.x = int(round((posex - originX) / resolution))
    gridPose.y = int(round((posey - originY) / resolution))
    return gridPose

  @classmethod
  def FloatGrid(cls,originX,originY,posex,posey,resolution):
    gridPose=Point(0,0,0)
    gridPose.x = round((posex - originX) / resolution, 2)
    gridPose.y = round((posey - originY) / resolution, 2)
    return gridPose

  @classmethod
  def GridWithAngle(cls,originX,originY,posex,posey,resolution,orientation):
    gridPose=Point(0,0,0)
    gridPose.x = int(round((posex - originX) / resolution))
    gridPose.y = int(round((posey - originY) / resolution))
    gridPose.angle = euler_from_quaternion([orientation.x, \
                                            orientation.y, \
                                            orientation.z, \
                                            orientation.w])
    return gridPose

  @classmethod
  def PoseToFloatGrid(cls,geometryPose,originX,originY,resolution):
    gridPose = Point(0,0,0)
    gridPose = cls().FloatGrid(originX,originY,geometryPose.position.x,geometryPose.position.y,resolution)
    return gridPose

  @classmethod
  def PoseToGrid(cls,geometryPose,originX,originY,resolution):
    gridPose = Point(0,0,0)
    gridPose = cls().Grid(originX,originY,geometryPose.position.x,geometryPose.position.y,resolution)
    return gridPose

  @classmethod
  def GridToPose(cls,Point,originX,originY,resolution):
    geometryPose = geometry_msgs.Pose()
    geometryPose.orientation = geometry_msgs.Quaternion(*quaternion_from_euler(0,0,Point.angle))
    geometryPose.position.x = originX + (Point.x)*resolution
    geometryPose.position.y = originY + (Point.y)*resolution
    geometryPose.position.z = 0
    return geometryPose

  @classmethod
  def UniqueGrid(cls,originGrids):
    if len(originGrids) == 0:
      return originGrids
    output = []
    output.append(originGrids[0])
    for point in originGrids:
      tempPoint = output[-1]
      if (tempPoint.x == point.x) and (tempPoint.y == point.y) :
        continue
      output.append(point)
    return output

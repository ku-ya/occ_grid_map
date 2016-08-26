#!/usr/bin/env python
class DDA:
  def __init__(self,start,end):
      self.start = start
      self.end = end
      self.path = []
      self.direction = end[1]-start[1]/(end[0]-start[0])

  def step(self):
      self.t_max_x
      self.t_max_y
      while :
          if(t_max_x<t_max_y):
              t_max_x += t_delta_x
              x += step_x
          else:
              t_max_y += t_delta_y
              y += step_y

        nextVoxel(x,y)

import math
import numpy as np

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return np.rad2deg(math.acos(dotproduct(v1, v2) / (length(v1) * length(v2))))


v1=[0,1]
v2=[1,0]

ang=angle(v1,v2)
print ang
print math.sin(2*math.pi/4)

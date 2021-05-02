import numpy as np
import math
from Obstacle import *


def halfRound(n):
    val = round(2*n)/2
    if (val == 10):
      val -= 0.5
    return val


def toRadian(angle):
    return np.pi * angle / 180
def toDegree(angle):
    return 180 * angle / np.pi
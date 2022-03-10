from threading import Thread

import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import Point


def numpyfy(point: Point) -> np.ndarray:
    return np.array([point.x, point.y, point.z])


def pointify(point: npt.ArrayLike) -> Point:
    assert len(point) == 3
    return Point(x=point[0], y=point[1], z=point[2])

def start_daemon_thread(*args, **kwargs):
    t = Thread(*args, **kwargs)
    t.daemon = True
    t.start()
    return t

"""
harita_dugumu.py - /map ve TF'i okuyan ROS 2 dugumu
"""

import math
import threading

import numpy as np

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

import tf2_ros


class HaritaDugumu(Node):

    def __init__(self):
        # TF damgalari sim saatinde, use_sim_time olmadan sorgular tutmuyor
        super().__init__(
            "otonom_kesif_harita",
            parameter_overrides=[Parameter(
                "use_sim_time", Parameter.Type.BOOL, True)])

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/map", self._harita_geldi, qos)

        self._tampon = tf2_ros.Buffer()
        self._dinleyici = tf2_ros.TransformListener(self._tampon, self)

        self._kilit = threading.Lock()
        self._izgara = None      # int16, satir y, sutun x
        self._res = None
        self._ox = None
        self._oy = None
        self._sayac = 0

    def _harita_geldi(self, msg):
        h = msg.info.height
        w = msg.info.width
        veri = np.array(msg.data, dtype=np.int16).reshape(h, w)
        with self._kilit:
            self._izgara = veri
            self._res = msg.info.resolution
            self._ox = msg.info.origin.position.x
            self._oy = msg.info.origin.position.y
            self._sayac += 1

    def harita_al(self):
        """(izgara, res, ox, oy), harita gelmediyse None."""
        with self._kilit:
            if self._izgara is None:
                return None
            return (self._izgara.copy(), self._res, self._ox, self._oy)

    def harita_sayaci(self):
        with self._kilit:
            return self._sayac

    def robot_konumu(self):
        """map cercevesinde (x, y, yaw), TF yoksa None. yaw cerceve acisi icin lazim."""
        try:
            t = self._tampon.lookup_transform(
                "map", "base_link", Time())
        except Exception:
            return None
        q = t.transform.rotation
        # kuaterniyondan yaw
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

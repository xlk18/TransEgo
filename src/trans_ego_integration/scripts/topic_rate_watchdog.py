#!/usr/bin/env python3
import rospy
from collections import deque
from std_msgs.msg import Header


class RateWatchdog:
    def __init__(self):
        topics_csv = rospy.get_param("~topics", "/dynamic_cloud,/mot_tracks,/predicted_trajectories")
        self.topics = [t.strip() for t in topics_csv.split(",") if t.strip()]
        self.min_rate_hz = float(rospy.get_param("~min_rate_hz", 10.0))
        self.window_sec = float(rospy.get_param("~window_sec", 2.0))
        self.buffers = {t: deque() for t in self.topics}
        self.subs = []

        for topic in self.topics:
            self.subs.append(rospy.Subscriber(topic, rospy.AnyMsg, self._cb, callback_args=topic, queue_size=100))

        self.timer = rospy.Timer(rospy.Duration(1.0), self._check)

    def _cb(self, _msg, topic):
        now = rospy.Time.now().to_sec()
        q = self.buffers[topic]
        q.append(now)
        while q and (now - q[0]) > self.window_sec:
            q.popleft()

    def _check(self, _evt):
        now = rospy.Time.now().to_sec()
        ok_all = True
        for topic in self.topics:
            q = self.buffers[topic]
            while q and (now - q[0]) > self.window_sec:
                q.popleft()
            rate = (len(q) / self.window_sec) if self.window_sec > 1e-3 else 0.0
            if rate < self.min_rate_hz:
                ok_all = False
                rospy.logwarn("[QA] topic=%s rate=%.2fHz < %.2fHz", topic, rate, self.min_rate_hz)
            else:
                rospy.loginfo("[QA] topic=%s rate=%.2fHz", topic, rate)

        if ok_all:
            rospy.loginfo_throttle(5.0, "[QA] All topics satisfy min rate %.2fHz", self.min_rate_hz)


if __name__ == "__main__":
    rospy.init_node("topic_rate_watchdog")
    RateWatchdog()
    rospy.spin()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import time
from std_msgs.msg import Float64MultiArray
from nlink_parser.msg import LinktrackAnchorframe0  # <-- your new type

class RangesTalkerFloat(object):
    def __init__(self):
        rospy.init_node('ranges_talker_float', anonymous=True)

        # Params
        self.in_topic    = rospy.get_param('~in_topic',  '/nlink_linktrack_anchorframe0')
        self.out_topic   = rospy.get_param('~out_topic', '/anchors_ranges')
        # ts_mode: 'none' | 'append_sec' | 'append_sec_nsec'
        # self.ts_mode     = rospy.get_param('~ts_mode',   'append_sec')
        # time_source: 'ros' (rospy.Time.now) | 'wall' (time.time)
        # self.time_source = rospy.get_param('~time_source', 'ros')
        # treat 0.0 distances as "not detected"
        self.zeros_as_nan = rospy.get_param('~zeros_as_nan', True)

        self.expected_len = 8
        self.latest = [float('nan')] * self.expected_len
        self.seen_any_msg = False

        self.pub = rospy.Publisher(self.out_topic, Float64MultiArray, queue_size=10)
        self.sub = rospy.Subscriber(self.in_topic, LinktrackAnchorframe0, self._cb, queue_size=10)

        # rospy.loginfo("RangesTalkerFloat: in=%s  out=%s  ts_mode=%s  time_source=%s  zeros_as_nan=%s",
        #               self.in_topic, self.out_topic, self.ts_mode, self.time_source, self.zeros_as_nan)

        # Publish at 10 Hz regardless
        # self.timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)

    def _cb(self, msg):
        # Expect msg.nodes[0].dis_arr (8 floats, 0.0 if not detected)
        if not hasattr(msg, 'nodes') or len(msg.nodes) == 0 or not hasattr(msg.nodes[0], 'dis_arr'):
            rospy.logwarn_throttle(5.0, "Missing nodes[0].dis_arr on %s", self.in_topic)
            return

        arr = list(msg.nodes[0].dis_arr)
        if len(arr) < self.expected_len:
            arr += [float('nan')] * (self.expected_len - len(arr))
        arr = arr[:self.expected_len]

        if self.zeros_as_nan:
            arr = [float('nan') if (not math.isfinite(x) or x == 0.0) else float(x) for x in arr]
        else:
            arr = [float('nan') if not math.isfinite(x) else float(x) for x in arr]

        self.latest = arr
        self.seen_any_msg = True
        data = self.latest
        data.append(rospy.Time.now().to_sec())
        # data.append(rospy.Time.now().to_sec())
        msg_out = Float64MultiArray()
        msg_out.data = data
        # msg_out.layout.data_offset =rospy.Time.now().to_sec()
        print(msg_out)
        self.pub.publish(msg_out)


    # def _append_timestamp(self, data_list):
    #     if self.ts_mode == 'append_sec':
    #         if self.time_source == 'wall':
    #             data_list.append(float(time.time()))
    #         else:
    #             data_list.append(rospy.Time.now().to_sec())
    #     elif self.ts_mode == 'append_sec_nsec':
    #         if self.time_source == 'wall':
    #             t = time.time()
    #             secs = int(t)
    #             nsecs = int((t - secs) * 1e9)
    #         else:
    #             now = rospy.Time.now()
    #             secs, nsecs = now.secs, now.nsecs
    #         data_list.append(float(secs))
    #         data_list.append(float(nsecs))

    # def _on_timer(self, _event):
    #     data = list(self.latest)
    #     self._append_timestamp(data)
    #
    #     msg_out = Float32MultiArray()
    #     msg_out.data = data
    #     self.pub.publish(msg_out)
    #
    #     if not self.seen_any_msg:
    #         rospy.logwarn_throttle(5.0,
    #             "No data yet from %s; publishing NaNs. Verify topic and message layout.",
    #             self.in_topic)

if __name__ == '__main__':
    try:
        RangesTalkerFloat()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
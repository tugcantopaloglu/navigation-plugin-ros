#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Polygon, Point32

class RvizPolygonPublisher:
    def __init__(self):
        rospy.init_node('rviz_polygon_publisher', anonymous=True)

        self.clicked_points = []
        self.num_required_points = 4

        self.output_polygon_topic = rospy.get_param('~output_polygon_topic', '/coverage_area_polygon')
        self.clicked_point_topic = rospy.get_param('~clicked_point_topic', '/clicked_point')

        self.point_subscriber = rospy.Subscriber(
            self.clicked_point_topic,
            PointStamped,
            self.clicked_point_callback
        )

        self.polygon_publisher = rospy.Publisher(
            self.output_polygon_topic,
            Polygon,
            queue_size=10
        )

        rospy.loginfo("RVIZ Polygon Publisher initialized.")
        rospy.loginfo("Listening for 4 points on topic: %s", self.clicked_point_topic)
        rospy.loginfo("Will publish Polygon to topic: %s", self.output_polygon_topic)

    def clicked_point_callback(self, msg):
        point = msg.point
        self.clicked_points.append(point)
        rospy.loginfo("Point %d/%d received: (%.2f, %.2f, %.2f)",
                      len(self.clicked_points),
                      self.num_required_points,
                      point.x, point.y, point.z)

        if len(self.clicked_points) == self.num_required_points:
            self.publish_polygon()
            self.clicked_points = []

    def publish_polygon(self):
        if len(self.clicked_points) != self.num_required_points:
            rospy.logwarn("Not enough points to form a polygon.")
            return

        polygon_msg = Polygon()
        
        for pt_stamped in self.clicked_points:
            pt32 = Point32()
            pt32.x = pt_stamped.x
            pt32.y = pt_stamped.y
            pt32.z = pt_stamped.z
            polygon_msg.points.append(pt32)

        self.polygon_publisher.publish(polygon_msg)
        rospy.loginfo("Polygon published with %d points to %s",
                      len(polygon_msg.points), self.output_polygon_topic)

if __name__ == '__main__':
    try:
        publisher_node = RvizPolygonPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

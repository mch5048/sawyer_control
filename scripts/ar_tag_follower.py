import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from sawyer_control.srv import *

class ARFollower():
    def __init__(self):
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        self.target_visible = False

        # Wait for the ar_pose_marker topic to become available
        rospy.loginfo("Waiting for ar_pose_marker topic...")
        rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

        # Subscribe to the ar_pose_marker topic to get the image width and height
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.set_cmd_vel)

        rospy.loginfo("Marker messages detected. Starting follower...")


    def set_cmd_vel(self, msg):
        # Pick off the first marker (in case there is more than one)
        try:
            self.markers = msg.markers
            if not self.target_visible:
                rospy.loginfo("FOLLOWER is Tracking Target!")
            self.target_visible = True
        except:
            if self.target_visible:
                rospy.loginfo("FOLLOWER LOST Target!")
            self.target_visible = False

            return
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)

def follow_tags(unused):
    for marker in ar.markers:
        pos = marker.pose.pose.position
        orientation = marker.pose.pose.orientation
        if marker.id == 1:
            left_marker_pose = [pos.x, pos.y, pos.z, orientation.x, orientation.y, orientation.z, orientation.w]
        if marker.id == 0:
            middle_marker_pose = [pos.x, pos.y, pos.z, orientation.x, orientation.y, orientation.z, orientation.w]
        if marker.id == 2:
            right_marker_pose = [pos.x, pos.y, pos.z, orientation.x, orientation.y, orientation.z, orientation.w]
    return ar_tagResponse(left_marker_pose, right_marker_pose, middle_marker_pose)

def ar_tag_server():
    rospy.init_node("ar_follower", anonymous=True)
	global ar
    ar = ARFollower()
    s = rospy.Service('ar_tag', ar_tag, follow_tags)
    rospy.spin()
if __name__ == '__main__':
    ar_tag_server()

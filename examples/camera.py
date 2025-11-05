from colibricf.camera import Camera
import rospy

rospy.init_node('cv')
Camera().draw_face()



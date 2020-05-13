
import rospy
from tbd_ros_msgs.msg import (
    faceAnimationGoal,
    faceAnimationAction,
    FaceAnimationObject,
)
import actionlib 


if __name__ == "__main__":
    rospy.init_node("test_animation")

    client = actionlib.SimpleActionClient('animation', faceAnimationAction)
    client.wait_for_server()
    goal = faceAnimationGoal()
    goal.objects = [
        FaceAnimationObject(type=FaceAnimationObject.LINEAR, name="left_eye.center", target=[0.20, 0.35], duration=1)
    ]
    
    rospy.loginfo("Sending first goal ....")
    client.send_goal_and_wait(goal)

    goal.objects = [
        FaceAnimationObject(type=FaceAnimationObject.LINEAR, name="left_eye.center", target=[0.15, 0.35], duration=2),
        FaceAnimationObject(type=FaceAnimationObject.LINEAR, name="right_eye.center", target=[0.65, 0.35], duration=2)
    ]
    rospy.loginfo("Sending second goal ....")
    client.send_goal(goal)
    rospy.sleep(1)
    client.cancel_all_goals()

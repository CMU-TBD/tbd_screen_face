
import rospy
from tbd_ros_msgs.msg import (
    faceAnimationGoal,
    faceAnimationAction,
    FaceAnimationObject,
    faceAnimationResult
)
import actionlib 
import pytest


@pytest.fixture
def shutdown(request, scope='session'):
    rospy.init_node("pytest_node")
    def fin():
        rospy.signal_shutdown("completed")
    request.addfinalizer(fin)
    return None

def test_connection(shutdown):
    client = actionlib.SimpleActionClient('animation', faceAnimationAction)
    client.wait_for_server()
    goal = faceAnimationGoal()
    goal.objects = [
        FaceAnimationObject(type=FaceAnimationObject.LINEAR, name="left_eye.center", target=[0.20, 0.35], duration=1)
    ]
    client.send_goal_and_wait(goal)
    result = client.get_result()
    assert result.success


    goal.objects = [
        FaceAnimationObject(type=FaceAnimationObject.LINEAR, name="left_eye.center", target=[0.15, 0.35], duration=2),
        FaceAnimationObject(type=FaceAnimationObject.LINEAR, name="right_eye.center", target=[0.65, 0.35], duration=2)
    ]
    rospy.loginfo("Sending second goal ....")
    client.send_goal(goal)
    rospy.sleep(1)
    client.cancel_all_goals()

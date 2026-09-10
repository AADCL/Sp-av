"""Isolated ROS service call entry point for deadline-enforced processes."""


def _pose(values):
    from geometry_msgs.msg import PoseStamped
    import rospy

    message = PoseStamped()
    stamp, frame_id, x, y, z, qx, qy, qz, qw = values
    message.header.stamp = rospy.Time.from_sec(stamp) if stamp > 0.0 else rospy.Time()
    message.header.frame_id = frame_id
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = z
    message.pose.orientation.x = qx
    message.pose.orientation.y = qy
    message.pose.orientation.z = qz
    message.pose.orientation.w = qw
    return message


def ros_service_bootstrap(navigation_name, control_name, availability_timeout):
    """Initialize ROS once and return a sequential typed service dispatcher."""
    import rospy
    import sys
    from ducted_msgs.srv import FlightCommand
    from ducted_navigation.srv import Navigate

    rospy.init_node(
        "ducted_mission_service_call", argv=[sys.argv[0]],
        anonymous=True, disable_signals=True)
    rospy.wait_for_service(navigation_name, timeout=availability_timeout)
    rospy.wait_for_service(control_name, timeout=availability_timeout)
    navigation = rospy.ServiceProxy(navigation_name, Navigate, persistent=True)
    control = rospy.ServiceProxy(control_name, FlightCommand, persistent=True)

    def dispatch(kind, command, request_id, target_values):
        nonlocal navigation, control
        if kind == "navigation":
            try:
                response = navigation(command, request_id, _pose(target_values))
            except Exception:
                navigation = rospy.ServiceProxy(
                    navigation_name, Navigate, persistent=True)
                raise
        elif kind == "control":
            try:
                response = control(command, _pose(target_values))
            except Exception:
                control = rospy.ServiceProxy(
                    control_name, FlightCommand, persistent=True)
                raise
        else:
            return False, "unknown service kind"
        return bool(response.accepted), str(response.message)

    return dispatch

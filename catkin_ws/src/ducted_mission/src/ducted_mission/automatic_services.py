"""Typed dispatcher in a killable helper process; never retries a command."""
def bootstrap(timeout):
    import sys
    import rospy
    from std_srvs.srv import Trigger
    from ducted_msgs.srv import FlightCommand
    from ducted_navigation.srv import Navigate
    from geometry_msgs.msg import PoseStamped
    rospy.init_node('automatic_service_helper',argv=[sys.argv[0]],anonymous=True,disable_signals=True)
    names=('/ducted/control/command','/ducted/mission/start','/ducted/mission/cancel','/ducted/navigation/command')
    for name in names:rospy.wait_for_service(name,timeout=timeout)
    control=rospy.ServiceProxy(names[0],FlightCommand)
    start=rospy.ServiceProxy(names[1],Trigger)
    cancel=rospy.ServiceProxy(names[2],Trigger)
    navigation=rospy.ServiceProxy(names[3],Navigate)
    def call(command,z):
        target=PoseStamped();target.header.frame_id='odom';target.header.stamp=rospy.Time.now()
        target.pose.orientation.w=1.;target.pose.position.z=z
        if command=='mission_start':
            r=start();return r.success,r.message
        if command in ('stop','fault_stop'):
            # Hold/abort must be attempted even when mission cancellation fails.
            errors=[]
            try:cancel()
            except Exception as e:errors.append('mission cancel: '+str(e))
            try:
                r=control('abort',target)
                if not r.accepted:errors.append('control abort: '+r.message)
            except Exception as e:errors.append('control abort: '+str(e))
            return not errors,'; '.join(errors)
        r=control(command,target);return r.accepted,r.message
    return call

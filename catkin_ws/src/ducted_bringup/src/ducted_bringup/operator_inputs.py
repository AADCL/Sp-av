"""Bounded operator reads that do not confuse a latched FCU state with a new heartbeat."""
import math
import threading
import time


def wait_for_fresh_message(ros, topic, kind, limit=None, wait_timeout=6.):
    if limit is None: limit=1.5 if topic=='/mavros/state' else .5
    if not all(math.isfinite(v) and v>0 for v in (limit,wait_timeout)):
        raise ValueError('invalid operator input timeout')
    subscribed_at=ros.get_time()
    require_new=topic in ('/mavros/state','/mavros/extended_state')
    done=threading.Event();lock=threading.Lock();result=[]
    reason=['no message received']

    def receive(msg):
        now=ros.get_time()
        if hasattr(msg,'header'):
            stamp=msg.header.stamp.to_sec()
            if not math.isfinite(stamp) or stamp<=0 or not math.isfinite(now):
                reason[0]='invalid source timestamp';return
            age=now-stamp
            if not -.05<=age<=limit:
                reason[0]='source age %.3f s outside [-0.05, %.2f] s'%(age,limit);return
            if require_new and stamp<=subscribed_at:
                reason[0]='waiting for a new FCU heartbeat after subscription';return
        with lock:
            result[:]=[msg]
        done.set()

    subscriber=ros.Subscriber(topic,kind,receive,queue_size=1)
    deadline=time.monotonic()+wait_timeout
    try:
        while not ros.is_shutdown():
            if ros.get_time()<subscribed_at-.05:
                raise RuntimeError(topic+' ROS clock moved backwards')
            remaining=deadline-time.monotonic()
            if remaining<=0:break
            if done.wait(min(.05,remaining)):
                with lock:msg=result[-1]
                if hasattr(msg,'header'):
                    age=ros.get_time()-msg.header.stamp.to_sec()
                    if not -.05<=age<=limit:
                        done.clear();continue
                return msg
        raise RuntimeError(topic+' fresh message timeout: '+reason[0])
    finally:
        subscriber.unregister()

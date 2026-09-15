"""One read-only EGO request in flight; fresh-cloud guards never wait for it."""
import math
import re
import threading


class EgoReferenceWorker:
    def __init__(self, client, wake, clock, timeout=.5):
        self.client,self.wake,self.clock,self.timeout=client,wake,clock,timeout
        self.lock=threading.Lock()
        self.key=None
        self.busy=False
        self.last_stamp=0.
        self.result=None
        self.error=None
        self.issued={}

    def reset(self):
        with self.lock:
            self.key=None;self.last_stamp=0.;self.result=None;self.error=None;self.issued.clear()

    def expired_error(self, error):
        if error=='EGO planning unavailable: EGO result exceeded snapshot deadline':
            return True
        if not error.startswith('EGO planning unavailable: invalid or stale EGO snapshot:'):
            return False
        match=re.search(r'\bage=([^ ]+)',error)
        try:return bool(match) and float(match.group(1))>self.timeout
        except ValueError:return False

    def _current(self):
        if self.error:raise ValueError(self.error)
        if self.result is None:return None
        reference,stamp,wall_deadline=self.result
        ros,wall=self.clock()
        if (not math.isfinite(ros+wall) or not -.05<=ros-stamp<=self.timeout
                or wall>wall_deadline):return None
        return reference,stamp

    def poll(self, snapshot, key):
        with self.lock:
            if key!=self.key:
                self.key=key;self.last_stamp=0.;self.result=None;self.error=None;self.issued.clear()
            current=self._current()
            if self.busy or snapshot.stamp<=self.last_stamp:return current
            self.busy=True;self.last_stamp=snapshot.stamp
            ros,wall=self.clock()
            deadline=wall+self.timeout-max(0.,ros-snapshot.stamp)
        # Preparation alone touches the guard's serialized history. The worker
        # owns a prepared request and cannot mutate live planning state.
        try:packet=self.client.prepare(snapshot,key[0])
        except Exception as error:
            with self.lock:
                self.busy=False
                if self.key==key:self.error=str(error);self.result=None
            raise ValueError(str(error)) from error
        def work():
            reference,error=None,None
            try:reference=self.client.execute(packet)
            except Exception as failure:error=str(failure)
            with self.lock:
                self.busy=False
                if self.key==key:
                    if error is None:
                        self.result=(reference,snapshot.stamp,deadline)
                        self.issued[snapshot.stamp]=deadline
                        while len(self.issued)>8:self.issued.pop(min(self.issued))
                    elif not self.expired_error(error):self.error=error;self.result=None
            self.wake()
        threading.Thread(target=work,name='ego-reference',daemon=True).start()
        with self.lock:return self._current()

    def commit_reference(self, key, stamp, publish):
        # A failure/new generation arriving during the last cloud check cannot
        # race a cached reference back onto the target stream.
        with self.lock:
            ros,wall=self.clock()
            if (self.key!=key or self.error or stamp not in self.issued
                    or not math.isfinite(ros+wall) or not -.05<=ros-stamp<=self.timeout
                    or wall>self.issued[stamp]):return False
            return publish()

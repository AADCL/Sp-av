"""One outstanding read-only planning RPC, with a monotonic caller timeout."""
import threading


class BoundedRPC:
    def __init__(self, call, timeout=.25):
        if timeout<=0 or timeout>.5:
            raise ValueError("invalid planner RPC timeout")
        self.call,self.timeout=call,timeout
        self._pending=None

    def request(self, value):
        if self._pending is not None:
            if not self._pending[0].is_set():
                raise RuntimeError("previous planner RPC still unresolved")
            self._pending=None  # discard a response whose caller already timed out
        done=threading.Event();result=[]
        def worker():
            try: result.append((True,self.call(value)))
            except Exception as error: result.append((False,error))
            finally: done.set()
        self._pending=(done,result)
        threading.Thread(target=worker,name="read-only-planner-rpc",daemon=True).start()
        if not done.wait(self.timeout):
            raise RuntimeError("planning RPC deadline exceeded")
        self._pending=None
        success,response=result[0]
        if not success: raise RuntimeError(str(response))
        return response

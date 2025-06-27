import threading
import collections
import time
import gc
import numpy

class myclass():
    
    def __init__(self):
        self.t = threading.Thread(target=self.myrun, daemon=True)
        #
        self.stop_thread = False
        self.q           = collections.deque(maxlen=1)
        self.q.append(0)
        #
        self.t.start()
        print("End of __init__")

    def myrun(self):
        print("Start of run")
        rng = numpy.random.default_rng()
        while not self.stop_thread:
            r = rng.integers(1,100,endpoint=True)
            print(f"Queued    : {r: #3}")
            self.q.append(r)
            time.sleep(0.02)
        print("End of run")
    
    def stop(self):
        self.stop_thread = True
        self.t.join()
        print("End of stop")

    def do_something(self):
        print(f"Received  : {self.q[0]: #3}")
        return

# ==============================================================
if __name__ == '__main__':  
    c = myclass()
    time.sleep(0.5)
    c.do_something()
    time.sleep(0.5)
    c.do_something()
    c.stop()


    g = gc.collect()
    print(g)
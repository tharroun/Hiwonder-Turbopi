#!/usr/bin/python3
# coding=utf8
import time
import threading
from queue import Queue
import numpy

KILL_THREAD = object() 
DO_NOTHING  = object()

global_variable = 0.0

# ==============================================================
def robot_servo(qs) -> int:
    global global_variable

    (x,y) = qs.get()
    qs.task_done()
    print(f"ROBOT_SERVO : {x}, {y}")
    time.sleep(1)

    while True:
        data = qs.get()
        # ----------------------------------------------------------
        # THE TASK
        if data is KILL_THREAD:
            qs.task_done()
            break
        elif data is DO_NOTHING:
            pass
        else :
            print(f"ROBOT_SERVO : {data[0]}, {data[1]}, {global_variable}")
            time.sleep(1)
        # ----------------------------------------------------------
        qs.task_done()
    
    #-------
    return 1 
# ==============================================================

# ==============================================================
def robot_mecanum(qm) -> int:
    global global_variable

    (x,y) = qm.get()
    qm.task_done()
    print(f"ROBOT_MECANUM : {x}, {y}")
    time.sleep(1)

    while True:
        data = qm.get()
        # ----------------------------------------------------------
        # THE TASK
        if data is KILL_THREAD:
            qm.task_done()
            break
        elif data is DO_NOTHING:
            pass
        else :
            print(f"ROBOT_MECANUM : {data[0]}, {data[1]}, {global_variable}")
            time.sleep(1)
        # ----------------------------------------------------------
        qm.task_done()
    
    #-------
    return 1 
# ==============================================================

# ==============================================================
def robot_see(qs,qm) -> int:
    global global_variable

    rng = numpy.random.default_rng()

    x = rng.integers(1,10)
    y = rng.integers(11,20)
    qs.put((x,y))
    qm.put((x,y))

    for i in range(10):
        x = rng.integers(1,10)
        y = rng.integers(11,20)
        r = rng.random()
        if r < 0.5 :
            global_variable = r
            print(f"ROBOT SEE : DO_NOTHING {global_variable}")
            qs.put(DO_NOTHING)
            qm.put(DO_NOTHING)
        else:
            qs.put((x,y,r))
            qm.put((x,y,r))
        time.sleep(0.5)
    # ------------------------------------------------
    qs.put(KILL_THREAD)
    qm.put(KILL_THREAD)
    return 1
# ==============================================================

# ==============================================================
if __name__ == '__main__':


    qs = Queue() 
    qm = Queue()

    thread_vision = threading.Thread(target=robot_see, args=(qs,qm,))
    thread_vision.start()

    thread_servo   = threading.Thread(target=robot_servo, args=(qs,))
    thread_mecanum = threading.Thread(target=robot_mecanum, args=(qm,))
    thread_servo.start()
    thread_mecanum.start()

    thread_vision.join()
    qs.join()
    qm.join()

    print(f"Queue Servo   : {qs.qsize()}")
    print(f"Queue Mecanum : {qm.qsize()}")

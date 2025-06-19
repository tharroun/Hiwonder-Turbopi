#!/usr/bin/python3
# coding=utf8
import time
import multiprocessing
from enum import Enum
import numpy

class namespace_values(Enum):
    KILL_PROCESS = 1 
    DO_NOTHING   = 2

# ==============================================================
def robot_servo(qs,Global) -> int:

    (x,y) = qs.get()
    qs.task_done()
    print(f"ROBOT_SERVO : {x}, {y}")
    time.sleep(1)

    while True:
        data = qs.get()
        # ----------------------------------------------------------
        # THE TASK
        if data is namespace_values.KILL_PROCESS:
            qs.task_done()
            break
        elif data is namespace_values.DO_NOTHING:
            pass
        else :
            print(f"ROBOT_SERVO : {data} {Global.global_variable}")
            time.sleep(1)
        # ----------------------------------------------------------
        qs.task_done()
    
    #-------
    return 1 
# ==============================================================

# ==============================================================
def robot_mecanum(qm,Global) -> int:

    (x,y) = qm.get()
    qm.task_done()
    print(f"ROBOT_MECANUM : {x}, {y}")
    time.sleep(1)

    while True:
        data = qm.get()
        # ----------------------------------------------------------
        # THE TASK
        if data is namespace_values.KILL_PROCESS:
            qm.task_done()
            break
        elif data is namespace_values.DO_NOTHING:
            pass
        else :
            print(f"ROBOT_MECANUM : {data} {Global.global_variable}")
            time.sleep(1)
        # ----------------------------------------------------------
        qm.task_done()
    
    #-------
    return 1 
# ==============================================================

# ==============================================================
def robot_see(qs,qm,Global) -> int:

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
            Global.global_variable = r
            print(f"ROBOT SEE : DO_NOTHING {Global.global_variable}")
            qs.put(namespace_values.DO_NOTHING)
            qm.put(namespace_values.DO_NOTHING)
        else:
            qs.put((x,y,r))
            qm.put((x,y,r))
        time.sleep(0.5)
    # ------------------------------------------------
    qs.put(namespace_values.KILL_PROCESS)
    qm.put(namespace_values.KILL_PROCESS)
    qm.close()
    qs.close()
    return 1
# ==============================================================

# ==============================================================
if __name__ == '__main__':

    Manager                = multiprocessing.Manager()
    Global                 = Manager.Namespace()
    Global.global_variable = 0.1

    qs = multiprocessing.JoinableQueue() 
    qm = multiprocessing.JoinableQueue()

    thread_vision = multiprocessing.Process(target=robot_see, args=(qs,qm,Global))
    thread_vision.start()

    thread_servo   = multiprocessing.Process(target=robot_servo, args=(qs,Global))
    thread_mecanum = multiprocessing.Process(target=robot_mecanum, args=(qm,Global))
    thread_servo.start()
    thread_mecanum.start()

    thread_vision.join()
    qs.join()
    qm.join()

    thread_servo.terminate()
    thread_mecanum.terminate()
    thread_vision.terminate()

    print(f"Queue Servo   : {qs.qsize()}")
    print(f"Queue Mecanum : {qm.qsize()}")

import time
import threading
from queue import Queue

KILL_THREAD = object() 

# ==============================================================
def robot_servo(qs) -> int:

    (x,y) = qs.get()
    assert (x,y)==(1,2) , "ERROR MECANUM"
    qs.task_done()

    while True:
        data = qs.get()
        if data is KILL_THREAD:
            qs.task_done()
            break
        print(data)
        time.sleep(2)
        qs.task_done()

    return 1 
# ==============================================================

# ==============================================================
def robot_mecanum(qm) -> int:

    (x,y) = qm.get()
    assert (x,y)==(1,2) , "ERROR MECANUM"
    qm.task_done()

    while True:
        data = qm.get()
        if data is KILL_THREAD:
            qm.task_done()
            break
        print(data)
        time.sleep(2)
        qm.task_done()

    return 1 
# ==============================================================

# ==============================================================
def robot_see(qs,qm) -> int:

    qs.put((1,2))
    qm.put((1,2))

    for i in range(5):
        qs.put(('servo',i))
        qm.put(('mecanum',i))
        time.sleep(1)
 
    qs.put(KILL_THREAD)
    qm.put(KILL_THREAD)
    return 1 
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

    #thread_vision.join()

    qs.join()
    qm.join()
    print(qs.qsize(),qm.qsize())
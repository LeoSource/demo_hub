import signal
import threading
import time
import sys

def signal_handler(signal, frame):
    # 终止所有正在运行的线程
    for thread in threading.enumerate():
        if thread.is_alive():
            thread.do_run = False
    print('程序已结束')
    sys.exit(0)

def forever_loop():
    while True:
        print('Running subthread...')
        time.sleep(1)

class MyThread(threading.Thread):
    def __init__(self, name):
        threading.Thread.__init__(self, name=name)
        self._stop_event = threading.Event()
        self.do_run = True

    def stop(self):
        self._stop_event.set()

    def run(self):
        while self.do_run:
            print('线程 %s 正在运行' % self.getName())
            time.sleep(1)

if __name__ == '__main__':
    # signal.signal(signal.SIGINT, signal_handler)
    t = time.strftime('%H%M%S',time.localtime())
    t1 = threading.Thread(target=forever_loop)
    t1.daemon = True
    t1.start()
    # forever_loop()
    while True:
    #     print('Running...')
        time.sleep(1)
    # threads = []
    # for i in range(5):
    #     t = MyThread(str(i))
    #     t.start()
    #     threads.append(t)

    # for t in threads:
    #     t.join()
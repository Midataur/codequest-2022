from multiprocessing import Queue

class NetworkManager:

    send: Queue
    recv: Queue

    @classmethod
    def set_queues(cls, send_queue, recv_queue):
        cls.send = send_queue
        cls.recv = recv_queue
    
    @classmethod
    def send_obj(cls, obj):
        cls.send.put(obj)
    
    @classmethod
    def recv_obj(cls):
        return cls.recv.get()

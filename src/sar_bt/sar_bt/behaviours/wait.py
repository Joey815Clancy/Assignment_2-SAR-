import time
import py_trees

class Wait(py_trees.behaviour.Behaviour):
    def __init__(self, name, duration=1.0):
        super().__init__(name)
        self.duration = duration
        self.start_time = None

    def update(self):
        if self.start_time is None:
            self.start_time = time.time()
            return py_trees.common.Status.RUNNING

        if time.time() - self.start_time >= self.duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.start_time = None
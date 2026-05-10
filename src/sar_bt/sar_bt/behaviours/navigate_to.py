import py_trees
from rclpy.action import ActionClient
from sar_msgs.action import NavigateTo as NavigateToAction


class NavigateTo(py_trees.behaviour.Behaviour):

    def __init__(self, name, node, target, stopping_distance=0.1):
        super().__init__(name)
        self.node = node
        self.target = target
        self.stopping_distance = stopping_distance
        self._action_client = None # action client 
        self._goal_request_pending = None # holds pending goal request 
        self._goal_request_accepted = None #holds accepted goal 
        self._result = None # holds the pending result
        self._sent = False # tracks if goal has been sent

    def setup(self, **kwargs):
        self._action_client = ActionClient(self.node, NavigateToAction, 'navigate_to')

    def update(self):
        ## Step 1 - send the goal if we haven't yet ##
        if not self._sent:
            goal = NavigateToAction.Goal() # create empty goal 
            goal.target = self.target # sets the target 
            goal.stopping_distance = self.stopping_distance #sets stoppping distance
            self._goal_request_pending = self._action_client.send_goal_async(goal) # send goal request
            self._sent = True # Set send status as true 
            return py_trees.common.Status.RUNNING # Returns running

        ## Step 2 - wait for the server to accept the goal ##
        if not self._goal_request_pending.done(): # Sever hasnt accepted 
            return py_trees.common.Status.RUNNING # Returns running 

        ## Step 3 - check if goal was accepted, get result future ##
        if self._goal_request_accepted is None: # goal is acknowleged
            self._goal_request_accepted = self._goal_request_pending.result() # get the result
            if not self._goal_request_accepted.accepted: # if it is rejected
                return py_trees.common.Status.FAILURE # returns failure
            self._result = self._goal_request_accepted.get_result_async() # wait for result
            return py_trees.common.Status.RUNNING # returns running

        # Step 4 - wait for the result
        if not self._result.done(): # Robot is still running 
            return py_trees.common.Status.RUNNING # return running

        # Step 5 - return SUCCESS or FAILURE based on result
        result = self._result.result().result # get final result 
        if result.success: # if it succeeds 
            return py_trees.common.Status.SUCCESS # return success
        return py_trees.common.Status.FAILURE # else return failure 

    def terminate(self, new_status): # Reset node to be used again
        self._sent = False
        self._goal_request_pending = None
        self._goal_request_accepted = None
        self._result = None

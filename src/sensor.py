import rospy
import threading
import numpy as np

from depth_jump_sensor.msg import DepthJump
from gap_sensor.msg import CriticalEvent, CriticalEvents, MovedGaps, GapMove

from critical_event_enum import CriticalEventEnum

class GapSensor:
    """
    - Determins which of the given depth jumps from the topic depth jumps is a valid gap.
      A valid gap needs to be wide enough for the robot to fit through. Node keeps track of all
      but marks non valid ones so .
    - Recignises appear, disappear, split, merge and publish
    """

    def __init__(self):
        rospy.init_node("gap_sensor")
        
        self.last_depth_jumps = None
        self.last_scan = None
        self.last_rotation = None
        self.last_movement = None

        self.critical_events = CriticalEvents()
        self.moved_gaps = MovedGaps()

        self.lock = threading.Lock()

    def _init_subscribers(self):
        """
        Initialise subscribers
        """
        self._sub_depth_jumps = rospy.Subscriber("depth_jumps", DepthJump, self._receive_depth_jumps)

    def _init_publisher(self):
        """
        Initialise publisher
        """
        self.pub_critical_events = rospy.Publisher("critical_events", CriticalEvents)
        self.pub_moved_gaps = rospy.Publisher("moved_gaps", MovedGaps)
        self.pub_critical_event = rospy.Publisher("critical_event", CriticalEvent)
        self.pub_gap_move = rospy.Publisher("gap_move", GapMove)

    def _receive_depth_jumps(self, data):
        """
        Receive depth jump data
        """
        if self.last_rotation == None:
            # initialise
            self.last_rotation = 0
            self.last_depth_jumps = np.zeros(len(depth_jump_data.depth_jumps))
            self.last_scan = np.zeros(len(depth_jump_data.depth_jumps))
            self.last_movement = 0

        self._process(data)

    def run(self):
        while not rospy.is_shutdown():
            pass

    def _process(self, depth_jump_data):
        self.lock.acquire()
        try:
            # initialise msgs empty
            self.critical_events = CriticalEvents()
            self.critical_events.events = []
            self.moved_gaps = MovedGaps()
            self.moved_gaps.gap_moves =[]

            # get data from msg
            depth_jumps = depth_jump_data.depth_jumps
            scan = depth_jump_data.range_data
            rotation = depth_jump_data.rotation
            movement = depth_jump_data.liniear_x

            # TODO Filter depth jumps to get the valid gaps. Depth jump is a gap if the robot fits throuh it.

            self._detect_critical_events(depth_jumps_last, rotation_last, movement_last, depth_jumps, rotation, movement)

            if len(self.critical_events.events) > 0:
                self.pub_critical_events.publish(self.critical_events)

            if len(self.moved_gaps.gap_moves) > 0:
                self.pub_moved_gaps.publish(self.moved_gaps)

        except Exception as ex:
            print(ex.message)

        self.lock.release()

    def _detect_critical_events(self, depth_jumps_last, rotation_last, movement_last, depth_jumps, rotation, movement):
        """
        Detection of the critical events appear, disappear, split, merge. Further more, notice where a depth jump has moved.
        """
        # rotation
        if shift > 0:
            self.match_rotation(0, len(depth_jumps_last), 1, depth_jumps_last, depth_jumps)
        elif shift < 0:    
            self.match_rotation(len(depth_jumps_last) - 1, -1, -1, depth_jumps_last, depth_jumps)
        elif movement == 0:
            self._match_drift_while_still_stand(depth_jumps_last, depth_jumps)

        # forwards backwards  
        if movement != 0:
            self._match_forward_backwards(depth_jumps_last, depth_jumps, movement)

    def match_rotation(self, start_index, end_index, increment, array_detected_depth_jumps):
        """
        Match the depth jumps from previous step with the current.
        """
        for i in range(start_index, end_index, increment):
            if ((self.gnt_root.depth_jumps[i % len(array_detected_depth_jumps)] != None and array_detected_depth_jumps[i % len(array_detected_depth_jumps)] == 0)):                
                index = i
                index_new = None

                for j in range(0,3):
                    if array_detected_depth_jumps[(index - increment * j) % len(array_detected_depth_jumps)] == 1
                        index_new = (index - increment * j) % len(array_detected_depth_jumps)
                        break

                if index_new != None:
                    index = index_new % len(array_detected_depth_jumps)
                    self._check_move_merge_disappear(i, index)
   
    def _match_drift_while_still_stand(self, array_detected_depth_jumps):
        """
        Match the depth jumps from previous step with the current when the robot is not moving by its values but slowly drifting of.
        """
        for i in range(0, 360):
            if ((self.gnt_root.depth_jumps[i] != None and array_detected_depth_jumps[i] == 0)):                
                index_new = None
                increment = 0

                if array_detected_depth_jumps[i - 1] == 1:
                    index_new = i - 1
                    increment = -1
                elif array_detected_depth_jumps[(i + 1) % len(array_detected_depth_jumps)] == 1:
                    index_new = i + 1
                    increment = +1

                if index_new != None:
                    index = index_new % len(array_detected_depth_jumps)
                    
                    # if on new position already node then move this aswell
                    if self.gnt_root.depth_jumps[index] != None and self.gnt_root.depth_jumps[index].move_direction == self.gnt_root.depth_jumps[i].move_direction:
                        self.match_rotation(index_new, index_new + (abs(index_new - i) + 2) * increment, increment, array_detected_depth_jumps)

                    self._check_move_merge_disappear(i, index)

    def _match_forward_backwards(self, array_detected_depth_jumps, robot_move_forwards, robot_move_backwards):
        """
        Match the depth jumps from previous step with the current when the robot is moving forwards or backwards. 
        """
        if robot_move_forwards:
            self._match_forward(array_detected_depth_jumps)
        if robot_move_backwards:
            self._match_backwards(array_detected_depth_jumps)

    def _match_forward(self, array_detected_depth_jumps):
        """
        Match the depth jumps from previous step with the current when the robot is moving forwards. 
        """
        # 0 -> 179
        for i in range(0, len(array_detected_depth_jumps) / 2):
            index_new = None

            # move, merge, disappear
            if (self.gnt_root.depth_jumps[i] != None and array_detected_depth_jumps[i] == 0):
                index_new = self._search_x_degree_positiv(array_detected_depth_jumps, i, 3)
                if index_new == None:
                    index_new = self._search_x_degree_negativ(array_detected_depth_jumps, i, 1)
                self._check_move_merge_disappear(i, index_new)

            # appear or split
            if index_new == None and (self.gnt_root.depth_jumps[i] == None and array_detected_depth_jumps[i] == 1):
                # split: check if node near (with in 3 degrees)
                index_new = self._search_x_degree_positiv(array_detected_depth_jumps, i, 3)
                self._check_split_appear(i, index_new)

        # 359 -> 180
        for i in range(len(array_detected_depth_jumps) - 1, len(array_detected_depth_jumps) / 2, -1):
            index_new = None

            # move, merge, disappear
            if (self.gnt_root.depth_jumps[i] != None and array_detected_depth_jumps[i] == 0):
                index_new = self._search_x_degree_negativ(array_detected_depth_jumps, i, 3)
                if index_new == None:
                    index_new = self._search_x_degree_positiv(array_detected_depth_jumps, i, 1)
                self._check_move_merge_disappear(i, index_new)

            # appear or split
            if index_new == None and (self.gnt_root.depth_jumps[i] == None and array_detected_depth_jumps[i] == 1):
                # split: check if node near (with in 3 degrees)
                index_new = self._search_x_degree_negativ(array_detected_depth_jumps, i, 3)
                self._check_split_appear(i, index_new)

    def _match_backwards(self, array_detected_depth_jumps):
        """
        Match the depth jumps from previous step with the current when the robot is moving backwards. 
        """
        # 180 -> 0
        for i in range(len(array_detected_depth_jumps) / 2, -1, -1):
            index_new = None
            # move, merge, disappear
            if (self.gnt_root.depth_jumps[i] != None and array_detected_depth_jumps[i] == 0):
                index_new = self._search_x_degree_negativ(array_detected_depth_jumps, i, 3)
                if index_new == None:
                    index_new = self._search_x_degree_positiv(array_detected_depth_jumps, i, 1)
                self._check_move_merge_disappear(i, index_new)

            # appear or split
            if index_new == None and (self.gnt_root.depth_jumps[i] == None and array_detected_depth_jumps[i] == 1):
                # split: check if node near (with in 3 degrees)
                index_new = self._search_x_degree_negativ(array_detected_depth_jumps, i, 3)
                self._check_split_appear(i, index_new)

        # 180 -> 359
        for i in range(len(array_detected_depth_jumps) / 2, len(array_detected_depth_jumps)):
            index_new = None
            # move, merge, disappear
            if (self.gnt_root.depth_jumps[i] != None and array_detected_depth_jumps[i] == 0):
                index_new = self._search_x_degree_positiv(array_detected_depth_jumps, i, 3)
                if index_new == None:
                    index_new = self._search_x_degree_negativ(array_detected_depth_jumps, i, 1)
                self._check_move_merge_disappear(i, index_new)
            
            # appear or split
            if index_new == None and (self.gnt_root.depth_jumps[i] == None and array_detected_depth_jumps[i] == 1):
                # split: check if node near (with in 3 degrees)
                index_new = self._search_x_degree_positiv(array_detected_depth_jumps, i, 3)
                self._check_split_appear(i, index_new)
    
    def _search_x_degree_positiv(self, array_detected_depth_jumps, index, degree_search):
        """
        Find new position of depth jump searching in positiv direction (counter clock wise).
        """
        index_new = None

        for increment in range(1, degree_search + 1):
            if array_detected_depth_jumps[(index + increment)%len(self.gnt_root.depth_jumps)] == 1:
                index_new = (index + increment)%len(self.gnt_root.depth_jumps)
                break

        return index_new

    def _search_x_degree_negativ(self, array_detected_depth_jumps, index, degree_search):
        """
        Find new position of depth jump searching in negative direction (clock wise).
        """
        index_new = None

        for decrement in range(1, degree_search + 1):
            if array_detected_depth_jumps[index - decrement] == 1:
                index_new = index - decrement
                break

        return index_new

    def _check_split_appear(self, index_old, index_new):
        """
        Check if it is a split or appear.
        """
        if index_new != None:
            # split
            self._discontinuity_split(index_old, index_new)
        else:
            # appear
            self._discontinuity_appear(index_old)


    def _check_move_merge_disappear(self, index_old, index_new):
        """
        Check if it is a move, merge or disappear.
        """
        if index_new != None:
            if self.gnt_root.depth_jumps[index_new] == None:
                # move
                self._discontinuity_moved(index_old, index_new)
            else:
                # merge
                self._discontinuity_merge(index_old, index_new)
        else:
            # disappear
            self._discontinuity_disappear(index_old)

    def _discontinuity_split(self, index_old, index_new):
        """
        Handle split
        """
        split = CriticalEvent()
        split.event_type = CriticalEventEnum.SPLIT.value
        split.angle_old = index_old
        split.angle_new = index_new

        self.critical_events.events.append(split)

        self._publish_critical_event(split)

    def _discontinuity_appear(self, index):
        """
        Handle appear
        """
        appear = CriticalEvent()
        appear.event_type = CriticalEventEnum.APPEAR.value
        appear.angle_old = 0
        appear.angle_new = index_new

        self.critical_events.events.append(appear)
        self._publish_critical_event(appear)

    def _discontinuity_disappear(self, index):
        """
        Handle disappear
        """
        disappear = CriticalEvent()
        disappear.event_type = CriticalEventEnum.DISAPPEAR.value
        disappear.angle_old = 0
        disappear.angle_new = index

        self.critical_events.events.append(disappear)
        self._publish_critical_event(disappear)

    def _discontinuity_moved(self, index_old, index_new):
        """
        Handle move
        """
        move = GapMove()
        move.angle_old = index_old
        move.angle_new = index_new

        self.moved_gaps.gap_moves.append(move)
        self._publish_gap_move(move)

    def _discontinuity_merge(self, index_old, index_new):
        """
        Handle merge
        """
        merge = CriticalEvent()
        merge.event_type = CriticalEventEnum.MERGE.value
        merge.angle_old = index_old
        merge.angle_new = index_new

        self.critical_events.events.append(merge)
        self._publish_critical_event(merge)

    def _publish_critical_event(self, event):
        """
        Publish critical event
        """
        self.pub_critical_event.publish(event)

    def _publish_gap_move(self, gap_move):
        """
        Publish gap move
        """
        self.pub_gap_move.publish(gap_move)

if __name__ == "__main__":
    try:
        gp = GapSensor()
        gp.run()
    except Exception as ex:
        print(ex.message) 

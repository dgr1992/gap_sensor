import traceback
import rospy
import threading
import time
import numpy as np
import os

from depth_jump_sensor.msg import DepthJump
from gap_sensor.msg import CriticalEvent, CriticalEvents, MovedGaps, GapMove

from critical_event_enum import CriticalEventEnum
from gap_visualiser import GapVisualiser

class GapSensor:
    """
    - Determins which of the given depth jumps from the topic depth jumps is a valid gap.
      A valid gap needs to be wide enough for the robot to fit through. Node keeps track of all
      but marks non valid ones so .
    - Recignises appear, disappear, split, merge and publish
    """

    def __init__(self):
        rospy.init_node("gap_sensor")
        
        self.depth_jumps_last = None
        self.scan_last = None
        self.rotation_last = None
        self.movement_last = None

        self.update_frequence = 90

        self.critical_events = CriticalEvents()
        self.moved_gaps = MovedGaps()

        self.debug_to_file = False
        self.file_depth_jumps_receive = "depth_jumps_receive.csv"
        self.file_depth_jumps = "depth_jumps.csv"
        self._remove_debug_files()

        self._init_publisher()
        self._init_subscribers()

        self.update_gap_view = False

        self.lock = threading.Lock()

    def _init_subscribers(self):
        """
        Initialise subscribers
        """
        self._sub_depth_jumps = rospy.Subscriber("depth_jumps", DepthJump, self._receive_depth_jumps, queue_size=15)

    def _init_publisher(self):
        """
        Initialise publisher
        """
        self.pub_critical_events = rospy.Publisher("critical_events", CriticalEvents, queue_size=1)
        self.pub_critical_event = rospy.Publisher("critical_event", CriticalEvent, queue_size=1)
        
        self.pub_moved_gaps = rospy.Publisher("moved_gaps", MovedGaps, queue_size=1)
        self.pub_gap_move = rospy.Publisher("gap_move", GapMove, queue_size=1)

    def _receive_depth_jumps(self, data):
        """
        Receive depth jump data
        """
        if self.rotation_last == None:
            # initialise
            self.rotation_last = 0
            self.depth_jumps_last = np.zeros(len(data.depth_jumps))
            self.scan_last = np.zeros(len(data.depth_jumps))
            self.movement_last = 0
        
        self._process(data)

    def _remove_debug_files(self):
        if os.path.exists(self.file_depth_jumps_receive):
            os.remove(self.file_depth_jumps_receive)

        if os.path.exists(self.file_depth_jumps):
            os.remove(self.file_depth_jumps)

    def run(self):
        angle_change = 0
        last_time = 0

        gap_visualisation_gnt = GapVisualiser('Gap Sensor')

        while not rospy.is_shutdown():

            if self.rotation_last != None and (time.time() - last_time) > (1.0/self.update_frequence) and self.update_gap_view:
                self.update_gap_view = False
                last_time = time.time()
                gap_visualisation_gnt.draw_gaps(self.depth_jumps_last)

        gap_visualisation_gnt.close()

    def _process(self, depth_jump_data):
        self.lock.acquire()
        try:
            start = time.time()
            # initialise msgs empty
            self.critical_events = CriticalEvents()
            self.critical_events.events = []
            self.moved_gaps = MovedGaps()
            self.moved_gaps.gap_moves = []

            # get data from msg
            depth_jumps = np.asarray(depth_jump_data.depth_jumps)
            scan = np.asarray(depth_jump_data.range_data)
            rotation = depth_jump_data.rotation
            movement = depth_jump_data.liniear_x

            if self.debug_to_file:
                with open(self.file_depth_jumps_receive,'ab') as f4:
                    tmp = np.asarray(depth_jumps)
                    tmp = np.insert(tmp,0,rotation)
                    np.savetxt(f4, tmp.reshape(1, tmp.shape[0]), delimiter=",")

            # TODO Filter depth jumps to get the valid gaps. Depth jump is a gap if the robot fits throuh it.

            self._detect_critical_events(self.depth_jumps_last, self.rotation_last, self.movement_last, depth_jumps, rotation, movement)

            if len(self.critical_events.events) > 0:
                self.pub_critical_events.publish(self.critical_events)

            if len(self.moved_gaps.gap_moves) > 0:
                self.pub_moved_gaps.publish(self.moved_gaps)

            self.rotation_last = rotation
            self.scan_last = scan
            self.movement_last = movement

            if self.debug_to_file:
                with open(self.file_depth_jumps,'ab') as f4:
                    tmp = np.asarray(self.depth_jumps_last)
                    tmp = np.insert(tmp,0,rotation)
                    np.savetxt(f4, tmp.reshape(1, tmp.shape[0]), delimiter=",")

            self.update_gap_view = True

            end = time.time()
            process_time = (end - start) * 1000
            print("Time to process scan: " + str(process_time) + " ms")
        except Exception as ex:
           print(ex.message)

        self.lock.release()

    def _detect_critical_events(self, depth_jumps_last, rotation_last, movement_last, depth_jumps, rotation, movement):
        """
        Detection of the critical events appear, disappear, split, merge. Further more, notice where a depth jump has moved.
        """
        # rotation
        if rotation < 0:
            self.match_rotation(0, len(depth_jumps_last), 1, depth_jumps_last, depth_jumps)
        elif rotation > 0:    
            self.match_rotation(len(depth_jumps_last) - 1, -1, -1, depth_jumps_last, depth_jumps)
        #elif movement == 0:
        #    self._match_drift_while_still_stand(depth_jumps_last, depth_jumps)

        # forwards backwards  
        if movement != 0:
            self._match_forward_backwards(depth_jumps_last, depth_jumps, movement)

    def match_rotation(self, start_index, end_index, increment, depth_jumps_last, depth_jumps):
        """
        Match the depth jumps from previous step with the current.
        """
        for index in range(start_index, end_index, increment):
            index_new = None

            # when at t-1 a depth jump was detected at this position, then try to find the new position of it
            if depth_jumps_last[index % len(depth_jumps_last)] != 0:
                if depth_jumps[index % len(depth_jumps)] == 0:
                    # search in direction
                    #for j in range(0,4):
                    #    if depth_jumps[(index + increment * j) % len(depth_jumps)] != 0:
                    #        index_new = (index + increment * j) % len(depth_jumps)
                    #        break
                    
                    # corresponding position might be in the opposite direction
                    #if index_new == None: # and depth_jumps[(index - increment) % len(depth_jumps)] > 0:
                        #index_new = (index - increment) % len(depth_jumps)
                    #    for j in range(1,3):
                    #        if depth_jumps[(index - increment * j) % len(depth_jumps)] != 0:
                    #            index_new = (index - increment * j) % len(depth_jumps)
                    #            break
                    index_new = self._find_new_pos_of_depth_jump(depth_jumps, index, increment)
                    self._check_move_merge_disappear(depth_jumps_last, index, index_new)    
                else:
                    depth_jumps[index] = 0
                    self.depth_jumps_last[index] = 1
            """
            else:
                # It is a new depth jump when at t-1 no detection and t is a detection
                last_no_detection = depth_jumps_last[i - 1] == 0 and depth_jumps_last[i % len(depth_jumps_last)] == 0 and depth_jumps_last[(i + 1) % len(depth_jumps_last)] == 0

                # When at the start index the detected depth jump has moved from 259 -> 0 or 0 -> 259
                if depth_jumps[i % len(depth_jumps)] == 1 and last_no_detection and index != start_index:
                    # appear
                    self._discontinuity_appear(i)
                    depth_jumps[i] = 0
            """
        # TODO make this more efficient
        # depth_jumps now contains all new depth jumps
        for index in range(start_index, end_index, increment):
            if depth_jumps[index % len(depth_jumps)] == 1:
                index_old = self._find_new_pos_of_depth_jump(depth_jumps_last, index, increment)
                if index_old != None:
                    #move
                    self._discontinuity_moved(index_old, index)
                else:
                    # appear
                    self._discontinuity_appear(index)
                    depth_jumps[index] = 0

    def _find_new_pos_of_depth_jump(self, depth_jumps, index, increment):
        index_next = None
        # search in direction
        for j in range(0,4):
            if depth_jumps[(index + increment * j) % len(depth_jumps)] != 0:
                index_next = (index + increment * j) % len(depth_jumps)
                break
        
        # corresponding position might be in the opposite direction
        if index_next == None: # and depth_jumps[(index - increment) % len(depth_jumps)] > 0:
            #index_new = (index - increment) % len(depth_jumps)
            for j in range(1,3):
                if depth_jumps[(index - increment * j) % len(depth_jumps)] != 0:
                    index_next = (index - increment * j) % len(depth_jumps)
                    break
        
        return index_next

    def _match_drift_while_still_stand(self, depth_jumps_last, depth_jumps):
        """
        Match the depth jumps from previous step with the current when the robot is not moving by its values but slowly drifting of.
        """
        for i in range(0, 360):
            if (depth_jumps_last[i] != 0 and depth_jumps[i] == 0):
                # check existing                
                index_new = None

                if depth_jumps[i - 1] == 1:
                    index_new = i - 1
                elif depth_jumps[(i + 1) % len(depth_jumps)] == 1:
                    index_new = (i + 1) % len(depth_jumps)

                self._check_move_merge_disappear(depth_jumps_last, i, index_new)
                
                if index_new != None:                    
                    depth_jumps[index_new] = 0
            elif (depth_jumps_last[i % len(depth_jumps_last)] == 0 and depth_jumps[i % len(depth_jumps)] == 1 and depth_jumps_last[i - 1] == 0 and depth_jumps_last[(i + 1) % len(depth_jumps_last)] == 0):
                # appear
                self._discontinuity_appear(i)
                depth_jumps[i] = 0

    def _match_forward_backwards(self, depth_jumps_last, depth_jumps, movement):
        """
        Match the depth jumps from previous step with the current when the robot is moving forwards or backwards. 
        """
        if movement > 0:
            self._match_forward(depth_jumps_last, depth_jumps)
        if movement < 0:
            self._match_backwards(depth_jumps_last, depth_jumps)

    def _match_forward(self, depth_jumps_last, depth_jumps):
        """
        Match the depth jumps from previous step with the current when the robot is moving forwards. 
        """
        # 0 -> 179
        for i in range(0, len(depth_jumps) / 2):
            index_new = None

            # move, merge, disappear
            if (depth_jumps_last[i] != 0 and depth_jumps[i] == 0):
                index_new = self._search_x_degree_positiv(depth_jumps, i, 4)
                if index_new == None:
                    index_new = self._search_x_degree_negativ(depth_jumps, i, 1)
                self._check_move_merge_disappear(depth_jumps_last, i, index_new)

            # appear or split
            if index_new == None and (depth_jumps_last[i] == 0 and depth_jumps[i] == 1):
                # split: check if node near (with in 3 degrees)
                index_new = self._search_x_degree_positiv(depth_jumps, i, 4)
                self._check_split_appear(i, index_new)

        # 359 -> 180
        for i in range(len(depth_jumps) - 1, len(depth_jumps) / 2, -1):
            index_new = None

            # move, merge, disappear
            if (depth_jumps_last[i] != 0 and depth_jumps[i] == 0):
                index_new = self._search_x_degree_negativ(depth_jumps, i, 4)
                if index_new == None:
                    index_new = self._search_x_degree_positiv(depth_jumps, i, 1)
                self._check_move_merge_disappear(depth_jumps_last, i, index_new)

            # appear or split
            if index_new == None and (depth_jumps_last[i] == 0 and depth_jumps[i] == 1):
                # split: check if node near (with in 3 degrees)
                index_new = self._search_x_degree_negativ(depth_jumps, i, 4)
                self._check_split_appear(i, index_new)

    def _match_backwards(self, depth_jumps_last, depth_jumps):
        """
        Match the depth jumps from previous step with the current when the robot is moving backwards. 
        """
        # 180 -> 0
        for i in range(len(depth_jumps) / 2, -1, -1):
            index_new = None
            # move, merge, disappear
            if (depth_jumps_last[i] != 0 and depth_jumps[i] == 0):
                index_new = self._search_x_degree_negativ(depth_jumps, i, 4)
                if index_new == None:
                    index_new = self._search_x_degree_positiv(depth_jumps, i, 1)
                self._check_move_merge_disappear(depth_jumps_last, i, index_new)

            # appear or split
            if index_new == None and (depth_jumps_last[i] == 0 and depth_jumps[i] == 1):
                # split: check if node near (with in 3 degrees)
                index_new = self._search_x_degree_negativ(depth_jumps, i, 4)
                self._check_split_appear(i, index_new)

        # 180 -> 359
        for i in range(len(depth_jumps) / 2, len(depth_jumps)):
            index_new = None
            # move, merge, disappear
            if (depth_jumps_last[i] != 0 and depth_jumps[i] == 0):
                index_new = self._search_x_degree_positiv(depth_jumps, i, 4)
                if index_new == None:
                    index_new = self._search_x_degree_negativ(depth_jumps, i, 1)
                self._check_move_merge_disappear(depth_jumps_last, i, index_new)
            
            # appear or split
            if index_new == None and (depth_jumps_last[i] == 0 and depth_jumps[i] == 1):
                # split: check if node near (with in 3 degrees)
                index_new = self._search_x_degree_positiv(depth_jumps, i, 4)
                self._check_split_appear(i, index_new)
    
    def _search_x_degree_positiv(self, depth_jumps, index, degree_search):
        """
        Find new position of depth jump searching in positiv direction (counter clock wise).
        """
        index_new = None

        for increment in range(1, degree_search + 1):
            if depth_jumps[(index + increment)%len(depth_jumps)] == 1:
                index_new = (index + increment)%len(depth_jumps)
                break

        return index_new

    def _search_x_degree_negativ(self, depth_jumps, index, degree_search):
        """
        Find new position of depth jump searching in negative direction (clock wise).
        """
        index_new = None

        for decrement in range(1, degree_search + 1):
            if depth_jumps[index - decrement] == 1:
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

    def _check_move_merge_disappear(self, depth_jumps_last, index_old, index_new):
        """
        Check if it is a move, merge or disappear.
        """
        if index_new != None:
            if depth_jumps_last[index_new] == 0:
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
        self.depth_jumps_last[index_old] = 3
        self.depth_jumps_last[index_new] = 3

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

        self.depth_jumps_last[index] = 2

        appear = CriticalEvent()
        appear.event_type = CriticalEventEnum.APPEAR.value
        appear.angle_old = 0
        appear.angle_new = index

        self.critical_events.events.append(appear)
        self._publish_critical_event(appear)

    def _discontinuity_disappear(self, index):
        """
        Handle disappear
        """
        self.depth_jumps_last[index] = 0
        
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
        self.depth_jumps_last[index_old] = 0
        self.depth_jumps_last[index_new] = 1

        move = GapMove()
        move.angle_old = index_old
        move.angle_new = index_new

        self.moved_gaps.gap_moves.append(move)
        self._publish_gap_move(move)

    def _discontinuity_merge(self, index_old, index_new):
        """
        Handle merge
        """
        self.depth_jumps_last[index_old] = 0
        self.depth_jumps_last[index_new] = 4

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
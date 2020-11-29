import traceback
import rospy
import threading
import time
import numpy as np
import os
import copy

from depth_jump_sensor.msg import DepthJump
from gap_sensor.msg import CriticalEvent, CriticalEvents, MovedGaps, GapMove, CollectionCriticalAndMoved

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
        rospy.init_node("gap_sensor", log_level=rospy.INFO)
        
        self.current_sequence_id = 0
        self.depth_jumps_last = None
        self.scan_last = None
        self.rotation_last = None
        self.movement_last = None

        self.update_frequence = 90

        self.critical_events = CriticalEvents()
        self.moved_gaps = MovedGaps()

        self.debug_to_file = True
        self.file_depth_jumps_receive = "depth_jumps_receive.csv"
        self.file_depth_jumps = "depth_jumps.csv"
        self._remove_debug_files()

        self._init_publisher()
        self._init_subscribers()

        self.update_gap_view = False
        
        self.depth_jumps_received_history = []
        self.depth_jumps_history = []

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

        self.pub_collection = rospy.Publisher("collection_critical_and_moved", CollectionCriticalAndMoved, queue_size=1)

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

        self._save_depth_jumps_history()
        self._save_depth_jumps_received_history()

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
            self.current_sequence_id = depth_jump_data.header.seq

            if self.debug_to_file:
                with open(self.file_depth_jumps_receive,'ab') as f4:
                    tmp = np.asarray(depth_jumps)
                    tmp = np.insert(tmp,0,rotation)
                    tmp = np.insert(tmp,0,movement)
                    tmp = np.insert(tmp,0,self.current_sequence_id)
                    self.depth_jumps_received_history.append(tmp)

            # TODO Filter depth jumps to get the valid gaps. Depth jump is a gap if the robot fits throuh it.

            self._detect_critical_events(depth_jumps, rotation, movement)

            if len(self.critical_events.events) > 0:
                self.pub_critical_events.publish(self.critical_events)

            if len(self.moved_gaps.gap_moves) > 0:
                self.pub_moved_gaps.publish(self.moved_gaps)

            if len(self.moved_gaps.gap_moves) > 0 or len(self.critical_events.events) > 0:
                self._publish_collected(self.moved_gaps, self.critical_events)

            self.rotation_last = rotation
            self.scan_last = scan
            self.movement_last = movement

            if self.debug_to_file:
                with open(self.file_depth_jumps,'ab') as f4:
                    tmp = np.asarray(self.depth_jumps_last)
                    tmp = np.insert(tmp,0,rotation)
                    tmp = np.insert(tmp,0,movement)
                    tmp = np.insert(tmp,0,self.current_sequence_id)
                    self.depth_jumps_history.append(tmp)

            self.update_gap_view = True

            end = time.time()
            process_time = (end - start) * 1000
            #print("Time to process scan: " + str(process_time) + " ms")
        except Exception as ex:
           print(ex.message)

        self.lock.release()

    def _detect_critical_events(self, depth_jumps, rotation, movement):
        """
        Detection of the critical events appear, disappear, split, merge. Further more, notice where a depth jump has moved.
        
        depth_jumps (int[]): discontinuity reading
        rotation (int): rotation direction of robot which caused reading depth_jumps
        movement (int): movement direction of robot which caused reading depth_jumps
        """

        #rotation
        if rotation != 0:
            self._match_rotation_left_right(self.depth_jumps_last, depth_jumps, rotation)
       
        # forwards backwards  
        if movement != 0:
            self._match_forward_backwards(self.depth_jumps_last, depth_jumps, movement)

        if movement == 0 and rotation == 0:
            self._match_drift_while_still_stand(self.depth_jumps_last, depth_jumps)

    def _match_rotation_left_right(self, depth_jumps_last, depth_jumps, rotation):
        """
        Match the depth jumps from previous step with the current when the robot is rotating. 
        
        Parameters:
        depth_jumps_last (int[]): Array indicating the depth jumps at t - 1
        depth_jumps (int[]): Array indicating the depth jumps at t
        rotation (int): counter-clockwise = 1, clockwise = -1
        """
        # rotation
        if rotation < 0:
            self._match_rotation(0, len(depth_jumps_last), 1, depth_jumps_last, depth_jumps)
        elif rotation > 0:    
            self._match_rotation(len(depth_jumps_last) - 1, -1, -1, depth_jumps_last, depth_jumps)


    def _match_rotation(self, start_index, end_index, increment, depth_jumps_last, depth_jumps):
        """
        Match the depth jumps from previous step with the current.
        """
        
        depth_jumps_cp = copy.copy(depth_jumps)
        for index in range(start_index, end_index, increment):
            index_new = None

            # when at t-1 a depth jump was detected at this position, then try to find the new position of it
            if depth_jumps_last[index % len(depth_jumps_last)] == 1:
                if depth_jumps_cp[index % len(depth_jumps_cp)] == 0:
                    index_new = self._find_new_pos_of_depth_jump(depth_jumps_cp, index, increment)
                    self._check_move_merge_disappear(depth_jumps_last, index, index_new, increment)    
                else:
                    # positions match, set copy to 0 so it can not get match with an other disconinutiy
                    depth_jumps_cp[index] = 0
                    self.depth_jumps_last[index] = 1

        # depth_jumps now contains all new depth jumps
        for index in range(start_index, end_index, increment):
            if depth_jumps_cp[index % len(depth_jumps_cp)] == 1:
                if depth_jumps_last[index % len(depth_jumps_last)] == 0:
                    index_old = self._find_new_pos_of_depth_jump(depth_jumps_last, index, increment)
                    if index_old != None:
                        #move
                        self._discontinuity_moved(index_old, index)
                    else:
                        # appear
                        self._discontinuity_appear(index)
                # positions matched at this point, set copy to 0 so it can not get match with an other disconinutiy
                depth_jumps_cp[index] = 0
                

    def _find_new_pos_of_depth_jump(self, depth_jumps, index, increment):
        """
        Search for the next depth jump starting at the given index and in the given direction.

        Parameters:
        depth_jumps (int[]): Array indicating the depth jumps
        index (int): start index for searching
        increment (int): direction to search
        """
        index_next = None
        # search in direction
        for j in range(0,5):
            if depth_jumps[(index + increment * j) % len(depth_jumps)] != 0:
                index_next = (index + increment * j) % len(depth_jumps)
                break
        
        # corresponding position might be in the opposite direction
        if index_next == None: # and depth_jumps[(index - increment) % len(depth_jumps)] > 0:
            #index_new = (index - increment) % len(depth_jumps)
            for j in range(1,4):
                if depth_jumps[(index - increment * j) % len(depth_jumps)] != 0:
                    index_next = (index - increment * j) % len(depth_jumps)
                    break
        
        return index_next

    def _match_drift_while_still_stand(self, depth_jumps_last, depth_jumps):
        """
        Match the depth jumps from previous step with the current when the robot is not moving by its values but slowly drifting of.
        
        Parameters:
        depth_jumps_last (int[]): Array indicating the depth jumps at t - 1
        depth_jumps (int[]): Array indicating the depth jumps at t
        """
        depth_jumps_cp = copy.copy(depth_jumps)
        for index in range(0, 360):
            if (depth_jumps_last[index] != 0 and depth_jumps_cp[index] == 0):
                # check existing                
                index_new = None

                if depth_jumps_cp[index - 1] == 1:
                    index_new = index - 1
                    #rospy.logdebug("drift while still stand (check) - seq: " + str(self.current_sequence_id) + " - index: " + str(index) + " - index_new: " + str(index_new))
                elif depth_jumps_cp[(index + 1) % len(depth_jumps_cp)] == 1:
                    index_new = (index + 1) % len(depth_jumps_cp)
                    #rospy.logdebug("drift while still stand (check) - seq: " + str(self.current_sequence_id) + " - index: " + str(index) + " - index_new: " + str(index_new))

                rospy.logdebug("drift while still stand  - seq: " + str(self.current_sequence_id) + " - index: " + str(index) + " - index_new: " + str(index_new))
                self._check_move_merge_disappear(depth_jumps_last, index, index_new, +1)
                
                #if index_new != None:                    
                #    depth_jumps_cp[index_new] = 0
            elif (depth_jumps_last[index % len(depth_jumps_last)] == 0 and depth_jumps_cp[index % len(depth_jumps_cp)] == 1 and depth_jumps_last[index - 1] == 0 and depth_jumps_last[(index + 1) % len(depth_jumps_last)] == 0):
                # appear
                self._discontinuity_appear(index)
                depth_jumps_cp[index] = 0
            elif (depth_jumps_last[index] > 1 and depth_jumps_cp[index] == 1):
                # set back to 1 after appear
                self.depth_jumps_last[index] = 1

    def _match_forward_backwards(self, depth_jumps_last, depth_jumps, movement):
        """
        Match the depth jumps from previous step with the current when the robot is moving forwards or backwards. 
        
        Parameters:
        depth_jumps_last (int[]): Array indicating the depth jumps at t - 1
        depth_jumps (int[]): Array indicating the depth jumps at t
        movement (int): forward = 1, backwards = -1
        """
        if movement > 0:
            self._match_forward(depth_jumps_last, depth_jumps)
        if movement < 0:
            self._match_backwards(depth_jumps_last, depth_jumps)

    def _match_forward(self, depth_jumps_last, depth_jumps):
        """
        Match the depth jumps from previous step with the current when the robot is moving forwards. 
        
        Parameters:
        depth_jumps_last (int[]): Array indicating the depth jumps at t - 1
        depth_jumps (int[]): Array indicating the depth jumps at t
        """
        depth_jumps_cp = copy.copy(depth_jumps)
        # 0 -> 179
        index = 0
        while index < (len(depth_jumps_cp) / 2):
            index = self._check_positive_direction(depth_jumps_last, depth_jumps_cp, index)
            index += 1

        # 359 -> 180
        index = len(depth_jumps_cp) - 1
        while index >= (len(depth_jumps_cp) / 2):
            index = self._check_negative_direction(depth_jumps_last, depth_jumps_cp, index)
            index -= 1

    def _match_backwards(self, depth_jumps_last, depth_jumps):
        """
        Match the depth jumps from previous step with the current when the robot is moving backwards. 
        
        Parameters:
        depth_jumps_last (int[]): Array indicating the depth jumps at t - 1
        depth_jumps_cp (int[]): Array indicating the depth jumps at t
        """
        depth_jumps_cp = copy.copy(depth_jumps)
        # 179 -> 0
        index = (len(depth_jumps_cp) / 2) - 1
        while index >= 0:
            index = self._check_negative_direction(depth_jumps_last, depth_jumps_cp, index)
            index -= 1

        # 180 -> 359
        index = len(depth_jumps_cp) / 2
        while index < len(depth_jumps_cp):
            index = self._check_positive_direction(depth_jumps_last, depth_jumps_cp, index)
            index += 1

    def _check_positive_direction(self, depth_jumps_last, depth_jumps_cp, index):
        """
        """
        index_new = None
        # move, merge, disappear
        if (depth_jumps_last[index] == 1 and depth_jumps_cp[index] == 0):
            index_new = self._search_x_degree_positiv(depth_jumps_cp, index, 5)
            rospy.logdebug("match backwards - 180 -> 359 - search positive - index_new: " + str(index_new))
            if index_new == None:
                index_new = self._search_x_degree_negativ(depth_jumps_cp, index, 3)
                rospy.logdebug("match backwards - 180 -> 359 - search negative - index_new: " + str(index_new))
            self._check_move_merge_disappear(depth_jumps_last, index, index_new, 1)
        
        if index_new == None:
            if depth_jumps_last[index] == 0 and depth_jumps_cp[index] == 1:
                # appear or split: try find node near (with in 3 degrees)
                index_new_1, index_new_2 = self._check_split_appear(depth_jumps_last, depth_jumps_cp, index, +1)
                if index_new_2 != None:
                    index = index_new_2
                else:
                    index = index_new_1
            elif depth_jumps_last[index] == 1 and depth_jumps_cp[index] == 1:
                # gap stayed at the same index, set to 1
                depth_jumps_last[index] = 1
                # depth jump got processed at this point
                depth_jumps_cp[index] = 0
        else:
            # depth jump got processed at this point
            depth_jumps_cp[index] = 0
        return index
            

    def _check_negative_direction(self, depth_jumps_last, depth_jumps_cp, index):
        """
        """
        index_new = None
        # move, merge, disappear
        if (depth_jumps_last[index] == 1 and depth_jumps_cp[index] == 0):
            index_new = self._search_x_degree_negativ(depth_jumps_cp, index, 5)
            if index_new == None:
                index_new = self._search_x_degree_positiv(depth_jumps_cp, index, 3)
            self._check_move_merge_disappear(depth_jumps_last, index, index_new, -1) 
        
        if index_new == None:
            if depth_jumps_last[index] == 0 and depth_jumps_cp[index] == 1:
                # appear or split: try find node near (with in 3 degrees)
                index_new_1, index_new_2 = self._check_split_appear(depth_jumps_last, depth_jumps_cp, index, -1)
                if index_new_2 != None:
                    index = index_new_2
                else:
                    index = index_new_1
            elif depth_jumps_last[index] != 1 and depth_jumps_cp[index] == 1:
                # gap stayed at the same index, set to 1
                depth_jumps_last[index] = 1
                # depth jump got processed at this point
                depth_jumps_cp[index] = 0
        else:
            # depth jump got processed at this point
            depth_jumps_cp[index] = 0
        return index

    def _search_x_degree_positiv(self, depth_jumps, index, degree_search):
        """
        Find new position of depth jump searching in positiv direction (counter clock wise).
        
        Parameters:
        depth_jumps_last (int[]): Array indicating the depth jumps at t - 1
        index (int): start index
        degree_search (int): number of neigbour elements to check
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

        Parameters:
        depth_jumps_last (int[]): Array indicating the depth jumps at t - 1
        index (int): start index
        degree_search (int): number of neigbour elements to check
        """
        index_new = None

        for decrement in range(1, degree_search + 1):
            if depth_jumps[index - decrement] == 1:
                index_new = index - decrement
                break

        return index_new

    def _check_split_appear(self, depth_jumps_last, depth_jumps, index_new_1, search_increment):
        """
        Check if it is a split or appear.

        Parameters: \n
        index_old (int): index that splitted\n
        index_new_1 (int): index of detection at t\n

        Returns:\n
        index_new_1 (int):\n
        index_new_1 (int):\n
        """
        index_old = None
        index_new_2 = None

        # try to find index that splitted
        if search_increment > 0:
            index_old = self._search_x_degree_positiv(depth_jumps_last, index_new_1, 2)
        else:
            index_old = self._search_x_degree_negativ(depth_jumps_last, index_new_1, 2)


        if index_old != None:
            index_new_2 = self._find_second_depth_jump_from_split(depth_jumps_last, depth_jumps, index_new_1, index_old)
        
        if index_old != None and index_new_2 != None:
            # split
            self._discontinuity_split(index_old, index_new_1, index_new_2)

            depth_jumps_last[index_old] = 0
            # depth jump got processed at this point
            depth_jumps[index_new_1] = 0
            depth_jumps[index_new_2] = 0
        else:
            # appear
            self._discontinuity_appear(index_new_1)
            depth_jumps[index_new_1] = 0

        return index_new_1, index_new_2

    def _check_move_merge_disappear(self, depth_jumps_last, index_old_1, index_new, search_increment):
        """
        Check if it is a move, merge or disappear.
        """
        if index_new != None:
            
            index_old_2 = self._check_merge(depth_jumps_last, index_new, index_old_1, search_increment)

            if index_old_2 == None:
                # move
                self._discontinuity_moved(index_old_1, index_new)
                rospy.logdebug("move - seq: " + str(self.current_sequence_id) + " - index_old: " + str(index_old_1) + " index_new: " + str(index_new))
            else:
                #rospy.logdebug("merge - seq: " + str(self.current_sequence_id) + " - index_old: " + str(index_old) + " index_new: " + str(index_new) + "; depth_jumps_last[index_new]=" + str(depth_jumps_last[index_new]))
                # merge
                self._discontinuity_merge(index_old_1, index_old_2, index_new)
                #rospy.logdebug("merge - seq: " + str(self.current_sequence_id) + " - index_old: " + str(index_old) + " index_new: " + str(index_new) + "; depth_jumps_last[index_new]=" + str(depth_jumps_last[index_new]))
                rospy.logdebug("merge - seq: " + str(self.current_sequence_id) + " - index_old_1: " + str(index_old_1) + " - index_old_2: " + str(index_old_2) + " index_new: " + str(index_new))
        else:
            # disappear
            self._discontinuity_disappear(index_old_1)

    def _find_second_depth_jump_from_split(self, depth_jumps_last, depth_jumps, index_new_1, index_old):
        """
        Check if the new index is from a split.

        Parameters:
        depth_jumps_last (int()): depth jumps at t - 1
        depth_jumps (int[]): depth jumps at t
        index_new_1 (int): index where a depth jump appeard from a split
        index_old (int): index of depth jump that splitted

        Returns:
        index_new_2 (int): index of the second gap from the split
        """
        index_new_2 = None

        # Find a second depth jump that appeared behinde the depth jump at index_old
        split_detect = False
        split_detect = split_detect or (depth_jumps[(index_new_1 + 2) % len(depth_jumps)] == 1)
        split_detect = split_detect or (depth_jumps[(index_new_1 - 2) % len(depth_jumps)] == 1)

        # check for no depth jump with in 2 degrees left right at t - 1
        if split_detect:
            no_depth_jump_in_range = True
            for j in range(1,4):
                no_depth_jump_in_range = no_depth_jump_in_range and (depth_jumps_last[(index_old + j) % len(depth_jumps_last)] == 0)
                no_depth_jump_in_range = no_depth_jump_in_range and (depth_jumps_last[(index_old - j) % len(depth_jumps_last)] == 0)

            if no_depth_jump_in_range:
                # find the second index of the split
                if depth_jumps[(index_new_1 + 2) % len(depth_jumps)] == 1:
                    index_new_2 = (index_new_1 + 2) % len(depth_jumps)
                elif depth_jumps[(index_new_1 - 2) % len(depth_jumps)] == 1:
                    index_new_2 = (index_new_1 - 2) % len(depth_jumps)

        return index_new_2

    def _check_merge(self, depth_jumps_last, index_new, index_old_1, search_increment):
        """
        Check if new index is from a merge of two depth jumps.

        Parameters:
        depth_jumps_last (int[]): depth jumps at t - 1
        index_new_1 (int): index of depth jump that appeard from merge
        index_old_1 (int): first old depth jump that might have caused a merge
        search_increment (int): the search increment used to find index_old_1

        Return:
        index_old_2 (int): second depth jump that merged
        """

        index_old_2 = None

        # search for second depth jump in oposite direction
        for j in range(1, 4):
            if index_old_2 == None and depth_jumps_last[(index_old_1 + search_increment * j) % len(depth_jumps_last)] > 0:
                index_old_2 = (index_old_1 + search_increment * j) % len(depth_jumps_last)

        # if a depth jump was found check if the distance between the two depth jumps is 2
        if index_old_2 != None:
            diff = abs(index_old_1 - index_old_2)
            if diff > 10:
                diff = 360 - diff
            if diff != 2:
                index_old_2 = None

        return index_old_2
        
    def _discontinuity_split(self, index_old_1, index_new_1, index_new_2):
        """
        Handle split

        Parameters:\n
        index_new_1 (int): gap at t-1 that splitted, first gap from split\n
        index_2 (int): second gap from split\n
        """
        rospy.logdebug("split - seq: " + str(self.current_sequence_id) + " - index_new_1: " + str(index_new_1) + " index_new_2: " + str(index_new_2))

        self.depth_jumps_last[index_old_1] = 0
        self.depth_jumps_last[index_new_1] = 3
        self.depth_jumps_last[index_new_2] = 3

        split = CriticalEvent()
        split.event_type = CriticalEventEnum.SPLIT.value
        split.angle_old_1 = index_old_1
        split.angle_new_1 = index_new_1
        split.angle_new_2 = index_new_2

        self.critical_events.events.append(split)

        self._publish_critical_event(split)

    def _discontinuity_appear(self, index):
        """
        Handle appear

        Parameters:
        index (int): index where the gap appeared
        """
        rospy.logdebug("appear - seq: " + str(self.current_sequence_id) + " - index: " + str(index))

        self.depth_jumps_last[index] = 2

        appear = CriticalEvent()
        appear.event_type = CriticalEventEnum.APPEAR.value
        appear.angle_new_1 = index

        self.critical_events.events.append(appear)
        self._publish_critical_event(appear)

    def _discontinuity_disappear(self, index):
        """
        Handle disappear

        Parameters:
        index (int): index where the gap disappeared
        """
        rospy.logdebug("disappear - seq: " + str(self.current_sequence_id) + " index:" + str(index))
        self.depth_jumps_last[index] = 0
        
        disappear = CriticalEvent()
        disappear.event_type = CriticalEventEnum.DISAPPEAR.value
        disappear.angle_old_1 = index

        self.critical_events.events.append(disappear)
        self._publish_critical_event(disappear)

    def _discontinuity_moved(self, index_old, index_new):
        """
        Handle move

        Parameters:
        index_old (index): index of gap at t-1
        index_new (index): index of gap at t
        """
        self.depth_jumps_last[index_old] = 0
        self.depth_jumps_last[index_new] = 1

        move = GapMove()
        move.angle_old = index_old
        move.angle_new = index_new

        self.moved_gaps.gap_moves.append(move)
        self._publish_gap_move(move)

    def _discontinuity_merge(self, index_old_1, index_old_2, index_new):
        """
        Handle merge

        Parameters:
        index_old_1 (int): index of gap 1 at t-1
        index_old_2 (int): index of gap 2 at t-1
        index_new (int): index of gap resulted from merge
        """
        self.depth_jumps_last[index_old_1] = 0
        self.depth_jumps_last[index_old_2] = 0
        self.depth_jumps_last[index_new] = 4

        merge = CriticalEvent()
        merge.event_type = CriticalEventEnum.MERGE.value
        merge.angle_old_1 = index_old_1
        merge.angle_old_2 = index_old_2
        merge.angle_new_1 = index_new

        self.critical_events.events.append(merge)
        self._publish_critical_event(merge)

    def _publish_critical_event(self, event):
        """
        Publish critical event

        Parameters:
        event (CriticalEvent):
        """
        self.pub_critical_event.publish(event)

    def _publish_gap_move(self, gap_move):
        """
        Publish gap move

        Parameters:
        gap_move (GapMover):
        """
        self.pub_gap_move.publish(gap_move)

    def _publish_collected(self, gaps_moved, critical_events):
        collection_critical_and_moved = CollectionCriticalAndMoved()
        collection_critical_and_moved.events = critical_events
        collection_critical_and_moved.gap_moves = gaps_moved

        self.pub_collection.publish(collection_critical_and_moved)

    def _save_depth_jumps_received_history(self):
        """
        Save depth jumps received history.
        """
        if self.debug_to_file:
            with open(self.file_depth_jumps_receive,'ab') as f4:
               for tmp in self.depth_jumps_received_history:
                    np.savetxt(f4, tmp.reshape(1, tmp.shape[0]), delimiter=",")

    def _save_depth_jumps_history(self):
        """
        Save depth jumps history.
        """
        if self.debug_to_file:
            with open(self.file_depth_jumps,'ab') as f4:
                for tmp in self.depth_jumps_history:
                    np.savetxt(f4, tmp.reshape(1, tmp.shape[0]), delimiter=",")

if __name__ == "__main__":
    try:
        gp = GapSensor()
        gp.run()
    except Exception as ex:
        print(ex.message)
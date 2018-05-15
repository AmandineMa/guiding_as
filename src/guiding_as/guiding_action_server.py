#! /usr/bin/env python
"""This module contains an action server class and the nested state machines classes which are executed by the action
server
"""
# from enum import Enum
import math
import rospy
import tf2_ros
import actionlib
import smach
import smach_ros
import copy
from nao_interaction_msgs.srv import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from perspectives_msgs.msg import *
from std_srvs.srv import Trigger
from guiding_as.msg import *
from dialogue_as.msg import *
from deictic_gestures.srv import *
from mummer_integration_msgs.srv import *
from semantic_route_description.srv import *
from perspectives_msgs.srv import *
from ontologenius.srv import *
from route_description.srv import *
from speech_wrapper.srv import *
from multimodal_human_monitor.srv import *
from head_manager.msg import *

__all__ = ['AskHumanToMoveAfter', 'AskPointAgain', 'AskSeen', 'AskShowDirection', 'AskShowPlace', 'DispatchYesNo',
           'DispatchYesNoCL', 'Failure', 'GetRouteRegion', 'GetYesNo', 'GuidingAction', 'HumanLost', 'IsOver',
           'LookAtHuman', 'LookAtHumanAssumedPlace', 'MoveToPose', 'PointAndLookAtHumanFuturePlace',
           'PointAndLookAtLandmark', 'PointingConfig', 'PointNotVisible', 'SaySeen', 'SelectLandmark',
           'ShouldHumanMove', 'StopTrackingCondition', 'Timer']

# constants to get with rospy.get_param
ROBOT_PLACE = ""
POINTING_DURATION = 0
WORLD = ""
STOP_TRACK_DIST_TH = 0
SHOULD_MOVE_DIST_TH = 0
TAKE_ROBOT_PLACE_DIST_TH = 0
LOST_PERCEPTION_TIMEOUT = 0
OBSERVE_LANDMARK_TIMEOUT = 0

# supervisor constants
PERSONNA = "lambda"
SPEECH_PRIORITY = 250

# global variable
human_perceived = False
next_state = ""
person_frame = ""


class GuidingAction(object):
    """Performs the guiding task with one target place and one human.
    """
    _result = taskResult()

    feedback = taskFeedback()
    action_server = None
    dialogue_client = None
    services_proxy = {}
    coord_signals_publisher = None

    def __init__(self, name):
        self._action_name = name

        GuidingAction.action_server = actionlib.SimpleActionServer(self._action_name, taskAction,
                                                                   execute_cb=self.execute_cb,
                                                                   auto_start=False)
        stand_pose_srv = rospy.get_param('/services/stand_pose')
        say_srv = rospy.get_param('/services/say')
        get_route_region_srv = rospy.get_param('/services/get_route_region')
        get_route_description_srv = rospy.get_param('/services/get_route_description')
        get_individual_info_srv = rospy.get_param('/services/get_individual_info')
        get_pointing_config_srv = rospy.get_param('/services/get_pointing_config')
        look_at_srv = rospy.get_param('/services/look_at')
        point_at_srv = rospy.get_param('/services/point_at')
        has_mesh_srv = rospy.get_param('/services/has_mesh')
        monitor_humans_srv = rospy.get_param('/services/monitor_humans')
        start_fact_srv = rospy.get_param('/services/start_fact')
        end_fact_srv = rospy.get_param('/services/end_fact')
        find_alternate_id_srv = rospy.get_param('/services/find_alternate_id')
        activate_dialogue_srv = rospy.get_param('/services/activate_dialogue')
        deactivate_dialogue_srv = rospy.get_param('/services/deactivate_dialogue')

        GuidingAction.coord_signals_publisher = rospy.Publisher(rospy.get_param('/topics/coord_signals'),
                                                                CoordinationSignal, queue_size=5)

        global ROBOT_PLACE
        ROBOT_PLACE = rospy.get_param('/perspective/robot_place')
        global WORLD
        WORLD = rospy.get_param('/perspective/world')
        global POINTING_DURATION
        POINTING_DURATION = rospy.get_param('/tuning_param/pointing_duration')
        global STOP_TRACK_DIST_TH
        STOP_TRACK_DIST_TH = rospy.get_param('/tuning_param/stop_tracking_dist_th')
        global SHOULD_MOVE_DIST_TH
        SHOULD_MOVE_DIST_TH = rospy.get_param('/tuning_param/should_move_dist_th')
        global TAKE_ROBOT_PLACE_DIST_TH
        TAKE_ROBOT_PLACE_DIST_TH = rospy.get_param('/tuning_param/take_robot_place_dist_th')
        global LOST_PERCEPTION_TIMEOUT
        LOST_PERCEPTION_TIMEOUT = rospy.get_param('/tuning_param/lost_perception_timeout')
        global OBSERVE_LANDMARK_TIMEOUT
        OBSERVE_LANDMARK_TIMEOUT = rospy.get_param('/tuning_param/observe_landmark_timeout')

        rospy.loginfo("waiting for service " + stand_pose_srv)
        rospy.wait_for_service(stand_pose_srv)

        rospy.loginfo("waiting for service " + say_srv)
        rospy.wait_for_service(say_srv)

        rospy.loginfo("waiting for service " + get_route_region_srv)
        rospy.wait_for_service(get_route_region_srv)

        rospy.loginfo("waiting for service " + get_route_description_srv)
        rospy.wait_for_service(get_route_description_srv)

        rospy.loginfo("waiting for service " + has_mesh_srv)
        rospy.wait_for_service(has_mesh_srv)

        rospy.loginfo("waiting for service " + get_individual_info_srv)
        rospy.wait_for_service(get_individual_info_srv)

        rospy.loginfo("waiting for service " + get_pointing_config_srv)
        rospy.wait_for_service(get_pointing_config_srv)

        rospy.loginfo("waiting for service " + look_at_srv)
        rospy.wait_for_service(look_at_srv)

        rospy.loginfo("waiting for service " + point_at_srv)
        rospy.wait_for_service(point_at_srv)

        rospy.loginfo("waiting for service " + monitor_humans_srv)
        rospy.wait_for_service(monitor_humans_srv)

        rospy.loginfo("waiting for service " + start_fact_srv)
        rospy.wait_for_service(start_fact_srv)

        rospy.loginfo("waiting for service " + end_fact_srv)
        rospy.wait_for_service(end_fact_srv)

        rospy.loginfo("waiting for service " + find_alternate_id_srv)
        rospy.wait_for_service(find_alternate_id_srv)

        # rospy.loginfo("waiting for service " + activate_dialogue_srv)
        # rospy.wait_for_service(activate_dialogue_srv)
        #
        # rospy.loginfo("waiting for service " + deactivate_dialogue_srv)
        # rospy.wait_for_service(deactivate_dialogue_srv)

        GuidingAction.services_proxy = {
            "stand_pose": rospy.ServiceProxy(stand_pose_srv,
                                             nao_interaction_msgs.srv.GoToPosture),
            "say": rospy.ServiceProxy(say_srv, SpeakTo),
            "get_route_region": rospy.ServiceProxy(get_route_region_srv, SemanticRoute),
            "get_route_description": rospy.ServiceProxy(get_route_description_srv, GetRouteDescription),
            "has_mesh": rospy.ServiceProxy(has_mesh_srv, HasMesh),
            "get_individual_info": rospy.ServiceProxy(get_individual_info_srv, standard_service),
            "get_pointing_config": rospy.ServiceProxy(get_pointing_config_srv, PointingPlanner),
            "look_at": rospy.ServiceProxy(look_at_srv, LookAt),
            "point_at": rospy.ServiceProxy(point_at_srv, PointAt),
            "monitor_humans": rospy.ServiceProxy(monitor_humans_srv, MonitorHumans),
            "start_fact": rospy.ServiceProxy(start_fact_srv, StartFact),
            "end_fact": rospy.ServiceProxy(start_fact_srv, EndFact),
            "find_alternate_id": rospy.ServiceProxy(find_alternate_id_srv, FindAlternateId)}
            # "activate_dialogue": rospy.ServiceProxy(activate_dialogue_srv, Trigger),
            # "deactivate_dialogue": rospy.ServiceProxy(deactivate_dialogue_srv, Trigger)}

        # Build Guiding Container
        self.guiding_sm = smach.StateMachine(outcomes=['task_succeeded', 'task_failed', 'preempted'],
                                             input_keys=['person_frame', 'human_look_at_point'],
                                             output_keys=['last_state', 'last_outcome', 'person_frame'])

        # Add States of Guiding Container
        with self.guiding_sm:
            smach.StateMachine.add('GetRouteRegion', GetRouteRegion(),
                                   transitions={'in_region': 'AskShowPlace', 'out_region': 'AskShowDirection',
                                                'unknown': 'Failure', 'preempted': 'preempted', 'aborted': 'Failure'})

            smach.StateMachine.add('AskShowPlace', AskShowPlace(),
                                   transitions={'succeeded': 'GetYesNo', 'preempted': 'preempted'})

            smach.StateMachine.add('AskShowDirection', AskShowDirection(),
                                   transitions={'succeeded': 'GetYesNo', 'preempted': 'preempted'})

            smach.StateMachine.add('GetYesNo', GetYesNo(),
                                   transitions={'succeeded': 'DispatchYesNo', 'aborted': 'Failure',
                                                'preempted': 'GetYesNo'})

            smach.StateMachine.add('DispatchYesNo', DispatchYesNo(),
                                   transitions={'show': 'Show', 'no': 'task_succeeded', 'preempted': 'preempted'})

            show_sm = smach.StateMachine(outcomes=['show_succeeded', 'show_failed', 'preempted'],
                                         input_keys=['target_frame', 'person_frame', 'human_look_at_point', 'route',
                                                     'last_state', 'last_outcome', 'landmarks_to_point'],
                                         output_keys=['last_state', 'last_outcome', 'person_frame'])

            with show_sm:
                smach.StateMachine.add('PointingConfig', PointingConfig(),
                                       transitions={'succeeded': 'ShouldHumanMove', 'aborted': 'show_failed',
                                                    'preempted': 'preempted', 'point_not_visible': 'SelectLandmark'})

                smach.StateMachine.add('ShouldHumanMove', ShouldHumanMove(),
                                       transitions={'human_first': 'PointAndLookAtHumanFuturePlace',
                                                    'no': 'MoveToPose1',
                                                    'robot_first': 'AskHumanToMoveAfter', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                smach.StateMachine.add('AskHumanToMoveAfter', AskHumanToMoveAfter(),
                                       transitions={'succeeded': 'MoveToPose2', 'preempted': 'preempted'})

                smach.StateMachine.add('PointAndLookAtHumanFuturePlace', PointAndLookAtHumanFuturePlace(),
                                       transitions={'succeeded': 'HumanTracking', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                human_tracking_concurrence = smach.Concurrence(
                    outcomes=['succeeded', 'continue_to_look', 'preempted', 'aborted', 'look_final_dest'],
                    default_outcome='aborted',
                    input_keys=['human_pose', 'person_frame'],
                    child_termination_cb=self.human_tracking_term_cb,
                    outcome_cb=self.human_tracking_out_cb)

                with human_tracking_concurrence:
                    smach.Concurrence.add('LookAtHumanTrack', LookAtHuman())
                    smach.Concurrence.add('StopTrackingCondition', StopTrackingCondition())
                    smach.Concurrence.add('HumanMonitorTrack',
                                          smach_ros.MonitorState("/base/current_facts", FactArrayStamped,
                                                                 self.human_track_perceive_monitor_cb,
                                                                 input_keys=['person_frame']))

                smach.StateMachine.add('HumanTracking', human_tracking_concurrence,
                                       transitions={'succeeded': 'MoveToPose1', 'continue_to_look': 'HumanTracking',
                                                    'look_final_dest': 'LookAtHumanAssumedPlace1',
                                                    'preempted': 'show_failed', 'aborted': 'show_failed'})

                smach.StateMachine.add('LookAtHumanAssumedPlace1', LookAtHumanAssumedPlace(),
                                       transitions={'succeeded': 'MoveToPose1', 'preempted': 'preempted',
                                                    'aborted': 'show_failed', 'look_again': 'LookAtHumanAssumedPlace1'})

                smach.StateMachine.add('MoveToPose1', MoveToPose(),
                                       transitions={'succeeded': 'LookAtHuman2',
                                                    'preempted': 'preempted', 'aborted': 'show_failed'})

                smach.StateMachine.add('LookAtHuman2', LookAtHuman(),
                                       transitions={'succeeded': 'SelectLandmark', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                smach.StateMachine.add('MoveToPose2', MoveToPose(),
                                       transitions={'succeeded': 'LookAtHumanAssumedPlace2',
                                                    'preempted': 'preempted', 'aborted': 'show_failed'})

                smach.StateMachine.add('LookAtHumanAssumedPlace2', LookAtHumanAssumedPlace(),
                                       transitions={'succeeded': 'SelectLandmark', 'preempted': 'preempted',
                                                    'aborted': 'show_failed', 'look_again': 'LookAtHumanAssumedPlace2'})

                smach.StateMachine.add('SelectLandmark', SelectLandmark(),
                                       transitions={'point_not_visible': 'PointNotVisible',
                                                    'point_direction': 'PointAndLookAtLandmark',
                                                    'aborted': 'show_failed',
                                                    'point_target': 'PointAndLookAtLandmark', 'preempted': 'preempted'})

                smach.StateMachine.add('PointNotVisible', PointNotVisible(),
                                       transitions={'succeeded': 'IsOver', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                smach.StateMachine.add('PointAndLookAtLandmark', PointAndLookAtLandmark(),
                                       transitions={'succeeded': 'LookAtHuman1', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                # To observe if the human has seen the target. If the human does not see it after x seconds,
                # the container is terminated and transition to ask if he has seen. If the robot observes the human
                # looking

                # LookAtHuman state if observe target seen
                smach.StateMachine.add('LookAtHuman1', LookAtHuman(),
                                       transitions={'succeeded': 'ObserveLandmarkSeen', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                observe_landmark_seen = smach.Concurrence(outcomes=['detected', 'not_detected'],
                                                          default_outcome='not_detected',
                                                          input_keys=['target_frame', 'person_frame'],
                                                          child_termination_cb=self.observe_term_cb,
                                                          outcome_cb=self.observe_out_cb)

                with observe_landmark_seen:
                    smach.Concurrence.add('Timer', Timer())
                    smach.Concurrence.add('HumanMonitorTargetSeen',
                                          smach_ros.MonitorState("/base/current_facts", FactArrayStamped,
                                                                 self.human_monitor_target_seen_cb,
                                                                 input_keys=['person_frame', 'target_frame']))

                smach.StateMachine.add('ObserveLandmarkSeen', observe_landmark_seen,
                                       transitions={'detected': 'SaySeen', 'not_detected': 'CheckLandmarkSeen'})

                smach.StateMachine.add('SaySeen', SaySeen(), transitions={'succeeded': 'IsOver',
                                                                          'preempted': 'preempted'})

                # Container to ask the human if he sees the landmark (case where the robot did not observe it)
                check_landmark_seen_sm = smach.StateMachine(outcomes=['yes', 'no', 'pointing', 'preempted', 'failure'],
                                                            input_keys=['human_look_at_point', 'last_state',
                                                                        'person_frame', 'last_outcome'])

                with check_landmark_seen_sm:
                    check_landmark_seen_sm.add('AskSeen', AskSeen(), transitions={'succeeded': 'GetYesNoCL',
                                                                                  'preempted': 'preempted'})

                    check_landmark_seen_sm.add('AskPointAgain', AskPointAgain(),
                                               transitions={'succeeded': 'GetYesNoCL', 'preempted': 'preempted'})

                    check_landmark_seen_sm.add('GetYesNoCL', GetYesNo(),
                                               transitions={'succeeded': 'DispatchYesNoCL', 'preempted': 'GetYesNoCL',
                                                            'aborted': 'failure'})

                    check_landmark_seen_sm.add('DispatchYesNoCL', DispatchYesNoCL(),
                                               transitions={'yes': 'yes',
                                                            'no': 'no',
                                                            'ask_point_again': 'AskPointAgain',
                                                            'pointing': 'pointing', 'preempted': 'preempted'})

                smach.StateMachine.add('CheckLandmarkSeen', check_landmark_seen_sm,
                                       transitions={'yes': 'IsOver', 'no': 'IsOver',
                                                    'pointing': 'PointAndLookAtLandmark',
                                                    'failure': 'show_failed', 'preempted': 'preempted'})

                smach.StateMachine.add('IsOver', IsOver(),
                                       transitions={'succeeded': 'show_succeeded', 'point_direction': 'SelectLandmark',
                                                    'preempted': 'preempted', 'aborted': 'show_failed'})

            smach.StateMachine.add('Show', show_sm, transitions={'show_succeeded': 'task_succeeded',
                                                                 'show_failed': 'Failure', 'preempted': 'preempted'})

            # Every state which failed transitions to this state
            smach.StateMachine.add('Failure', Failure(), transitions={'aborted': 'task_failed'})

        guiding_monitoring = smach.Concurrence(outcomes=['task_succeeded', 'task_failed', 'preempted', 'human_lost'],
                                               default_outcome='task_failed',
                                               outcome_cb=self.g_m_c_out_cb,
                                               input_keys=['person_frame', 'human_look_at_point'],
                                               child_termination_cb=self.g_m_c_term_cb)
        with guiding_monitoring:
            smach.Concurrence.add('GUIDING', self.guiding_sm)
            smach.Concurrence.add('HUMAN_MONITOR', smach_ros.MonitorState(rospy.get_param("/topics/current_facts"),
                                                                          FactArrayStamped,
                                                                          self.human_perceive_monitor_cb,
                                                                          input_keys=['person_frame'],
                                                                          output_keys=['duration_lost',
                                                                                       'human_lost']))

        self.top_sm = smach.StateMachine(outcomes=['task_succeeded', 'task_failed', 'preempted'])

        with self.top_sm:
            smach.StateMachine.add('GUIDING_MONITORING', guiding_monitoring,
                                   transitions={'task_succeeded': 'task_succeeded', 'task_failed': 'task_failed',
                                                'preempted': 'preempted', 'human_lost': 'HumanLost'})
            smach.StateMachine.add('HumanLost', HumanLost())

        # callbacks registration
        self.guiding_sm.register_transition_cb(self.guiding_transition_cb, cb_args=[self.guiding_sm])
        self.guiding_sm.register_start_cb(self.guiding_start_cb, cb_args=[])
        self.guiding_sm.register_termination_cb(self.term_cb, cb_args=[])
        show_sm.register_start_cb(self.guiding_start_cb, cb_args=[])
        show_sm.register_transition_cb(self.guiding_start_cb, cb_args=[show_sm])
        check_landmark_seen_sm.register_transition_cb(self.guiding_transition_cb, cb_args=[check_landmark_seen_sm])

        self.sis = smach_ros.IntrospectionServer('server_name', self.top_sm, '/SM_ROOT')
        self.sis.start()
        smach_ros.set_preempt_handler(self.top_sm)

        rospy.loginfo("start action server")
        GuidingAction.action_server.start()
        rospy.loginfo("action server started")

        # GuidingAction.services_proxy["deactivate_dialogue"]()

    def guiding_start_cb(self, userdata, initial_states, *cb_args):
        """Callback to initialize the userdata.active_state variable"""
        userdata.last_state = initial_states[0]
        global next_state
        next_state = initial_states[0]

    def guiding_transition_cb(self, userdata, active_states, *cb_args):
        """Callback to update the userdata.last_state variable"""
        if active_states[0] != 'Failure':
            userdata.last_state = active_states[0]
        userdata.last_outcome = cb_args[0].get_last_outcome()
        global next_state
        next_state = cb_args[0].get_next_state()
        userdata.person_frame = person_frame

    def term_cb(self, userdata, terminal_states, *cb_args):
        """Callback to return the failure reason to the action server client"""
        if terminal_states[0] == 'Failure':
            self._result.failure_reason = userdata.last_state
            GuidingAction.action_server.set_aborted(self._result)

    human_lost = False
    time_lost = 0
    duration_lost = 0
    fact_id = ""

    def human_perceive_monitor_cb(self, userdata, msg):
        """Callback that writes in the human_perceived global variable if the human is being perceived by the robot
        or not. It performs it during the whole task.
        """
        global human_perceived
        perceived = False
        for fact in msg.facts:
            if fact.predicate == "isPerceiving" and fact.object_name == userdata.person_frame \
                    and fact.subject_name == 'robot':
                perceived = True

        if perceived:
            human_perceived = True
            if self.human_lost:
                rospy.logwarn("human found again")
                # rospy.logwarn("fact id %s"+self.fact_id)
                # GuidingAction.services_proxy["end_fact"](WORLD, self.fact_id)
                self.human_lost = False
                self.duration_lost = 0
        else:
            human_perceived = False
            if next_state not in ('MoveToPose1', 'MoveToPose2', 'PointNotVisible', 'PointAndLookAtLandmark',
                                  'PointAndLookAtHumanFuturePlace'):
                if not self.human_lost:
                    rospy.logwarn("human lost")
                    self.human_lost = True
                    self.time_lost = rospy.Time.now()
                    # start_fact = GuidingAction.services_proxy["start_fact"](WORLD,
                    #                                                         "isLost(" + userdata.person_frame + ")",
                    #                                                         rospy.Time().to_sec(), False)
                    # if start_fact.success:
                    #     self.fact_id = start_fact.fact_id
                else:
                    self.duration_lost = (rospy.Time.now() - self.time_lost).to_sec()

                # rospy.logwarn("lost since "+str(self.duration_lost))

                # [TO_TUNE] to stop the state machine if human not perceived after X time
                if rospy.Duration(self.duration_lost) > rospy.Duration(LOST_PERCEPTION_TIMEOUT):
                    return False

            # pas joli de faire ca ici
            try:
                alt_id = GuidingAction.services_proxy["find_alternate_id"](userdata.person_frame)
                if alt_id.success:
                    monitor_humans_request = MonitorHumansRequest()
                    monitor_humans_request.action = "REMOVE"
                    try:
                        GuidingAction.services_proxy["monitor_humans"](monitor_humans_request)
                        global person_frame
                        person_frame = alt_id.result_id
                        monitor_humans_request.action = "ADD"
                        monitor_humans_request.humans_to_monitor = [alt_id.result_id]
                        GuidingAction.services_proxy["monitor_humans"](monitor_humans_request)
                    except rospy.ServiceException, e:
                        rospy.logerr("monitor human exception")
                    rospy.logwarn(alt_id.result_id)
            except rospy.ServiceException, e:
                rospy.logerr("find alternate id exception")

            userdata.duration_lost = self.duration_lost
            userdata.human_lost = self.human_lost

        return True

    def g_m_c_term_cb(self, outcome_map):
        """Callback that terminates the GUIDING_MONITORING container when the GUIDING state machine finishes."""
        # [TO_TUNE] add " or outcome_map['HUMAN_MONITOR'] == 'invalid' " condition to stop the state machine if the
        # human is not perceived
        if outcome_map['GUIDING'] == 'task_succeeded' or outcome_map['GUIDING'] == 'task_failed' \
                or outcome_map['GUIDING'] == 'preempted' or outcome_map['HUMAN_MONITOR'] == 'invalid':
            return True
        else:
            return False

    def g_m_c_out_cb(self, outcome_map):
        """Callback that returns the outcomes of the GUIDING_MONITORING container."""
        if outcome_map['GUIDING'] == 'task_succeeded':
            return 'task_succeeded'
        elif outcome_map['GUIDING'] == 'task_failed':
            return 'task_failed'
        else:
            return 'human_lost'

    def human_tracking_term_cb(self, outcome_map):
        """Callback that terminates the HumanTracking concurrent container in either of those cases:

        * the human has not reached his pose and the robot finishes to perform the look at (if the condition
          outcome_map['LookAtHumanTrack'] == 'succeeded' is not set, the LookAtHumanTrack state will be preempted if the
          StopTrackingCondition finishes first)
        * the human has reached his pose
        * the human is not perceived anymore

        TODO: add the preempted and aborted cases
        """
        if (outcome_map['StopTrackingCondition'] == 'continue_tracking' and outcome_map[
            'LookAtHumanTrack'] == 'succeeded') and outcome_map['HumanMonitorTrack'] =='valid' \
                or outcome_map['StopTrackingCondition'] == 'succeeded' or outcome_map['HumanMonitorTrack'] == 'invalid':
            return True
        else:
            return False

    def human_tracking_out_cb(self, outcome_map):
        """Callback that directs the different outcomes of the HumanTracking concurrent container.

        There are 3 possible outcomes:

        - look_at_final_dest: the human is not perceived anymore, the robot's next step is to look at his expected pose
        - succeeded: the human reached his expected pose
        - continue_to_look: the robot can still track the human who has not arrived at his expected pose yet, it means
          the HumanTrackingHumanTracking container will be called again
        """
        if outcome_map['HumanMonitorTrack'] == 'invalid':
            return 'look_final_dest'
        if outcome_map['StopTrackingCondition'] == 'succeeded':
            return 'succeeded'
        elif outcome_map['StopTrackingCondition'] == 'continue_tracking' and \
                outcome_map['LookAtHumanTrack'] == 'succeeded':
            return 'continue_to_look'

    count = 0

    def human_track_perceive_monitor_cb(self, userdata, msg):
        """Callback that returns False if the human is not perceived after X times, during the tracking of the
        human going from his start pose to the expected pose. A False means that the calling HumanMonitorTrack state
        will return 'invalid'
        """
        perceived = False
        for fact in msg.facts:
            if fact.predicate == "isPerceiving" and fact.object_name == userdata.person_frame \
                    and fact.subject_name == 'robot':
                perceived = True

        if not perceived:
            self.count += 1
        else:
            return True

        if self.count > 15:
            self.count = 0
            return False
        else:
            return True

    # ------ Callback to handle the monitoring of if the target is visible by the human or not ------ #

    # Return 'invalid' if the target is visible by the human
    def human_monitor_target_seen_cb(self, ud, msg):
        for fact in msg.facts:
            if fact.predicate == "isVisibleBy" \
                    and fact.object_name == ud.person_frame \
                    and fact.subject_name == ud.target_frame:
                rospy.logwarn("visible")
                return False
            else:
                return True

    # ------ Callbacks to handle the termination and the outcome of the ObserveLandmarkSeen container ------ #

    def observe_term_cb(self, outcome_map):
        # Terminate the concurrent container ObserveLandmarkSeen if one of these two outcomes is reached
        if outcome_map['Timer'] == 'stop' or outcome_map['HumanMonitorTargetSeen'] == 'invalid':
            return True
        else:
            return False

    def observe_out_cb(self, outcome_map):
        if outcome_map['Timer'] == 'stop':
            return 'not_detected'
        elif outcome_map['HumanMonitorTargetSeen'] == 'invalid':
            return 'detected'
        else:
            return 'not_detected'

    def execute_cb(self, goal):
        """ Execute the nested state machines
        Userdata of the state machines:

        - person_frame: tf frame of the human the robot is interacting with
        - target_frame: tf frame of the target place (where the human wants to go)
        - human_look_at_point: geometry_msgs.PointStamped which contains the human frame in its header.frame_id
          (exists by convenience, to not have to creates a new one for each look at action)
        - route: route returned by the route planner (list of str)
        - human_pose: the human wanted/optimal pose returned by the pointing planner
        - target_pose: the robot optimal pose returned by the pointing planner
        - landmarks_to_point: list of landmarks to point returned by the pointing planner (list of str)
        - landmark_to_point: the selected landmark to point (output_key of the SelectLandmark state)
        - question_asked: the question asked by one of the "Ask" states
        - result_word: word returned by the dialogue action server ('yes' or 'no')
        - last_outcome: Outcome returned by the last state the machine was in
        - last_state: Last active state the machine was in
        """
        self.top_sm.set_initial_state(['GUIDING_MONITORING'])
        # userdata which needs to be initiate
        self.top_sm.userdata.person_frame = goal.person_frame
        self.guiding_sm.userdata.target_frame = goal.place_frame
        global person_frame
        person_frame = goal.person_frame
        global human_perceived
        human_perceived = False
        global next_state
        next_state = ""
        self.human_lost = False
        self.time_lost = 0
        self.duration_lost = 0
        self.fact_id = ""
        self.count = 0
        LookAtHumanAssumedPlace.does_not_see = 0
        SelectLandmark.target_pointed = False
        GetYesNo.repeat_question = 0

        look_at_point = PointStamped()
        look_at_point.header.frame_id = self.top_sm.userdata.person_frame
        self.top_sm.userdata.human_look_at_point = look_at_point

        self.guiding_sm.userdata.route = None
        self.guiding_sm.userdata.last_outcome = None
        self.guiding_sm.userdata.landmarks_to_point = []

        outcome = self.top_sm.execute()

        # if self.human_lost:
        #     try:
        #         GuidingAction.services_proxy["end_fact"](WORLD, self.fact_id)
        #     except rospy.ServiceException, e:
        #         rospy.logerr("end fact exception")

        # Before shutdown
        GuidingAction.feedback.current_step = "Go to stand pose"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        self.stand_pose()

        rospy.loginfo("State machine outcome : %s ", outcome)
        if outcome == 'task_succeeded':
            self._result.success = True
            self._result.failure_reason = ''
        else:
            self._result.success = False
            if self.guiding_sm.userdata.last_state is None:
                self._result.failure_reason = "SM did not start"
            else:
                self._result.failure_reason = self.guiding_sm.userdata.last_state

        GuidingAction.action_server.set_succeeded(self._result)

        self.sis.stop()

    @staticmethod
    def stand_pose():
        rospy.loginfo("stand pose")
        go_to_posture_request = GoToPostureRequest()
        go_to_posture_request.posture_name = "StandInit"
        go_to_posture_request.speed = 1
        GuidingAction.services_proxy["stand_pose"](go_to_posture_request)


class GetRouteRegion(smach.State):
    """Write to userdata.route the best route to go from ROBOT_PLACE to the place asked by the human.

    There are 5 possible outcomes:

    - in_region: if the robot is in the same region than the place right now
    - out_region: the place is located in a different region than the robot's one
    - unknown: the region is unknown from the robot
    - preempted: the state has been preempted
    - aborted: an exception has been returned by the service called

    """

    def __init__(self):
        """Constructor for GetRouteRegion state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['in_region', 'out_region', 'unknown', 'preempted', 'aborted'],
                             input_keys=['target_frame', 'human_look_at_point'],
                             io_keys=['route'])

    def _select_best_route(self, routes, costs):
        """Select the best route among the list of routes, according to the cost of each one"""
        best_route = []
        min_cost = None
        if len(routes) > 0:
            if len(routes) == 1:
                best_route = routes[0]
            else:
                for i in range(0, len(routes), 1):
                    if min_cost is None:
                        min_cost = costs[i]
                        best_route = routes[i]
                    else:
                        if costs[i] < min_cost:
                            best_route = routes[i]
                            min_cost = costs[i]

        return best_route

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the route planner which returns a list of possible routes. Then select the best one among those.

        If the list of route is empty, it means the place is unknown.

        If the list of route has only one element, it means the target place is in the same region than the robot.

        If the list of route has more than one element, it means the target place is in a different region.
        """
        GuidingAction.feedback.current_step = "get_route_region"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        try:
            get_route_region = GuidingAction.services_proxy["get_route_region"](
                ROBOT_PLACE,
                userdata.target_frame,
                PERSONNA,
                False)

        except rospy.ServiceException, e:
            return 'aborted'

        # if the routes list returned by the route planner is not empty
        if len(get_route_region.routes) != 0:
            userdata.route = self._select_best_route(get_route_region.routes, get_route_region.costs).route
        else:
            userdata.route = []

        # if the routes list returned by the route planner is empty, the target is unknown
        if len(userdata.route) == 0:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point, "I'm sorry, I don't know this place",
                                                SPEECH_PRIORITY)
            return 'unknown'

        elif len(userdata.route) == 1:
            return 'in_region'
        else:
            return 'out_region'


class AskShowPlace(smach.State):
    """Case where the robot is in the same region than the place

    Ask to the human : TARGET_NAME is nearby. Would you like me to show you the shop ?

    Then, write "ask_show_target" to userdata.question_asked, it will be used by the state L{DispatchYesNo}.

    There are 2 possible outcomes:

    - succeeded
    - preempted: the state has been preempted

    """

    def __init__(self):
        """Constructor for AskShowPlace state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['target_frame', 'human_look_at_point'],
                             output_keys=['question_asked'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the speech service to ask the question"""
        GuidingAction.feedback.current_step = "Ask if the robot should show the shop in region"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        target_name = GuidingAction.services_proxy["get_individual_info"]("getName", userdata.target_frame)
        # If a verbalized/label name exists in the ontology, it is used
        if target_name.code == 0:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                target_name.value +
                                                "is nearby. Would you like me to show you the shop ?",
                                                SPEECH_PRIORITY)
        # If there is not, the 'technical' name is used
        else:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                userdata.target_frame +
                                                "is nearby. Would you like me to show you the shop ?",
                                                SPEECH_PRIORITY)

        userdata.question_asked = 'ask_show_target'
        return 'succeeded'


class AskShowDirection(smach.State):
    """Case where the robot is not in the same region than the place

    First, it calls a service to get the verbalized description of the route.

    Then, it asks to the human : It's not here. You need to ROUTE_DESCRIPTION.
                                Would you like me to show you the way to go to the shop ?
    Finally, it writes "ask_show_direction" to userdata.question_asked, it will be used by the state L{DispatchYesNo}.

    There are 2 possible outcomes:

    - succeeded
    - preempted: the state has been preempted

    """

    def __init__(self):
        """Constructor for AskShowDirection state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.
        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['target_frame', 'human_look_at_point'],
                             output_keys=['question_asked'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the speech service to ask the question"""
        GuidingAction.feedback.current_step = "Ask if the robot should show the direction"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        # call the route description service
        route_description = \
            GuidingAction.services_proxy["get_route_description"](ROBOT_PLACE, userdata.target_frame).region_route

        GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                            "It's not here. You need to " + route_description +
                                            "Would you like me to show you the way to go to the shop ?",
                                            SPEECH_PRIORITY)

        userdata.question_asked = 'ask_show_direction'
        return 'succeeded'


class PointingConfig(smach.State):
    """Calls the pointing planner to get the best configuration (pose) for the human and the robot

    It extracts the first passage from the route returned by the route planner (route[1])
    and write it in a variable called direction. If len(route) == 1, direction is empty.

    The pointing planner takes in parameters: the place, the direction  and the human tf frame. Before calling the
    pointing planner, it checks if there is an associated mesh to the place. If there is one, it calls the pointing
    planner.

    The pointing planner returns a pose each for the robot and one for the human.
    It also returns a list of string that contains the landmarks which will be visible from this configuration.

    - If len(landmarks_to_point) == 0, the pointing planner failed
    - If landmarks_to_point = [TARGET] -> only the target will be visible (normally, this case happens only when the
      robot is in the same region than the target because the pointing planner defined the visibility of the direction
      as a priority compared to the visibility of the target)
    - If landmarks_to_point = [DIRECTION] -> only the direction will be visible
    - If landmarks_to_point = [TARGET, DIRECTION] -> both will be visible

    It writes in 3 userdata variables:

    - target_pose: the pose for the robot returned by the pointing planner
    - human_pose: the pose for the human returned by the pointing planner
    - landmarks_to_point: the list of string of landmarks to point returned by the pointing planner

    There are 3 possible outcomes:

    - succeeded: it went well
    - preempted: the state has been preempted
    - aborted: the pointing planner raised an exception or returned an empty list of visible landmarks
    """

    def __init__(self):
        """Constructor for PointingConfig state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'point_not_visible'],
                             input_keys=['target_frame', 'person_frame', 'route', 'human_look_at_point', 'landmarks_to_point'],
                             output_keys=['target_pose', 'landmarks_to_point', 'human_pose'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Does some tests, then calls the pointing planner service"""
        GuidingAction.feedback.current_step = "get_pointing_config"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)

        direction_landmark = ""
        target_frame = ""
        # if there is a direction landmark in the route list returned by the route planner
        if len(userdata.route) > 2:
            direction_landmark = userdata.route[1]

        # Check if it exists an associated mesh to the target
        has_mesh_result = GuidingAction.services_proxy["has_mesh"](WORLD, userdata.target_frame)
        if not has_mesh_result.success:
            rospy.logerr("has_mesh service failed")
        else:
            # if there is no mesh, the target_frame remains empty
            if has_mesh_result.has_mesh:
                target_frame = userdata.target_frame

        try:
            # call the pointing planner service
            get_pointing_config = GuidingAction.services_proxy["get_pointing_config"](
                target_frame,
                direction_landmark,
                userdata.person_frame)

            # write in the userdata
            userdata.target_pose = get_pointing_config.robot_pose
            userdata.human_pose = get_pointing_config.human_pose
            # remplissage du tableau dans l'ordre target s'il y en a une, puis direction
            if userdata.target_frame in get_pointing_config.pointed_landmarks:
                userdata.landmarks_to_point.append(userdata.target_frame)
            if direction_landmark in get_pointing_config.pointed_landmarks:
                userdata.landmarks_to_point.append(direction_landmark)

            if self.preempt_requested():
                rospy.loginfo(self.get_name() + " preempted")
                self.service_preempt()
                return 'preempted'

            if len(get_pointing_config.pointed_landmarks) == 0:
                return 'point_not_visible'
            else:
                return 'succeeded'

        except rospy.ServiceException, e:
            rospy.logerr('pointing planner exception')
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                "I am sorry, my pointing planner crashed !",
                                                SPEECH_PRIORITY)
            return 'aborted'


class ShouldHumanMove(smach.State):
    """"Based on the human pose returned by the pointing planner, it evaluates if the robot should
    ask the human to move or not, and if he has to move, if he should move before or after the robot move.

    It decides according to the distance criteria. If the actual human pose is close to the calculate human position
    (dist between actual and calculate position < 0.5), then the human the human does not need to move.

    If the human needs to move, there are 2 possible options: if the calculate human position is close to the actual
    robot position (dist < 0.6), then the human should wait that the robot moves first to take its actual spot.

    If not, the human can move first.

    There are 5 possible outcomes:

    - robot_first: the robot should move first so the human will take its actual position
    - human_first: the human should move first, the robot will follow his movement and will start to move when the human
      will reach his position
    - no: the human does not need to move, only the robot will do.
    - preempted: the state has been preempted
    - aborted: tf raised an exception
    """

    def __init__(self):
        """Constructor for ShouldHumanMove state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['robot_first', 'human_first', 'no', 'preempted', 'aborted'],
                             input_keys=['person_frame', 'human_pose'])
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calculates the distance between the actual human position and the one returned by the pointing planner,
        the distance between the actual robot position and the human's one returned by the pointing planner, then
        estimates the outcome to return."""
        GuidingAction.feedback.current_step = "Determine if the human should move or not"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        try:
            # calculate the distance between the human actual pose and the human wanted pose
            trans = self._tfBuffer.lookup_transform('map', userdata.person_frame, rospy.Time(0))
            distance_h_now_h_future = math.sqrt(
                (trans.transform.translation.x - userdata.human_pose.pose.position.x) ** 2 +
                (trans.transform.translation.y - userdata.human_pose.pose.position.y) ** 2)
            rospy.logwarn("dist human now future : %f", distance_h_now_h_future)

            if self.preempt_requested():
                rospy.loginfo(self.get_name() + " preempted")
                self.service_preempt()
                return 'preempted'

            # if the distance is superior to the threshold, the human will be asked to move
            if distance_h_now_h_future > SHOULD_MOVE_DIST_TH:
                try:
                    # calculate the distance between the robot actual pose and the human wanted pose
                    trans = self._tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
                    distance_r_now_h_future = math.sqrt(
                        (trans.transform.translation.x - userdata.human_pose.pose.position.x) ** 2 +
                        (trans.transform.translation.y - userdata.human_pose.pose.position.y) ** 2)
                    rospy.logwarn("dist robot now human future : %f", distance_r_now_h_future)
                    # if the distance is inferior to the threshold, the human should take the actual robot place,
                    # therefore the robot should move first
                    if distance_r_now_h_future < TAKE_ROBOT_PLACE_DIST_TH:
                        return 'robot_first'
                    # the human wanted pose is at a certain distance of the robot, the human can move first
                    else:
                        return 'human_first'

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr('tf exception')
                    return 'aborted'

            else:
                return 'no'

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('tf exception')
            return 'aborted'


class AskHumanToMoveAfter(smach.State):
    """Case where the robot has to move first. It asks the human to come to its current place.

    There are 2 possible outcomes:

    - succeeded
    - preempted: the state has been preempted
    """

    def __init__(self):
        """Constructor for AskHumanToMoveAfter state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['human_look_at_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the speech service to ask the human to move"""
        GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                            "I am going to move, so you can come to my current place."
                                            "You will better see from here.",
                                            SPEECH_PRIORITY)
        GuidingAction.feedback.current_step = "Ask human to move after"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


class PointAndLookAtHumanFuturePlace(smach.State):
    """Announces to the human that he is supposed to come to its actual place.

    Then, point and look at where the human was supposed to go (with z = 0) so head and arm pointing at the floor.

    There are 3 possible outcomes:

    - succeeded: it went well
    - preempted: the state has been preempted
    - aborted: the look at or the point at services failed

    """

    def __init__(self):
        """Constructor for PointAndLookAtHumanFuturePlace state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['landmark_to_point', 'human_look_at_point', 'human_pose'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the speech, point_at and look_at services"""
        GuidingAction.feedback.current_step = "Point at where the human should go"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        try:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                "I need you to make a few steps.... You will see better "
                                                "what I am about to show you. Can you go there ?",
                                                SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr("speech exception")

        coord_signal = CoordinationSignal()
        coord_signal.header.frame_id = userdata.human_pose.header.frame_id
        target = TargetWithExpiration()
        target.target.header.frame_id = userdata.human_pose.header.frame_id
        target.target.point = userdata.human_pose.pose.position
        target.duration = rospy.Duration(2.0)
        coord_signal.targets.append(target)
        coord_signal.priority = 100
        coord_signal.expiration = rospy.Time() + rospy.Duration(3.0)
        # coord_signal.regex_end_condition = "isPerceiving\(robot," + userdata.human_pose.header.frame_id + "\)"
        coord_signal.predicate = "isWaiting(robot," + userdata.human_pose.header.frame_id + ")"
        GuidingAction.coord_signals_publisher.publish(coord_signal)

        point_at_request = PointAtRequest()
        point_at_request.point.header = userdata.human_pose.header
        point_at_request.point.point = userdata.human_pose.pose.position
        point_at = GuidingAction.services_proxy["point_at"](point_at_request)

        # look_at_request = LookAtRequest()
        # look_at_request.point.header = userdata.human_pose.header
        # look_at_request.point.point = userdata.human_pose.pose.position
        # look_at = GuidingAction.services_proxy["look_at"](look_at_request)
        rospy.sleep(POINTING_DURATION)

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        if point_at.success:
            return 'succeeded'
        else:
            if not point_at.success:
                rospy.logerr("pointing service failed")
            return 'aborted'


class LookAtHumanAssumedPlace(smach.State):
    """The robot looks at where it asked the human to go. It checks if the human is perceived
    and if not (check the variable human_perceived modified by the callback on the facts listener),
    returned that is should look again. After 2 times where the human has not been seen, it will assume that
    he is here and continue the interaction (it is not the proper way to handle it but for now as we are not able to
    detect an ID change yet, we do not take the risk to end the interaction whereas the person is still here)

    There are 3 possible outcomes:

    - succeeded: it went well
    - look_again : it did not see the human so it will try to look again
    - preempted: the state has been preempted
    - aborted: the look at or the point at services failed

    """

    does_not_see = 0

    def __init__(self):
        """Constructor for LookAtHumanAssumedPlace state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'look_again', 'preempted', 'aborted'],
                             input_keys=['human_pose', 'human_look_at_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the look_at service, check if the human is perceived and may speak according to the perception
        result"""

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        # look_at_request = LookAtRequest()
        # # deepcopy allows to copy human_pose so when look_at_request.point is modified, it does not affect human_pose
        # look_at_request.point.header.frame_id = copy.deepcopy(userdata.human_pose.header.frame_id)
        # look_at_request.point.point = copy.deepcopy(userdata.human_pose.pose.position)
        # # so the robot look at an averaged human size
        # look_at_request.point.point.z += 1.5
        # look_at = GuidingAction.services_proxy["look_at"](look_at_request)

        look_at = PointStamped()
        look_at.header.frame_id = copy.deepcopy(userdata.human_pose.header.frame_id)
        look_at.point = copy.deepcopy(userdata.human_pose.pose.position)
        look_at.point.z += 1.5

        coord_signal = CoordinationSignal()
        coord_signal.header.frame_id = look_at.header.frame_id
        target = TargetWithExpiration()
        target.target = look_at
        target.duration = rospy.Duration(4.0)
        coord_signal.targets.append(target)
        coord_signal.priority = 100
        coord_signal.expiration = rospy.Time() + rospy.Duration(3.0)
        coord_signal.regex_end_condition = "isPerceiving\(robot," + userdata.human_pose.header.frame_id + "\)"
        coord_signal.predicate = "isWaiting(robot," + userdata.human_pose.header.frame_id + ")"
        GuidingAction.coord_signals_publisher.publish(coord_signal)

        # if the human is not perceived for less than 3 times
        if not human_perceived and self.does_not_see < 2:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                "I am sorry, I cannot see you. Can you come in front of me ?",
                                                SPEECH_PRIORITY)
            self.does_not_see += 1
            # wait a bit before checking again
            rospy.logwarn("wait human to come in front of it")
            rospy.sleep(3.0)
            return 'look_again'
        # if the human is still not perceived
        # (It can happen if the human is in front of the robot but its ID changed - Maybe the interaction should
        # stop instead of still being going on)
        # elif not human_perceived:
        #     GuidingAction.services_proxy["say"](userdata.human_look_at_point,
        #                                         "I still cannot see you but I will assume that you are here",
        #                                         SPEECH_PRIORITY)
        self.does_not_see = 0

        return 'succeeded'
        # if look_at.success:
        #     return 'succeeded'
        # else:
        #     rospy.logerr('look at failed')
        #     return 'aborted'


class StopTrackingCondition(smach.State):
    """It checks the distance between the human actual pose/frame and the human final/wanted pose.

    If the distance is inferior to a certain threshold, it means the human reached the wanted pose, therefore,
    it returns that the tracking has to stop. Otherwise, it returns that the tracking have to continue.

    There are 4 possible outcomes:

    - succeeded: it went well
    - look_again : it did not see the human so it will try to look again
    - preempted: the state has been preempted
    - aborted: the look at or the point at services failed

    """

    def __init__(self):
        """Constructor for StopTrackingCondition state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['continue_tracking', 'succeeded', 'preempted'],
                             input_keys=['human_pose', 'person_frame'])
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calculate the distance between the human actual pose/frame and the human final/wanted pose and returns if
        it should continue to track or not."""
        GuidingAction.feedback.current_step = "stop tracking condition"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        # Calculate the distance between the human actual pose/frame and the human final/wanted pose
        trans = self._tfBuffer.lookup_transform('map',
                                                userdata.person_frame, rospy.Time(0))
        distance = math.sqrt((trans.transform.translation.x - userdata.human_pose.pose.position.x) ** 2 +
                             (trans.transform.translation.y - userdata.human_pose.pose.position.y) ** 2)
        # if the wanted pose is reached (+/- a certain threshold)
        if distance < STOP_TRACK_DIST_TH:
            return 'succeeded'
        else:
            return 'continue_tracking'


class MoveToPose(smach_ros.SimpleActionState):
    """It calls the navigation action server to reach the pose returned by the pointing planner

    There are 3 possible outcomes:

    - succeeded: it went well
    - preempted: the state has been preempted
    - aborted: the navigation action server failed
    """

    def __init__(self):
        """Constructor for MoveToPose state

        It calls the super constructor of L{smach.State} and defines the outcomes, the input_keys, the output_keys,
        the 3 action state callbacks, each one for the goal, the feedback and the result, and a server timeout.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach_ros.SimpleActionState.__init__(self, rospy.get_param("/action_servers/move_to"),
                                             MoveBaseAction, goal_cb=self.move_to_goal_cb,
                                             feedback_cb=self.move_to_feedback_cb,
                                             result_cb=self.move_to_result_cb,
                                             input_keys=['target_pose', 'human_look_at_point'],
                                             server_wait_timeout=rospy.Duration(2)),

    def get_name(self):
        return self.__class__.__name__

    def move_to_goal_cb(self, userdata, goal):
        """Announces to the human that it is about to move. Then, creates a MoveBaseGoal to the to the action server
        with the pose the robot should reach."""
        try:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Now wait, I am going to move.",
                                                SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr("speech exception")
        GuidingAction.feedback.current_step = "moving"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        move_to_goal = MoveBaseGoal()
        move_to_goal.target_pose = userdata.target_pose
        return move_to_goal

    def move_to_feedback_cb(self, userdata, feedback):
        # rospy.loginfo(feedback.base_position)
        pass

    def move_to_result_cb(self, userdata, status, result):
        """Returns succeeded if the navigation reached its goal, otherwise returns aborted"""
        if status == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        else:
            rospy.logerr('navigation failed')
            return 'aborted'


# class LandmarkType(Enum):
#: To access in a list, the landmark of target type or to define a landmark of target type
LANDMARK_TYPE_TARGET = 0
#: To access in a list, the landmark of direction type or to define a landmark of direction type
LANDMARK_TYPE_DIRECTION = 1

#: To access in a list, the landmark name
LANDMARK_NAME = 0
#: To access in a list, the landmark type
LANDMARK_TYPE = 1


class SelectLandmark(smach.State):
    """Selects between the target and the direction which one should be pointed at, according to
    the route returned by the route planner and the list of landmarks to point returned by the pointing planner.

    The first time the state is called, it will return either point_target or point_not_visible in the case the target
    was not returned by the pointing planner.

    Then, it is the state IsOver which decides if SelectLandmark should be called a second time. At the second call,
    it checks that first the target has been pointed at and then checks that the direction is in the list of things to
    point at (if not, it's not the expected case)
    (TODO: maybe the SelectLandmark state should be called 2 times in any case and returned another outcome if there is no direction)

    It writes in the userdata.landmark_to_point the next landmark to point by the robot. It is a tuple
    (landmark_name, landmark_type)

    There are 5 possible outcomes:

    - point_target: the pointing planner returned the target to point at, it is the first pass in the state
    - point_not_visible: the pointing planner did not returned the target to point at, either because the found optimal
      position allowed to point only the direction (to point at the direction is a priority compared to the target).
      It is the first pass in the state
    - point_direction:  the pointing planner returned the direction to point at, it is the second pass in the state
    - preempted: the state has been preempted
    - aborted: the state should not have been called a second time
    """
    target_pointed = False

    def __init__(self):
        """Constructor for SelectLandmark state

        It calls the super constructor of L{smach.State} and defines the outcomes, the input_keys, the output_keys.
        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['point_not_visible', 'point_direction',
                                             'point_target', 'preempted', 'aborted'],
                             input_keys=['target_frame', 'route', 'landmarks_to_point', 'human_look_at_point'],
                             output_keys=['landmark_to_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """It computes what should be point at."""
        # if a direction exists in the route returned by the route planner, it takes its value, otherwise it remains
        # empty
        direction = ""
        if len(userdata.route) > 2:
            direction = userdata.route[LANDMARK_TYPE_DIRECTION]

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        # if it is the first pass in the state, the target has not been pointed at yet
        if not SelectLandmark.target_pointed:
            SelectLandmark.target_pointed = True
            # the target is defined as the next landmark to point at, it's written in the userdata
            userdata.landmark_to_point = (userdata.target_frame, LANDMARK_TYPE_TARGET)
            # if the target has been returned by the pointing planner
            if userdata.target_frame in userdata.landmarks_to_point:
                return 'point_target'
            # if the pointing planner decided that the target is not visible (no target in landmarks_to_point)
            else:
                return 'point_not_visible'
        # second pass in the state, the direction has to be pointed
        elif direction in userdata.landmarks_to_point:
            userdata.landmark_to_point = (direction, LANDMARK_TYPE_DIRECTION)
            return 'point_direction'
        else:
            rospy.logwarn("There is no direction to point at")
            return 'aborted'


class PointNotVisible(smach.State):
    """Check if the not visible landmark to point has a tf frame. If so, it will point at its direction.

    If not, it will announces to the human that it will show the passage (direction - pointing planner vocab) to take
    to go there.

    Finally, it goes back to init pose

    There are 3 possible outcomes:

    - succeeded: the pointing succeeded or there is no pointing to do
    - preempted: the state has been preempted
    - aborted: the pointing failed
    """

    def __init__(self):
        """Constructor for PointNotVisible state

        It calls the super constructor of L{smach.State} and defines the outcomes and the input_keys.
        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['landmark_to_point', 'human_look_at_point'])
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Performs the pointing in the landmark direction if it is possible, otherwise announces it will show the
        passage """
        GuidingAction.feedback.current_step = "Point at not visible landmark"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        target_name = GuidingAction.services_proxy["get_individual_info"]("getName",
                                                                          userdata.landmark_to_point[LANDMARK_NAME])

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        # if the landmark has a tf frame
        if self._tfBuffer.can_transform('map', userdata.landmark_to_point[LANDMARK_NAME], rospy.Time(0)):
            # if it exists a verbalized name in the ontology
            if target_name.code == 0:
                try:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                        target_name.value +
                                                        "is nearby. It is not visible but it is in this direction,",
                                                        SPEECH_PRIORITY)
                except rospy.ServiceException, e:
                    rospy.logerr("speech exception")
            else:
                try:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                        userdata.landmark_to_point[LANDMARK_NAME] +
                                                        "is nearby. It is not visible but it is in this direction,",
                                                        SPEECH_PRIORITY)
                except rospy.ServiceException, e:
                    rospy.logerr("speech exception")

            point_at_request = PointAtRequest()
            point_at_request.point.header.frame_id = userdata.landmark_to_point[LANDMARK_NAME]
            point_at = GuidingAction.services_proxy["point_at"](point_at_request)

            rospy.sleep(POINTING_DURATION)
            GuidingAction.stand_pose()

            if point_at.success:
                return 'succeeded'
            else:
                return 'aborted'
        else:
            # try:
            #     GuidingAction.services_proxy["say"](userdata.human_look_at_point,
            #                                         "I am going to show the passage you have to take",
            #                                         SPEECH_PRIORITY)
            # except rospy.ServiceException, e:
            #     rospy.logerr("speech exception")
            rospy.logwarn(userdata.landmark_to_point[LANDMARK_NAME]+" has no tf transform")
            return 'succeeded'


class PointAndLookAtLandmark(smach.State):
    """First, points at the landmark, says to the human the landmark is there (different speech
    according to the different cases - same area or not for the target, or direction), then looks at the landmark and
    finally goes back to init pose.

    There are 3 possible outcomes:

    - succeeded: the pointing and the looking succeeded
    - preempted: the state has been preempted
    - aborted: either the pointing or the looking failed
    """

    def __init__(self):
        """Constructor for PointAndLookAtLandmark state

        It calls the super constructor of L{smach.State} and defines the outcomes and the input_keys.
        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['target_frame', 'landmark_to_point', 'human_look_at_point', 'route'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Point, speak, look, init pose"""
        GuidingAction.feedback.current_step = "Look at the landmark"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        target_name = GuidingAction.services_proxy["get_individual_info"]("getName",
                                                                          userdata.landmark_to_point[LANDMARK_NAME])
        if target_name.code != 0:
            target_name.value = userdata.landmark_to_point[LANDMARK_NAME]

        speech = ""
        # if the direction does not exist (target in same area)
        if userdata.landmark_to_point[LANDMARK_TYPE] == LANDMARK_TYPE_TARGET and len(userdata.route) == 1:
            speech = "Look, " + target_name.value + "is here."
        # if both exists (target in a different area)
        elif userdata.landmark_to_point[LANDMARK_TYPE] == LANDMARK_TYPE_TARGET:
            speech = "Look, " + target_name.value + "is over there."
        # point the direction if it exists, no matter the existence of the target
        elif userdata.landmark_to_point[LANDMARK_TYPE] == LANDMARK_TYPE_DIRECTION:
            speech = "You need to go through the " + target_name.value + " here"

        try:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point, speech, SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr("speech exception")

        coord_signal = CoordinationSignal()
        coord_signal.header.frame_id = userdata.landmark_to_point[LANDMARK_NAME]
        target = TargetWithExpiration()
        target.target.header.frame_id = userdata.landmark_to_point[LANDMARK_NAME]
        target.duration = rospy.Duration(3.0)
        coord_signal.targets.append(target)
        coord_signal.priority = 100
        coord_signal.expiration = rospy.Time() + rospy.Duration(3.0)
        # coord_signal.regex_end_condition = "isPerceiving\(robot," + userdata.landmark_to_point[LANDMARK_NAME] + "\)"
        # coord_signal.predicate = "isWaiting(robot," + userdata.landmark_to_point[LANDMARK_NAME] + ")"
        GuidingAction.coord_signals_publisher.publish(coord_signal)

        GuidingAction.feedback.current_step = "Point at the landmark"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        point_at_request = PointAtRequest()
        point_at_request.point.header.frame_id = userdata.landmark_to_point[LANDMARK_NAME]
        point_at = GuidingAction.services_proxy["point_at"](point_at_request)

        rospy.sleep(POINTING_DURATION)
        # look_at_request = LookAtRequest()
        # look_at_request.point.header.frame_id = userdata.landmark_to_point[LANDMARK_NAME]
        # look_at = GuidingAction.services_proxy["look_at"](look_at_request)

        GuidingAction.stand_pose()

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        if point_at.success:
            return 'succeeded'
        else:
            return 'aborted'

        # if look_at.success and point_at.success:
        #     return 'succeeded'
        # else:
        #     return 'aborted'


class LookAtHuman(smach.State):
    """Look at the human

    There are 3 possible outcomes:

    - succeeded: the looking succeeded
    - preempted: the state has been preempted
    - aborted: the looking failed
    """

    def __init__(self):
        """Constructor for LookAtHuman state

        It calls the super constructor of L{smach.State} and defines the outcomes and the input_keys.
        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=['person_frame'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Looks at the human"""
        coord_signal = CoordinationSignal()
        coord_signal.header.frame_id = userdata.person_frame
        target = TargetWithExpiration()
        target.target.header.frame_id = userdata.person_frame
        target.duration = rospy.Duration(2.0)
        coord_signal.targets.append(target)
        coord_signal.priority = 100
        coord_signal.expiration = rospy.Time() + rospy.Duration(3.0)
        coord_signal.regex_end_condition = "isPerceiving\(robot," + userdata.person_frame + "\)"
        coord_signal.predicate = "isWaiting(robot," + userdata.person_frame + ")"
        GuidingAction.coord_signals_publisher.publish(coord_signal)
        # GuidingAction.feedback.current_step = "Look at the human"
        # GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        # look_at_request = LookAtRequest()
        # look_at_request.point.header.frame_id = userdata.person_frame
        # look_at = GuidingAction.services_proxy["look_at"](look_at_request)
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        return 'succeeded'

        # if look_at.success:
        #     return 'succeeded'
        # else:
        #     return 'aborted'


class AskSeen(smach.State):
    """Ask to the human that it saw the landmark

    It writes 'ask_seen' to userdata.question_asked. It will then be used by the L{DispatchYesNo} state.

    There are 2 possible outcomes:

    - succeeded: the speech succeeded
    - preempted: the state has been preempted
    """

    def __init__(self):
        """Constructor for AskSeen state

        It calls the super constructor of L{smach.State} and defines the outcomes, the input_keys and the output_key.
        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['human_look_at_point'],
                             output_keys=['question_asked'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Speak"""
        try:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Have you seen where you have to go ?",
                                                SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr("speech exception")
        GuidingAction.feedback.current_step = "Ask if the landmark has been seen"
        userdata.question_asked = 'ask_seen'
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


class AskPointAgain(smach.State):
    """Ask to the human if it should point again

    It writes 'ask_point_again' to userdata.question_asked. It will then be used by the L{DispatchYesNo} state.

    There are 2 possible outcomes:

    - succeeded: the speech succeeded
    - preempted: the state has been preempted
    """

    def __init__(self):
        """Constructor for AskPointAgain state

        It calls the super constructor of L{smach.State} and defines the outcomes, the input_keys and the output_key.
        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['human_look_at_point'],
                             output_keys=['question_asked'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Speak"""
        try:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Should I show you the direction again ?",
                                                SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr("speech exception")
        GuidingAction.feedback.current_step = "Ask if the robot should show again"
        userdata.question_asked = 'ask_point_again'
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


class GetYesNo(smach_ros.SimpleActionState):
    """It calls the dialogue action server to get the answer at question asked. The expected words from the
    human are 'yes' or 'no'.

    If the human answers otherwise, he will be asked to not to.

    If the robot cannot hear what
    the human says (no result from the action server since 5 seconds (exec_timeout)), it will ask to repeat louder.
    This can be asked 2 times.

    After 3 times the robot does not hear, he gives up the interaction. If the human answers
    an unexpected word, the 'cannot hear' counter is reset.

    It writes 'yes' or 'no' to userdata.result_word.

    There are 3 possible outcomes:

    - succeeded: it got 'yes' or 'no'
    - preempted: it did not hear properly, the action server status returned 'preempted'. This outcome means that this
      state will be called again immediatly so it can have another chance to get the answer.
    - aborted: the action server failed to get the answer for the third time.
    """
    repeat_question = 0

    def __init__(self):
        """Constructor for GetYesNo state

        It calls the super constructor of L{smach.State} and defines the outcomes, the input_keys, the output_keys,
        the 3 action state callbacks, each one for the goal, the feedback and the result, a execution timeout (no answer
        from the action server since X seconds) and a server timeout.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach_ros.SimpleActionState.__init__(self, rospy.get_param("/action_servers/dialogue"),
                                             dialogue_actionAction,
                                             goal_cb=self.dialogue_goal_cb,
                                             feedback_cb=self.dialogue_feedback_cb,
                                             result_cb=self.dialogue_result_cb,
                                             input_keys=['human_look_at_point'],
                                             output_keys=['result_word'],
                                             exec_timeout=rospy.Duration(5),
                                             server_wait_timeout=rospy.Duration(1))

    def get_name(self):
        return self.__class__.__name__

    def dialogue_goal_cb(self, userdata, goal):
        # GuidingAction.services_proxy["activate_dialogue"]()
        GuidingAction.feedback.current_step = "Receive yes no answer"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        dialogue_goal = dialogue_actionGoal()
        # the action server has to catch the words 'yes' and 'no'
        dialogue_goal.subjects = ['yes', 'no']
        dialogue_goal.enable_only_subject = True
        return dialogue_goal

    def dialogue_feedback_cb(self, userdata, feedback):
        # the human said something else than yes or no, an unexpected word
        self._activate_time = rospy.Time.now()
        GuidingAction.feedback.current_step = "Human said something else than yes or no"
        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
        try:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                "I'm sorry I did not understand, you should say yes or no",
                                                SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr("speech exception")
        self._activate_time = rospy.Time.now()
        self.repeat_question = 0
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'aborted'

    def dialogue_result_cb(self, userdata, status, result):
        action_success = 'aborted'
        # the human said yes or no
        if status == actionlib.GoalStatus.SUCCEEDED:
            GuidingAction.feedback.current_step = "Human said yes or no"
            GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
            userdata.result_word = result.subject
            action_success = 'succeeded'
            self.repeat_question = 0
        # the robot did not hear what the human said
        elif status == actionlib.GoalStatus.PREEMPTED:
            # if the status is preempted because it did not hear, not because the node received a request to be killed
            if not self.preempt_requested():
                # if it asked less than 3 times to repeat
                if self.repeat_question < 2:
                    # if it is the first time to ask to repeat
                    if self.repeat_question == 0:
                        GuidingAction.feedback.current_step = "Robot did not hear for the first time"
                        GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
                        try:
                            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                                "Sorry I can't hear you, can you repeat ?",
                                                                SPEECH_PRIORITY)
                        except rospy.ServiceException, e:
                            rospy.logerr("speech exception")
                    # if it is the second time to ask to repeat
                    elif self.repeat_question == 1:
                        GuidingAction.feedback.current_step = "Robot did not hear for the second time"
                        try:
                            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                                "I am really sorry I still can not hear you, "
                                                                "can you speak louder ?",
                                                                SPEECH_PRIORITY)
                        except rospy.ServiceException, e:
                            rospy.logerr("speech exception")

                    self.repeat_question += 1
                    action_success = 'preempted'
                # it already asked 2 times to repeat
                else:
                    GuidingAction.feedback.current_step = "Robot did not hear and gave up"
                    GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
                    userdata.result_word = result.subject
                    try:
                        GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                            "I did not hear, sorry, I give up",
                                                            SPEECH_PRIORITY)
                    except rospy.ServiceException, e:
                        rospy.logerr("speech exception")
                    action_success = 'aborted'
            # if the node received a kill request
            else:
                GuidingAction.feedback.current_step = self.get_name() + " state preempted"
                GuidingAction.action_server.publish_feedback(GuidingAction.feedback)
                action_success = 'aborted'
        else:
            GuidingAction.feedback.current_step = "Dialog goal status aborted"
            GuidingAction.action_server.publish_feedback(GuidingAction.feedback)

            action_success = 'aborted'

        # GuidingAction.services_proxy["deactivate_dialogue"]()

        return action_success


class DispatchYesNo(smach.State):
    """According to the question which has been asked and the human answer which has been received,
    it returns the corresponding outcome.

    There are 3 possible outcomes:

    - show: the human want the robot to show him the place or the way to follow (for now it it the same case if the
      target is in the same area or in a different one)
    - no: the human does want the robot's help
    - preempted: the state has been preempted
    """

    def __init__(self):
        """Constructor for DispatchYesNo state

        It calls the super constructor of L{smach.State} and defines the outcomes and the input_keys.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['show', 'no', 'preempted'],
                             input_keys=['question_asked', 'result_word', 'human_look_at_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Dispatch the question-answer couples"""
        next_action = 'preempted'
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            next_action = 'preempted'
        elif userdata.question_asked == 'ask_show_target':
            if userdata.result_word == 'yes':
                next_action = 'show'
            else:
                try:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Ok, I hope you will get there!",
                                                        SPEECH_PRIORITY)
                except rospy.ServiceException, e:
                    rospy.logerr("speech exception")
                next_action = 'no'
        elif userdata.question_asked == 'ask_show_direction':
            if userdata.result_word == 'yes':
                next_action = 'show'
            else:
                try:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Ok, I hope you will get there!",
                                                        SPEECH_PRIORITY)
                except rospy.ServiceException, e:
                    rospy.logerr("speech exception")
                next_action = 'no'

        return next_action


class DispatchYesNoCL(smach.State):
    """According to the question which has been asked and the human answer which has been received,
    it returns the corresponding outcome.

    There are 5 possible outcomes:

    - yes: the human has seen the landmark
    - ask_point_again: the human has not seen the landmark, so in the next state the robot will ask if it should point
      again
    - pointing: the human agreed that the robot point again
    - no: the human does want the robot's help anymore
    - preempted: the state has been preempted
    """

    def __init__(self):
        """Constructor for DispatchYesNoCL state

        It calls the super constructor of L{smach.State} and defines the outcomes and the input_keys.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['yes', 'preempted', 'ask_point_again', 'pointing', 'no'],
                             input_keys=['question_asked', 'result_word', 'human_look_at_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Dispatch the question-answer couples"""
        next_action = 'preempted'
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            next_action = 'preempted'

        if userdata.question_asked == 'ask_seen':
            if userdata.result_word == 'yes':
                try:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Awesome !", SPEECH_PRIORITY)
                except rospy.ServiceException, e:
                    rospy.logerr("speech exception")
                next_action = 'yes'
            else:
                try:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                        "Oh it's too bad, I'm sorry I wasn't good enough", SPEECH_PRIORITY)
                except rospy.ServiceException, e:
                    rospy.logerr("speech exception")
                next_action = 'ask_point_again'

        elif userdata.question_asked == 'ask_point_again':
            if userdata.result_word == 'yes':
                next_action = 'pointing'
            else:
                GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Ok, too bad. Good luck !",
                                                    SPEECH_PRIORITY)
                next_action = 'no'

        return next_action


class IsOver(smach.State):
    """Computes if the landmarks which needed to be pointed are all pointed -> is there (still) a direction to
    point or not

    There are 4 possible outcomes:

    - succeeded: the task is over and went well
    - point_direction: the next step is to point at the direction
    - preempted: the state has been preempted
    - aborted: the task is over but met some trouble
    """

    def __init__(self):
        """Constructor for IsOver state

        It calls the super constructor of L{smach.State} and defines the outcomes and the input_keys.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'point_direction', 'preempted', 'aborted'],
                             input_keys=['landmark_to_point', 'landmarks_to_point', 'last_outcome'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Computes if there is the direction to point or if the task is over"""
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        # if a target and a direction to point had been returned by the pointing planner and the direction has not been
        # pointed yet
        # or if the only landmark contained in the list returned by the pointing planner is a direction
        if (len(userdata.landmarks_to_point) == 2 and
            userdata.landmark_to_point[LANDMARK_NAME] != userdata.landmarks_to_point[LANDMARK_TYPE_DIRECTION]) \
                or \
                (userdata.landmark_to_point[LANDMARK_NAME] not in userdata.landmarks_to_point
                 and len(userdata.landmarks_to_point) == 1):
            return 'point_direction'
        # if the human does want to see the landmark again
        elif userdata.last_outcome == 'no':
            return 'aborted'
        # what had to be shown is shown properly
        else:
            return 'succeeded'


class Timer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])

    def execute(self, userdata):
        activate_time = rospy.Time.now()
        while not self.preempt_requested():
            rospy.sleep(0.1)
            if rospy.Time.now() - activate_time > rospy.Duration(OBSERVE_LANDMARK_TIMEOUT):
                return 'stop'

        return 'in_time'


class SaySeen(smach.State):
    """Say to the human that it saw him look at the landmark

    There are 2 possible outcomes:
    - succeeded: the speech succeeded
    - preempted: the state has been preempted
    """

    def __init__(self):
        """Constructor for SaySeen state

        It calls the super constructor of L{smach.State} and defines the outcomes and the input_keys.
        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['human_look_at_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Speak"""
        try:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                "I saw that you have seen where you have to go. Awesome !",
                                                SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr("speech exception")
        GuidingAction.feedback.current_step = "Say it saw the human saw the way"
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


class HumanLost(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['task_failed'], input_keys=['human_look_at_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        try:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                "I am sorry, I am not able to see you anymore. Bye bye.",
                                                SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr("speech exception")

        return 'task_failed'


class Failure(smach.State):
    """Allow to print a message or perform an action if the task failed.
    """

    def __init__(self):
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['aborted'], input_keys=['last_state'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        return 'aborted'


if __name__ == '__main__':
    rospy.init_node('guiding_action_server')
    server = GuidingAction(rospy.get_name())
    rospy.on_shutdown(server.stand_pose)
    rospy.spin()

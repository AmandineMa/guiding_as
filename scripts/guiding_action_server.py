#! /usr/bin/env python

from enum import Enum
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
import guiding_as.msg
from dialogue_as.msg import *
from deictic_gestures.srv import *
from mummer_integration_msgs.srv import *
from semantic_route_description.srv import *
from perspectives_msgs.srv import *
from ontologenius.srv import *
from route_description.srv import *
from speech_wrapper.srv import *
from multimodal_human_monitor.srv import *

# constants
ROBOT_PLACE = "robot_infodesk"
PERSONNA = "lambda"
SPEECH_PRIORITY = 100
POINTING_DURATION = 0.5
WORLD = "base"
STOP_TRACK_DIST_TH = 0.3
SHOULD_MOVE_DIST_TH = 0.5
TAKE_ROBOT_PLACE_DIST_TH = 0.6

# global variable
human_perceived = False


class ShowAction(object):
    _result = guiding_as.msg.taskResult()

    feedback = guiding_as.msg.taskFeedback()
    action_server = None
    dialogue_client = None
    services_proxy = {}

    def __init__(self, name):
        self._action_name = name

        ShowAction.action_server = actionlib.SimpleActionServer(self._action_name, guiding_as.msg.taskAction,
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
        activate_dialogue_srv = rospy.get_param('/services/activate_dialogue')
        deactivate_dialogue_srv = rospy.get_param('/services/deactivate_dialogue')

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

        rospy.loginfo("waiting for service" + point_at_srv)
        rospy.wait_for_service(point_at_srv)

        rospy.loginfo("waiting for service" + monitor_humans_srv)
        rospy.wait_for_service(monitor_humans_srv)

        # rospy.loginfo("waiting for service " + activate_dialogue_srv)
        # rospy.wait_for_service(activate_dialogue_srv)
        #
        # rospy.loginfo("waiting for service " + deactivate_dialogue_srv)
        # rospy.wait_for_service(deactivate_dialogue_srv)

        ShowAction.services_proxy = {
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
            "monitor_humans": rospy.ServiceProxy(monitor_humans_srv, MonitorHumans)}
            # "activate_dialogue": rospy.ServiceProxy(activate_dialogue_srv, Trigger),
            # "deactivate_dialogue": rospy.ServiceProxy(deactivate_dialogue_srv, Trigger)}

        rospy.loginfo("start action server")
        ShowAction.action_server.start()
        rospy.loginfo("action server started")

        # ShowAction.services_proxy["deactivate_dialogue"]()

    # ------ Callbacks to handle the active_state variable ------ #

    def guiding_start_cb(self, userdata, initial_states, *cb_args):
        userdata.active_state = initial_states[0]
        userdata.previous_state = None

    def guiding_transition_cb(self, userdata, active_states, *cb_args):
        userdata.previous_state = userdata.active_state
        if active_states[0] != 'Failure':
            userdata.active_state = active_states[0]
        userdata.last_outcome = cb_args[0].get_last_outcome()

    def term_cb(self, userdata, terminal_states, *cb_args):
        if terminal_states[0] == 'Failure':
            self._result.failure_reason = userdata.active_state
            ShowAction.action_server.set_succeeded(self._result)

    # ------ Callbacks to handle the termination and the outcome of the HumanTracking container ------ #

    def human_tracking_term_cb(self, outcome_map):
        # Terminate the concurrent container HumanTracking if the human is detected at the pose his supposed to reach
        if (outcome_map['StopTrackingCondition'] == 'continue_tracking' and outcome_map[
            'LookAtHumanTrack'] == 'succeeded') \
                or outcome_map['StopTrackingCondition'] == 'succeeded' or outcome_map['HumanMonitor'] == 'invalid':
            return True
        else:
            return False

    def human_tracking_out_cb(self, outcome_map):
        # if 'aborted' in outcome_map.values():
        #     return 'aborted'
        if outcome_map['HumanMonitor'] == 'invalid':
            return 'look_final_dest'
        if outcome_map['StopTrackingCondition'] == 'succeeded':
            return 'succeeded'
        elif outcome_map['StopTrackingCondition'] == 'continue_tracking' and \
                outcome_map['LookAtHumanTrack'] == 'succeeded':
            return 'continue_to_look'

    # ------ Callback to handle the monitoring of if the human is perceived or not ------ #
    def human_perceive_monitor_cb(self, userdata, msg):
        global human_perceived
        perceived = False
        for fact in msg.facts:
            if fact.predicate == "isPerceiving" and fact.object_name == userdata.person_frame and fact.subject_name == 'robot':
                perceived = True

        if perceived:
            human_perceived = True
        else:
            human_perceived = False

            return True

    count = 0

    def human_track_perceive_monitor_cb(self, userdata, msg):
        perceived = False
        for fact in msg.facts:
            if fact.predicate == "isPerceiving" and fact.object_name == userdata.person_frame and fact.subject_name == 'robot':
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

    # ------ Callbacks for the CONCURRENCE container (HUMAN_MONITOR // GUIDING) ------ #

    def h_m_c_child_term_cb(self, outcome_map):
        if outcome_map['GUIDING'] == 'task_succeeded' or outcome_map['GUIDING'] == 'task_failed' \
                or outcome_map['GUIDING'] == 'preempted':
            return True
        else:
            return False

    def h_m_c_out_cb(self, outcome_map):
        if outcome_map['GUIDING'] == 'task_succeeded':
            return 'task_succeeded'
        else:
            return 'task_failed'

    def execute_cb(self, goal):

        ShowAction.feedback.current_step = "Start state machine"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)

        # Build Guiding Container
        guiding_sm = smach.StateMachine(outcomes=['task_succeeded', 'task_failed', 'preempted'],
                                        input_keys=['person_frame'],
                                        output_keys=['active_state'])

        # guiding_sm.userdata.person_frame = goal.person_frame
        guiding_sm.userdata.place_frame = goal.place_frame

        # Add States of Guiding Container
        with guiding_sm:

            smach.StateMachine.add('LookAtHuman1', LookAtHuman(),
                                   transitions={'succeeded': 'GetRouteRegion', 'aborted': 'show_failed',
                                                'preempted': 'preempted'})

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
                                         input_keys=['place_frame', 'person_frame', 'human_look_at_point', 'route',
                                                     'active_state', 'last_outcome'],
                                         output_keys=['active_state'])

            with show_sm:
                smach.StateMachine.add('PointingConfig', PointingConfig(),
                                       transitions={'succeeded': 'ShouldHumanMove', 'aborted': 'show_failed',
                                                    'preempted': 'preempted', 'point_not_visible': 'SelectLandmark'})

                smach.StateMachine.add('ShouldHumanMove', ShouldHumanMove(),
                                       transitions={'human_first': 'PointAndLookAtHumanFuturePlace', 'no': 'MoveToPose',
                                                    'robot_first': 'AskHumanToMoveAfter', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                smach.StateMachine.add('AskHumanToMoveAfter', AskHumanToMoveAfter(),
                                       transitions={'succeeded': 'MoveToPose', 'preempted': 'preempted'})

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
                    smach.Concurrence.add('HumanMonitor',
                                          smach_ros.MonitorState("/base/current_facts", FactArrayStamped,
                                                                 self.human_track_perceive_monitor_cb,
                                                                 input_keys=['person_frame']))

                smach.StateMachine.add('HumanTracking', human_tracking_concurrence,
                                       transitions={'succeeded': 'MoveToPose', 'continue_to_look': 'HumanTracking',
                                                    'look_final_dest': 'LookAtHumanAssumedPlace1',
                                                    'preempted': 'show_failed', 'aborted': 'show_failed'})

                smach.StateMachine.add('LookAtHumanAssumedPlace1', LookAtHumanAssumedPlace(),
                                       transitions={'succeeded': 'MoveToPose', 'preempted': 'preempted',
                                                    'aborted': 'show_failed', 'look_again': 'LookAtHumanAssumedPlace1'})

                smach.StateMachine.add('MoveToPose', MoveToPose(),
                                       transitions={'succeeded': 'LookAtHumanAssumedPlace2',
                                                    'preempted': 'preempted', 'aborted': 'show_failed'})

                smach.StateMachine.add('LookAtHumanAssumedPlace2', LookAtHumanAssumedPlace(),
                                       transitions={'succeeded': 'SelectLandmark', 'preempted': 'preempted',
                                                    'aborted': 'show_failed', 'look_again': 'LookAtHumanAssumedPlace2'})

                smach.StateMachine.add('SelectLandmark', SelectLandmark(),
                                       transitions={'point_invisible': 'PointNotVisible',
                                                    'point_direction': 'PointAndLookAtLandmark',
                                                    'point_target': 'PointAndLookAtLandmark',
                                                    'aborted': 'show_failed', 'preempted': 'preempted'})

                smach.StateMachine.add('PointNotVisible', PointNotVisible(),
                                       transitions={'succeeded': 'IsOver', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                smach.StateMachine.add('PointAndLookAtLandmark', PointAndLookAtLandmark(),
                                       transitions={'succeeded': 'LookAtHuman2', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                smach.StateMachine.add('LookAtHuman2', LookAtHuman(),
                                       transitions={'succeeded': 'CheckLandmarkSeen', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                # Container to ask the human if he sees the landmark (case where the robot did not observe it)
                check_landmark_seen_sm = smach.StateMachine(outcomes=['yes', 'no', 'pointing', 'preempted', 'failure'],
                                                            input_keys=['human_look_at_point', 'active_state',
                                                                        'previous_state', 'last_outcome'])

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

        guiding_sm.register_transition_cb(self.guiding_transition_cb, cb_args=[guiding_sm])
        guiding_sm.register_start_cb(self.guiding_start_cb, cb_args=[])
        guiding_sm.register_termination_cb(self.term_cb, cb_args=[])
        show_sm.register_start_cb(self.guiding_start_cb, cb_args=[])
        show_sm.register_transition_cb(self.guiding_start_cb, cb_args=[show_sm])
        check_landmark_seen_sm.register_transition_cb(self.guiding_transition_cb, cb_args=[check_landmark_seen_sm])

        human_monitor_concurrence = smach.Concurrence(outcomes=['task_succeeded', 'task_failed', 'preempted'],
                                                      default_outcome='task_failed',
                                                      outcome_cb=self.h_m_c_out_cb,
                                                      input_keys=['person_frame'],
                                                      child_termination_cb=self.h_m_c_child_term_cb)
        with human_monitor_concurrence:
            smach.Concurrence.add('GUIDING', guiding_sm)
            smach.Concurrence.add('HUMAN_MONITOR', smach_ros.MonitorState(rospy.get_param("/topics/current_facts"),
                                                                          FactArrayStamped,
                                                                          self.human_perceive_monitor_cb,
                                                                          input_keys=['person_frame']))

        top_sm = smach.StateMachine(outcomes=['task_succeeded', 'task_failed', 'preempted'])
        top_sm.userdata.person_frame = goal.person_frame

        look_at_point = PointStamped()
        look_at_point.header.frame_id = top_sm.userdata.person_frame
        guiding_sm.userdata.human_look_at_point = look_at_point
        guiding_sm.userdata.route = None

        with top_sm:
            smach.StateMachine.add('CONCURRENCE', human_monitor_concurrence)

        monitor_humans_request = MonitorHumansRequest()
        monitor_humans_request.action = "ADD"
        monitor_humans_request.humans_to_monitor = [goal.person_frame]
        ShowAction.services_proxy["monitor_humans"](monitor_humans_request)

        sis = smach_ros.IntrospectionServer('server_name', top_sm, '/SM_ROOT')
        sis.start()
        smach_ros.set_preempt_handler(top_sm)
        outcome = top_sm.execute()

        # monitor_humans_request.action = "REMOVE"
        # ShowAction.services_proxy["monitor_humans"](monitor_humans_request)

        # Before shutdown
        ShowAction.feedback.current_step = "Go to stand pose"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        self.stand_pose()

        rospy.loginfo("State machine outcome : %s ", outcome)
        if outcome == 'task_succeeded':
            self._result.success = True
            self._result.failure_reason = ''
        else:
            self._result.success = False
            if guiding_sm.userdata.active_state is None:
                self._result.failure_reason = "SM did not start"
            else:
                self._result.failure_reason = guiding_sm.userdata.active_state

        ShowAction.action_server.set_succeeded(self._result)

        sis.stop()

    @staticmethod
    def stand_pose():
        rospy.loginfo("stand pose")
        go_to_posture_request = GoToPostureRequest()
        go_to_posture_request.posture_name = "StandInit"
        go_to_posture_request.speed = 1
        ShowAction.services_proxy["stand_pose"](go_to_posture_request)


class GetRouteRegion(smach.State):
    """GetRouteRegion

    Write to userdata.route the best route to go from ROBOT_PLACE to the place asked by the human.

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
        rospy.loginfo("Initialization of GetRouteRegion state")
        smach.State.__init__(self, outcomes=['in_region', 'out_region', 'unknown', 'preempted', 'aborted'],
                             input_keys=['place_frame', 'human_look_at_point'],
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

    def execute(self, userdata):
        """Calls the route planner which returns a list of possible routes. Then select the best one among those.
        If the list of route is empty, it means the place is unknown.
        If the list of route has only one element, it means the target place is in the same region than the robot.
        If the list of route has more than one element, it means the target place is in a different region.
        """
        ShowAction.feedback.current_step = "get_route_region"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)

        if self.preempt_requested():
            rospy.loginfo("GetRouteRegion preempted")
            self.service_preempt()
            return 'preempted'

        try:
            get_route_region = ShowAction.services_proxy["get_route_region"](
                ROBOT_PLACE,
                userdata.place_frame,
                PERSONNA,
                True)

        except rospy.ServiceException, e:
            return 'aborted'

        # if the routes list returned by the route planner is not empty
        if len(get_route_region.routes) != 0:
            userdata.route = self._select_best_route(get_route_region.routes, get_route_region.costs).route
        else:
            userdata.route = []

        # if the routes list returned by the route planner is empty, the place is unknown
        if len(userdata.route) == 0:
            ShowAction.services_proxy["say"](userdata.human_look_at_point, "I'm sorry, I don't know this place",
                                             SPEECH_PRIORITY)
            return 'unknown'

        elif len(userdata.route) == 1:
            return 'in_region'
        else:
            return 'out_region'


class AskShowPlace(smach.State):
    """AskShowPlace - case where the robot is in the same region than the place
    Ask to the human : PLACE_NAME is nearby. Would you like me to show you the shop ?
    Then, write to userdata.question_asked "ask_show_place", it will be used by the state L{DispatchYesNo}.

    There are 2 possible outcomes:
    - succeeded
    - preempted: the state has been preempted
    """

    def __init__(self):
        """Constructor for AskShowPlace state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of AskShowPlace state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['place_frame', 'human_look_at_point'],
                             output_keys=['question_asked'])

    def execute(self, userdata):
        """Calls the speech service to ask the question"""
        ShowAction.feedback.current_step = "Ask if the robot should show the shop in region"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)

        if self.preempt_requested():
            rospy.loginfo("AskShowPlace preempted")
            self.service_preempt()
            return 'preempted'

        place_name = ShowAction.services_proxy["get_individual_info"]("getName", userdata.place_frame)
        # If a verbalized/label name exists in the ontology, it is used
        if place_name.code == 0:
            ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                             place_name.value + "is nearby. Would you like me to show you the shop ?",
                                             SPEECH_PRIORITY)
        # If there is not, the 'technical' name is used
        else:
            ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                             userdata.place_frame + "is nearby. Would you like me to show you the shop ?",
                                             SPEECH_PRIORITY)

        userdata.question_asked = 'ask_show_place'
        return 'succeeded'


class AskShowDirection(smach.State):
    """AskShowDirection - case where the robot is not in the same region than the place
    First, it calls a service to get the verbalized description of the route.
    Then, it asks to the human : It's not here. You need to ROUTE_DESCRIPTION.
                                Would you like me to show you the way to go to the shop ?
    Finally, it writes to userdata.question_asked "ask_show_direction", it will be used by the state L{DispatchYesNo}.

    There are 2 possible outcomes:
    - succeeded
    - preempted: the state has been preempted
    """

    def __init__(self):
        """Constructor for AskShowDirection state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of AskShowDirection state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['place_frame', 'human_look_at_point'],
                             output_keys=['question_asked'])

    def execute(self, userdata):
        """Calls the speech service to ask the question"""

        ShowAction.feedback.current_step = "Ask if the robot should show the direction"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        if self.preempt_requested():
            rospy.loginfo("AskShowDirection preempted")
            self.service_preempt()
            return 'preempted'

        # call the route description service
        route_description = \
            ShowAction.services_proxy["get_route_description"](ROBOT_PLACE, userdata.place_frame).region_route

        ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                         "It's not here. You need to " + route_description +
                                         "Would you like me to show you the way to go to the shop ?",
                                         SPEECH_PRIORITY)

        userdata.question_asked = 'ask_show_direction'
        return 'succeeded'


class PointingConfig(smach.State):
    """PointingConfig - calls the pointing planner to get the best configuration (pose) for the human and the robot
    It extracts the first passage from the route returned by the route planner (route[1])
    and write it in a variable called direction. If len(route) == 1, direction is empty.

    The pointing planner takes in parameters: the place, the direction  and the human tf frame. Before calling the
    pointing planner, it checks if there is an associated mesh to the place. If there is one, it calls the pointing
    planner.

    The pointing planner returns a pose each for the robot and one for the human.
    It also returns a list of string that contains the landmarks which will be visible from this configuration.
    - If len(landmarks_to_point) == 0, the pointing planner failed
    - If landmarks_to_point = [PLACE] -> only the place will be visible (normally, this case happens only when the robot
    is in the same region than the place because the pointing planner defined the visibility of the direction as a
    priority compared to the visibility of the place)
    - If landmarks_to_point = [DIRECTION] -> only the direction will be visible
    - If landmarks_to_point = [PLACE, DIRECTION] -> both will be visible

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
        rospy.loginfo("Initialization of PointingConfig state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'point_not_visible'],
                             input_keys=['place_frame', 'person_frame', 'route', 'human_look_at_point'],
                             output_keys=['target_pose', 'landmarks_to_point', 'human_pose'])

    def execute(self, userdata):
        """Does some tests, then calls the pointing planner service"""
        ShowAction.feedback.current_step = "get_pointing_config"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)

        direction_landmark = ""
        place_frame = ""
        # if there is a direction landmark in the route list returned by the route planner
        if len(userdata.route) > 2:
            direction_landmark = userdata.route[1]

        # Check if it exists an associated mesh to the place
        has_mesh_result = ShowAction.services_proxy["has_mesh"](WORLD, userdata.place_frame)
        if not has_mesh_result.success:
            rospy.logerr("has_mesh service failed")
        else:
            # if there is no mesh, the place_frame remains empty
            if has_mesh_result.has_mesh:
                place_frame = userdata.place_frame

        try:
            # call the pointing planner service
            get_pointing_config = ShowAction.services_proxy["get_pointing_config"](
                place_frame,
                direction_landmark,
                userdata.person_frame)

            # write in the userdata
            userdata.target_pose = get_pointing_config.robot_pose
            userdata.human_pose = get_pointing_config.human_pose
            userdata.landmarks_to_point = get_pointing_config.pointed_landmarks

            if self.preempt_requested():
                rospy.loginfo("PointingConfig preempted")
                self.service_preempt()
                return 'preempted'

            if len(get_pointing_config.pointed_landmarks) == 0:
                return 'point_not_visible'
            else:
                return 'succeeded'

        except rospy.ServiceException, e:
            rospy.logerr('pointing planner exception')
            ShowAction.services_proxy["say"](userdata.human_look_at_point, "I am sorry, my pointing planner crashed !",
                                             SPEECH_PRIORITY)
            return 'aborted'


class ShouldHumanMove(smach.State):
    """"ShouldHumanMove - Based on the human pose returned by the pointing planner, it evaluates if the robot should
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
        rospy.loginfo("Initialization of ShouldHumanMove state")
        smach.State.__init__(self, outcomes=['robot_first', 'human_first', 'no', 'preempted', 'aborted'],
                             input_keys=['person_frame', 'human_pose'])
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def execute(self, userdata):
        """Calculates the distance between the actual human position and the one returned by the pointing planner,
        the distance between the actual robot position and the human's one returned by the pointing planner, then
        estimates the outcome to return."""
        ShowAction.feedback.current_step = "Determine if the human should move or not"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        try:
            # calculate the distance between the human actual pose and the human wanted pose
            trans = self._tfBuffer.lookup_transform('map', userdata.person_frame, rospy.Time())
            distance_h_now_h_future = math.sqrt(
                (trans.transform.translation.x - userdata.human_pose.pose.position.x) ** 2 +
                (trans.transform.translation.y - userdata.human_pose.pose.position.y) ** 2)
            rospy.logwarn("dist human now future : %f", distance_h_now_h_future)

            if self.preempt_requested():
                rospy.loginfo("ShouldHumanMove preempted")
                self.service_preempt()
                return 'preempted'

            # if the distance is superior to the threshold, the human will be asked to move
            if distance_h_now_h_future > SHOULD_MOVE_DIST_TH:
                try:
                    # calculate the distance between the robot actual pose and the human wanted pose
                    trans = self._tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
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
    """AskShowDirection - case where the robot has to move first. It asks the human to come to its current place.

    There are 2 possible outcomes:
    - succeeded
    - preempted: the state has been preempted
    """
    def __init__(self):
        """Constructor for AskHumanToMoveAfter state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of AskSeen state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['human_look_at_point'])

    def execute(self, userdata):
        """Calls the speech service to ask the human to move"""
        ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                         "I am going to move, so you can come to my current place."
                                         "You will better see from here.",
                                         SPEECH_PRIORITY)
        ShowAction.feedback.current_step = "Ask human to move after"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        if self.preempt_requested():
            rospy.loginfo("AskHumanToMoveAfter preempted")
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


class PointAndLookAtHumanFuturePlace(smach.State):
    """PointAtHumanFuturePlace - Announces to the human that he is supposed to come to its actual place.
    Then, point and look at where the human was supposed to go (with z = 0) so head and arm pointing at the floor.

    There are 3 possible outcomes:
    - succeeded: it went well
    - preempted: the state has been preempted
    - aborted: the look at or the point at services failed

    """
    def __init__(self):
        rospy.loginfo("Initialization of PointAndLookAtHumanFuturePlace state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['landmark_to_point', 'human_look_at_point', 'human_pose'])

    def execute(self, userdata):
        ShowAction.feedback.current_step = "Point at where the human should go"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                         "I need you to make a few steps.... You will see better "
                                         "what I am about to show you. Can you go there ?",
                                         SPEECH_PRIORITY)

        point_at_request = PointAtRequest()
        point_at_request.point.header = userdata.human_pose.header
        point_at_request.point.point = userdata.human_pose.pose.position
        point_at = ShowAction.services_proxy["point_at"](point_at_request)

        look_at_request = LookAtRequest()
        look_at_request.point.header = userdata.human_pose.header
        look_at_request.point.point = userdata.human_pose.pose.position
        look_at = ShowAction.services_proxy["look_at"](look_at_request)
        rospy.sleep(POINTING_DURATION)

        if self.preempt_requested():
            rospy.loginfo("PointAndLookAtHumanFuturePlace preempted")
            self.service_preempt()
            return 'preempted'

        if point_at.success and look_at.success:
            return 'succeeded'
        else:
            return 'aborted'


class LookAtHumanAssumedPlace(smach.State):
    _does_not_see = 0

    def __init__(self):
        rospy.loginfo("Initialization of LookAtPlace state")
        smach.State.__init__(self, outcomes=['succeeded', 'look_again', 'preempted', 'aborted'],
                             input_keys=['human_pose', 'human_look_at_point'])

    def execute(self, userdata):
        global human_perceived

        if self.preempt_requested():
            rospy.loginfo("LookAtHuman preempted")
            self.service_preempt()
            return 'preempted'

        look_at_request = LookAtRequest()
        # deepcopy allows to copy human_pose so when look_at_request.point is modified, it does not affect human_pose
        look_at_request.point.header.frame_id = copy.deepcopy(userdata.human_pose.header.frame_id)
        look_at_request.point.point = copy.deepcopy(userdata.human_pose.pose.position)
        # so the robot look at an averaged human size
        look_at_request.point.point.z += 1.5
        look_at = ShowAction.services_proxy["look_at"](look_at_request)

        rospy.logwarn(human_perceived)
        # if the human is not perceived for less than 3 times
        if not human_perceived and self._does_not_see < 2:
            ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                             "I am sorry, I cannot see you. Can you come in front of me ?",
                                             SPEECH_PRIORITY)
            self._does_not_see += 1
            # wait a bit before checking again
            rospy.sleep(2.0)
            return 'look_again'
        # if the human is still not perceived
        # (It can happen if the human is in front of the robot but its ID changed - Maybe the interaction should
        # stop instead of still being going on)
        elif not human_perceived:
            ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                             "I still cannot see you but I will assume that you are here",
                                             SPEECH_PRIORITY)
        self._does_not_see = 0

        if look_at.success:
            return 'succeeded'
        else:
            rospy.logerr('look at failed')
            return 'aborted'


class StopTrackingCondition(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of StopTrackingCondition state")
        smach.State.__init__(self, outcomes=['continue_tracking', 'succeeded', 'preempted'],
                             input_keys=['human_pose', 'person_frame'])
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def execute(self, userdata):
        ShowAction.feedback.current_step = "stop tracking condition"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)

        if self.preempt_requested():
            rospy.loginfo("StopTrackingCondition preempted")
            self.service_preempt()
            return 'preempted'

        # Calculate the distance between the human actual pose/frame and the human final/wanted pose
        trans = self._tfBuffer.lookup_transform('map',
                                                userdata.person_frame, rospy.Time())
        distance = math.sqrt((trans.transform.translation.x - userdata.human_pose.pose.position.x) ** 2 +
                             (trans.transform.translation.y - userdata.human_pose.pose.position.y) ** 2)
        # if the wanted pose is reached (+/- a certain threshold)
        if distance < STOP_TRACK_DIST_TH:
            return 'succeeded'
        else:
            return 'continue_tracking'


class MoveToPose(smach_ros.SimpleActionState):
    def __init__(self):
        rospy.loginfo("Initialization of MoveToPose state")
        smach_ros.SimpleActionState.__init__(self, rospy.get_param("/action_servers/move_to"),
                                             MoveBaseAction, goal_cb=self.move_to_goal_cb,
                                             feedback_cb=self.move_to_feedback_cb,
                                             result_cb=self.move_to_result_cb,
                                             input_keys=['target_pose', 'human_look_at_point'],
                                             server_wait_timeout=rospy.Duration(2)),

    def move_to_goal_cb(self, userdata, goal):
        ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                         "Now wait, I am going to move.",
                                         SPEECH_PRIORITY)
        ShowAction.feedback.current_step = "moving"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        move_to_goal = MoveBaseGoal()
        move_to_goal.target_pose = userdata.target_pose
        return move_to_goal

    def move_to_feedback_cb(self, userdata, feedback):
        # rospy.loginfo(feedback.base_position)
        pass

    def move_to_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        else:
            return 'aborted'


# class LandmarkType(Enum):
LANDMARK_TYPE_TARGET = 0
LANDMARK_TYPE_DIRECTION = 1

LANDMARK_NAME = 0
LANDMARK_TYPE = 1


class SelectLandmark(smach.State):
    _place_pointed = False

    def __init__(self):
        rospy.loginfo("Initialization of SelectLandmark state")
        smach.State.__init__(self, outcomes=['point_invisible', 'point_direction',
                                             'point_target', 'preempted', 'aborted'],
                             input_keys=['place_frame', 'route', 'landmarks_to_point', 'human_look_at_point'],
                             output_keys=['landmark_to_point'])

    def execute(self, userdata):
        direction = ""
        if len(userdata.route) > 2:
            direction = userdata.route[LANDMARK_TYPE_DIRECTION]

        if self.preempt_requested():
            rospy.loginfo("SelectLandmark preempted")
            self.service_preempt()
            return 'preempted'

        if not self._place_pointed:
            self._place_pointed = True
            userdata.landmark_to_point = (userdata.place_frame, LANDMARK_TYPE_TARGET)
            if userdata.place_frame in userdata.landmarks_to_point:
                return 'point_target'
            else:
                return 'point_invisible'
        elif direction in userdata.landmarks_to_point:
            userdata.landmark_to_point = (direction, LANDMARK_TYPE_DIRECTION)
            return 'point_direction'
        else:
            userdata.landmark_to_point = ()
            ShowAction.services_proxy["say"](userdata.human_look_at_point, "I am sorry, I am not able to show you.",
                                             SPEECH_PRIORITY)
            return 'aborted'


class PointNotVisible(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of PointNotVisible state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['landmark_to_point', 'human_look_at_point'])
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def execute(self, userdata):
        ShowAction.feedback.current_step = "Point at not visible landmark"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        place_name = ShowAction.services_proxy["get_individual_info"]("getName",
                                                                      userdata.landmark_to_point[LANDMARK_NAME])

        if self.preempt_requested():
            rospy.loginfo("PointNotVisible preempted")
            self.service_preempt()
            return 'preempted'

        if self._tfBuffer.can_transform('map', userdata.landmark_to_point[LANDMARK_NAME], rospy.Time(0)):
            if place_name.code == 0:
                ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                                 place_name.value + "is nearby. It is not visible but it is in this direction,",
                                                 SPEECH_PRIORITY)
            else:
                ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                                 userdata.landmark_to_point[LANDMARK_NAME] +
                                                 "is nearby. It is not visible but it is in this direction,",
                                                 SPEECH_PRIORITY)

            point_at_request = PointAtRequest()
            point_at_request.point.header.frame_id = userdata.landmark_to_point[LANDMARK_NAME]
            point_at = ShowAction.services_proxy["point_at"](point_at_request)

            rospy.sleep(POINTING_DURATION)
            ShowAction.stand_pose()

            if point_at.success:
                return 'succeeded'
            else:
                return 'aborted'
        else:
            ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                             "I am going to show the passage you have to take", SPEECH_PRIORITY)
            return 'succeeded'


class PointAndLookAtLandmark(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of PointAndLookAtLandmark state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['place_frame', 'landmark_to_point', 'human_look_at_point', 'route'])

    def execute(self, userdata):
        ShowAction.feedback.current_step = "Look at the landmark"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        place_name = ShowAction.services_proxy["get_individual_info"]("getName",
                                                                      userdata.landmark_to_point[LANDMARK_NAME])
        if place_name.code != 0:
            place_name.value = userdata.landmark_to_point[LANDMARK_NAME]

        speech = ""
        # if the direction does not exist
        if userdata.landmark_to_point[LANDMARK_TYPE] == LANDMARK_TYPE_TARGET and len(userdata.route) == 1:
            speech = "Look, " + place_name.value + "is here."
        # if both exists
        elif userdata.landmark_to_point[LANDMARK_TYPE] == LANDMARK_TYPE_TARGET:
            speech = "Look, " + place_name.value + "is over there."
        # point the direction if it exists, no matter the existence of the target
        elif userdata.landmark_to_point[LANDMARK_TYPE] == LANDMARK_TYPE_DIRECTION:
            speech = "You need to go through the " + place_name.value + " here"

        ShowAction.feedback.current_step = "Point at the landmark"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        point_at_request = PointAtRequest()
        point_at_request.point.header.frame_id = userdata.landmark_to_point[LANDMARK_NAME]
        point_at = ShowAction.services_proxy["point_at"](point_at_request)
        rospy.sleep(POINTING_DURATION)

        ShowAction.services_proxy["say"](userdata.human_look_at_point, speech, SPEECH_PRIORITY)
        look_at_request = LookAtRequest()
        look_at_request.point.header.frame_id = userdata.landmark_to_point[LANDMARK_NAME]
        look_at = ShowAction.services_proxy["look_at"](look_at_request)
        if self.preempt_requested():
            rospy.loginfo("PointAndLookAtLandmark preempted")
            self.service_preempt()
            return 'preempted'

        if look_at.success:
            return 'succeeded'
        else:
            return 'aborted'


class LookAtHuman(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of LookAtHuman state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=['person_frame'])

    def execute(self, userdata):
        ShowAction.feedback.current_step = "Look at the human"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        look_at_request = LookAtRequest()
        look_at_request.point.header.frame_id = userdata.person_frame
        look_at = ShowAction.services_proxy["look_at"](look_at_request)
        if self.preempt_requested():
            rospy.loginfo("LookAtHuman preempted")
            self.service_preempt()
            return 'preempted'

        if look_at.success:
            return 'succeeded'
        else:
            return 'aborted'


class SaySeen(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of SaySeen state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['human_look_at_point'])

    def execute(self, userdata):
        ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                         "I can see you saw where you have to go, my job is done here", SPEECH_PRIORITY)
        ShowAction.feedback.current_step = "Say it saw the human saw the way"
        if self.preempt_requested():
            rospy.loginfo("SaySeen preempted")
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


class AskSeen(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of AskSeen state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['human_look_at_point'],
                             output_keys=['question_asked'])

    def execute(self, userdata):
        ShowAction.services_proxy["say"](userdata.human_look_at_point, "Have you seen where you have to go ?",
                                         SPEECH_PRIORITY)
        ShowAction.feedback.current_step = "Ask if the landmark has been seen"
        userdata.question_asked = 'ask_seen'
        if self.preempt_requested():
            rospy.loginfo("AskSeen preempted")
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


class AskPointAgain(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of AskPointAgain state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['human_look_at_point'],
                             output_keys=['question_asked'])

    def execute(self, userdata):
        ShowAction.services_proxy["say"](userdata.human_look_at_point, "Should I show you the direction again ?",
                                         SPEECH_PRIORITY)
        ShowAction.feedback.current_step = "Ask if the robot should show again"
        userdata.question_asked = 'ask_point_again'
        if self.preempt_requested():
            rospy.loginfo("AskPointAgain preempted")
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


class GetYesNo(smach_ros.SimpleActionState):
    _repeat_question = 0

    def __init__(self):
        rospy.loginfo("Initialization of GetYesNo state")
        smach_ros.SimpleActionState.__init__(self, rospy.get_param("/action_servers/dialogue"),
                                             dialogue_actionAction,
                                             goal_cb=self.dialogue_goal_cb,
                                             feedback_cb=self.dialogue_feedback_cb,
                                             result_cb=self.dialogue_result_cb,
                                             input_keys=['human_look_at_point'],
                                             output_keys=['result_word'],
                                             exec_timeout=rospy.Duration(5.0),
                                             server_wait_timeout=rospy.Duration(1.0))

    def dialogue_goal_cb(self, userdata, goal):
        # ShowAction.services_proxy["activate_dialogue"]()
        ShowAction.feedback.current_step = "Receive yes no answer"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        dialogue_goal = dialogue_actionGoal()
        dialogue_goal.subjects = ['yes', 'no']
        dialogue_goal.enable_only_subject = True
        return dialogue_goal

    def dialogue_feedback_cb(self, userdata, feedback):
        self._activate_time = rospy.Time.now()
        ShowAction.feedback.current_step = "Human said something else thant yes or no"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                         "I'm sorry I did not understand, you should say yes or no",
                                         SPEECH_PRIORITY)
        self._activate_time = rospy.Time.now()
        self._repeat_question = 0

    def dialogue_result_cb(self, userdata, status, result):
        action_success = ''
        if status == actionlib.GoalStatus.SUCCEEDED:
            ShowAction.feedback.current_step = "Human said yes or no"
            ShowAction.action_server.publish_feedback(ShowAction.feedback)
            userdata.result_word = result.subject
            action_success = 'succeeded'
            self._repeat_question = 0
        elif status == actionlib.GoalStatus.PREEMPTED:
            if not self.preempt_requested():
                if self._repeat_question < 2:
                    if self._repeat_question == 0:
                        ShowAction.feedback.current_step = "Robot did not hear for the first time"
                        ShowAction.action_server.publish_feedback(ShowAction.feedback)
                        ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                                         "Sorry I can't hear you, can you repeat ?",
                                                         SPEECH_PRIORITY)
                    elif self._repeat_question == 1:
                        ShowAction.feedback.current_step = "Robot did not hear for the second time"
                        ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                                         "I am really sorry I still can not hear you, "
                                                         "can you speak louder ?",
                                                         SPEECH_PRIORITY)

                    self._repeat_question += 1
                    action_success = 'preempted'
                else:
                    ShowAction.feedback.current_step = "Robot did not hear and gave up"
                    ShowAction.action_server.publish_feedback(ShowAction.feedback)
                    userdata.result_word = result.subject
                    ShowAction.services_proxy["say"](userdata.human_look_at_point, "I did not hear, sorry, I give up",
                                                     SPEECH_PRIORITY)
                    action_success = 'aborted'
            else:
                ShowAction.feedback.current_step = "GetYesNo state preempted"
                ShowAction.action_server.publish_feedback(ShowAction.feedback)
                action_success = 'aborted'
        else:
            ShowAction.feedback.current_step = "Dialog goal status aborted"
            ShowAction.action_server.publish_feedback(ShowAction.feedback)

            action_success = 'aborted'

        # ShowAction.services_proxy["deactivate_dialogue"]()

        return action_success


class DispatchYesNo(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of DispatchYesNo state")
        smach.State.__init__(self, outcomes=['show', 'no', 'preempted'],
                             input_keys=['question_asked', 'result_word', 'human_look_at_point'])

    def execute(self, userdata):
        next_action = ''
        if self.preempt_requested():
            rospy.loginfo("DispatchYesNo preempted")
            self.service_preempt()
            next_action = 'preempted'
        elif userdata.question_asked == 'ask_show_place':
            if userdata.result_word == 'yes':
                next_action = 'show'
            else:
                ShowAction.services_proxy["say"](userdata.human_look_at_point, "Ok, I hope you will get there!",
                                                 SPEECH_PRIORITY)
                next_action = 'no'
        elif userdata.question_asked == 'ask_show_direction':
            if userdata.result_word == 'yes':
                next_action = 'show'
            else:
                ShowAction.services_proxy["say"](userdata.human_look_at_point, "Ok, I hope you will get there!",
                                                 SPEECH_PRIORITY)
                next_action = 'no'

        return next_action


class DispatchYesNoCL(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of DispatchYesNo state")
        smach.State.__init__(self, outcomes=['yes', 'preempted', 'ask_point_again', 'pointing', 'no'],
                             input_keys=['question_asked', 'result_word', 'human_look_at_point'])

    def execute(self, userdata):
        next_action = ''

        if self.preempt_requested():
            rospy.loginfo("DispatchYesNoCL preempted")
            self.service_preempt()
            next_action = 'preempted'

        if userdata.question_asked == 'ask_seen':
            if userdata.result_word == 'yes':
                ShowAction.services_proxy["say"](userdata.human_look_at_point, "Awesome !", SPEECH_PRIORITY)
                next_action = 'yes'
            else:
                ShowAction.services_proxy["say"](userdata.human_look_at_point,
                                                 "Oh it's too bad, I'm sorry I wasn't good enough", SPEECH_PRIORITY)
                next_action = 'ask_point_again'

        elif userdata.question_asked == 'ask_point_again':
            if userdata.result_word == 'yes':
                next_action = 'pointing'
            else:
                ShowAction.services_proxy["say"](userdata.human_look_at_point, "Ok, too bad. Good luck !",
                                                 SPEECH_PRIORITY)
                next_action = 'no'

        return next_action


class IsOver(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of DispatchYesNo state")
        smach.State.__init__(self, outcomes=['succeeded', 'point_direction', 'preempted', 'aborted'],
                             input_keys=['landmark_to_point', 'landmarks_to_point', 'last_outcome', 'last_outcome'])

    def execute(self, userdata):
        if self.preempt_requested():
            rospy.loginfo("IsOver preempted")
            self.service_preempt()
            return 'preempted'

        if len(userdata.landmarks_to_point) == 2 or \
                (userdata.landmark_to_point[LANDMARK_NAME] not in userdata.landmarks_to_point
                 and len(userdata.landmarks_to_point) == 1):
            return 'point_direction'
        elif userdata.last_outcome == 'no':
            return 'aborted'
        else:
            return 'succeeded'


class Failure(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of Failure state")
        smach.State.__init__(self, outcomes=['aborted'], input_keys=['active_state'])

    def execute(self, userdata):
        return 'aborted'


if __name__ == '__main__':
    rospy.init_node('guiding_action_server')
    server = ShowAction(rospy.get_name())
    rospy.on_shutdown(server.stand_pose)
    rospy.spin()

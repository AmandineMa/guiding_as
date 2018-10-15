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
from collections import namedtuple
import json
import tf.transformations as transform
from nao_interaction_msgs.srv import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from perspectives_msgs.msg import *
from guiding_as.msg import *
from deictic_gestures_msgs.srv import *
from semantic_route_description_msgs.srv import *
from semantic_route_description_msgs.msg import *
from perspectives_msgs.srv import *
from ontologenius_msgs.srv import *
from route_verbalization_msgs.srv import *
from multimodal_human_monitor_msgs.srv import *
from head_manager_msgs.msg import *
from pointing_planner_msgs.msg import *
from std_srvs.srv import Trigger
from dialogue_as.msg import *
from pointing_planner_msgs.srv import *
from speech_wrapper_msgs.srv import *
from rpn_recipe_planner_msgs.srv import *
from mummer_navigation_msgs.msg import *
from service_wrapper import ServiceWrapper
from visualization_msgs.msg import *
from std_msgs.msg import ColorRGBA

__all__ = ['AskHumanToMoveAfter', 'AskPointAgain', 'AskSeen', 'AskShowDirection', 'AskShowPlace', 'DispatchYesNo',
           'DispatchYesNoCL', 'GetRouteRegion', 'GuidingAction', 'HumanLost', 'IsOver',
           'LookAtHuman', 'LookAtAssumedPlace', 'MoveToPose', 'PointAndLookAtHumanFuturePlace',
           'PointAndLookAtLandmark', 'PointingConfig', 'PointNotVisible', 'SaySeen', 'SelectLandmark',
           'ShouldHumanMove', 'StopTrackingCondition', 'Timer']

# constants to get with rospy.get_param
ROBOT_PLACE = ""
POINTING_DURATION = 0
WORLD = ""
STOP_TRACK_DIST_TH = 0
HUMAN_SHOULD_MOVE_DIST_TH = 0
ROBOT_SHOULD_MOVE_DIST_TH = 0
TAKE_ROBOT_PLACE_DIST_TH = 0
LOST_PERCEPTION_TIMEOUT = 0
OBSERVE_LANDMARK_TIMEOUT = 0
HWU_DIAL = False
SIGNPOST = False
HUMAN_FOLLOW = True

# supervisor constants
SPEECH_PRIORITY = 250

ROBOT_POSE = 0
HUMAN_POSE = 3
LOOK_AT_HUMAN = 1
LOOK_AT_ASSUMED_PLACE = 2
LOOK_AT_LANDMARK = 4

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
    sm_test_rotation = None
    coord_signals_publisher = None
    visualization_publisher = None
    marker_handling = None

    def __init__(self, name):
        self._action_name = name
        self.waiting_goals = []
        self.top_userdata = None
        self.guiding_userdata = None
        self.show_userdata = None
        self.SavedGoal = namedtuple("SavedGoal", "target_frame person_frame")
        self.SavedEnv = namedtuple("SavedEnv", "goal interrupted_state top_ud guiding_ud show_ud")
        self.stand_pose_srv = ""
        self.say_srv = ""
        # get_route_region_srv = ""
        self.get_route_srv = ""
        self.route_verbalization_srv = ""
        self.get_individual_info_srv = ""
        self.is_visible_srv = ""
        self.can_look_at_srv = ""
        self.look_at_srv = ""
        self.can_point_at_srv = ""
        self.point_at_srv = ""
        self.rest_arm_srv = ""
        self.has_mesh_srv = ""
        self.monitor_humans_srv = ""
        self.start_fact_srv = ""
        self.end_fact_srv = ""
        self.find_alternate_id_srv = ""
        self.dialogue_inform_srv = ""
        self.dialogue_query_srv = ""

        GuidingAction.action_server = actionlib.SimpleActionServer(self._action_name, taskAction,
                                                                   execute_cb=self.execute_cb,
                                                                   auto_start=False)

        self.run = True
        GuidingAction.action_server.register_preempt_callback(self.preempt_cb)

        self.get_service_names()
        self.wait_for_services_and_as()

        GuidingAction.services_proxy = {
            "stand_pose": ServiceWrapper(self.stand_pose_srv, GoToPosture),
            "say": rospy.ServiceProxy(self.say_srv, SpeakTo),
            "get_route": ServiceWrapper(self.get_route_srv, SemanticRoute),
            "route_verbalization": ServiceWrapper(self.route_verbalization_srv, VerbalizeRegionRoute),
            "is_visible": ServiceWrapper(self.is_visible_srv, VisibilityScore),
            "has_mesh": ServiceWrapper(self.has_mesh_srv, HasMesh),
            "get_individual_info": ServiceWrapper(self.get_individual_info_srv, OntologeniusService),
            "can_look_at": ServiceWrapper(self.can_look_at_srv, CanLookAt),
            "can_point_at": ServiceWrapper(self.can_point_at_srv, CanPointAt),
            "look_at": ServiceWrapper(self.look_at_srv, LookAt),
            "point_at": ServiceWrapper(self.point_at_srv, PointAt),
            "rest_arm": rospy.ServiceProxy(self.rest_arm_srv, RestArm),
            "monitor_humans": rospy.ServiceProxy(self.monitor_humans_srv, MonitorHumans),
            "start_fact": rospy.ServiceProxy(self.start_fact_srv, StartFact),
            "end_fact": rospy.ServiceProxy(self.start_fact_srv, EndFact),
            "find_alternate_id": rospy.ServiceProxy(self.find_alternate_id_srv, FindAlternateId),
            "dialogue_inform": ServiceWrapper(self.dialogue_inform_srv, SuperInform),
            "dialogue_query": ServiceWrapper(self.dialogue_query_srv, SuperQuery)}
        # "activate_dialogue": rospy.ServiceProxy(activate_dialogue_srv, Trigger),
        # "deactivate_dialogue": rospy.ServiceProxy(deactivate_dialogue_srv, Trigger)}

        GuidingAction.coord_signals_publisher = rospy.Publisher(rospy.get_param('/guiding/topics/coord_signals'),
                                                                CoordinationSignal, queue_size=5)

        GuidingAction.marker_handling = MarkerHandling()

        self.guiding_sm = smach.StateMachine(outcomes=['task_succeeded', 'task_failed', 'preempted'],
                                             input_keys=['person_frame', 'human_look_at_point'],
                                             output_keys=['last_state', 'last_outcome', 'person_frame'])

        self.show_sm = smach.StateMachine(outcomes=['show_succeeded', 'show_failed', 'preempted'],
                                          input_keys=['target_frame', 'person_frame', 'human_look_at_point',
                                                      'route', 'goal_frame', 'persona', 'direction',
                                                      'last_state', 'last_outcome', 'landmarks_to_point',
                                                      'route_2_shop_wo_sign', 'signpost'],
                                          output_keys=['last_state', 'last_outcome', 'person_frame'])

        self.check_landmark_seen_sm = smach.StateMachine(outcomes=['yes', 'no', 'pointing', 'preempted', 'failure'],
                                                         input_keys=['human_look_at_point', 'last_state',
                                                         'person_frame', 'last_outcome'])

        self.top_sm = smach.StateMachine(outcomes=['task_succeeded', 'task_failed', 'preempted'])

        self.build_state_machines()

        self.register_sm_cb()

        # temporary here
        GuidingAction.sm_test_rotation = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
        with GuidingAction.sm_test_rotation:
            smach.StateMachine.add('GetRotationAngle', GetRotationAngle(), transitions={'succeeded': 'Rotate',
                                                                                        'aborted': 'Rotate'})
            smach.StateMachine.add('Rotate', RotateRobot())
        # --- #

        self.sis = smach_ros.IntrospectionServer('server_name', self.top_sm, '/SM_ROOT')
        self.sis.start()
        self.sis_rot = smach_ros.IntrospectionServer('server_rot', GuidingAction.sm_test_rotation, '/SM_ROT')
        self.sis_rot.start()
        smach_ros.set_preempt_handler(self.top_sm)

        rospy.loginfo("start action server")
        GuidingAction.action_server.start()
        rospy.loginfo("action server started")

        # GuidingAction.services_proxy["deactivate_dialogue"]()

    def get_service_names(self):
        self.stand_pose_srv = rospy.get_param('/guiding/services/stand_pose')
        self.say_srv = rospy.get_param('/guiding/services/say')
        self.get_route_srv = rospy.get_param('/guiding/services/get_route')
        self.route_verbalization_srv = rospy.get_param('/guiding/services/route_verbalization')
        self.get_individual_info_srv = rospy.get_param('/guiding/services/get_individual_info')
        self.is_visible_srv = rospy.get_param('/guiding/services/is_visible')
        self.can_look_at_srv = rospy.get_param('/guiding/services/can_look_at')
        self.look_at_srv = rospy.get_param('/guiding/services/look_at')
        self.can_point_at_srv = rospy.get_param('/guiding/services/can_point_at')
        self.point_at_srv = rospy.get_param('/guiding/services/point_at')
        self.rest_arm_srv = rospy.get_param('/guiding/services/rest_arm')
        self.has_mesh_srv = rospy.get_param('/guiding/services/has_mesh')
        self.monitor_humans_srv = rospy.get_param('/guiding/services/monitor_humans')
        self.start_fact_srv = rospy.get_param('/guiding/services/start_fact')
        self.end_fact_srv = rospy.get_param('/guiding/services/end_fact')
        self.find_alternate_id_srv = rospy.get_param('/guiding/services/find_alternate_id')
        self.dialogue_inform_srv = rospy.get_param('guiding/services/dialogue_inform')
        self.dialogue_query_srv = rospy.get_param('guiding/services/dialogue_query')

    def get_params(self):
        global HUMAN_FOLLOW
        HUMAN_FOLLOW = rospy.get_param('/guiding/tuning_param/human_follow')
        global ROBOT_PLACE
        ROBOT_PLACE = rospy.get_param('/guiding/perspective/robot_place')
        global WORLD
        WORLD = rospy.get_param('/guiding/perspective/world')
        global POINTING_DURATION
        POINTING_DURATION = rospy.get_param('/guiding/tuning_param/pointing_duration')
        global STOP_TRACK_DIST_TH
        STOP_TRACK_DIST_TH = rospy.get_param('/guiding/tuning_param/stop_tracking_dist_th')
        global SIGNPOST
        SIGNPOST = rospy.get_param('/guiding/tuning_param/signpost')
        global HUMAN_SHOULD_MOVE_DIST_TH
        HUMAN_SHOULD_MOVE_DIST_TH = rospy.get_param('/guiding/tuning_param/human_should_move_dist_th')
        global ROBOT_SHOULD_MOVE_DIST_TH
        ROBOT_SHOULD_MOVE_DIST_TH = rospy.get_param('/guiding/tuning_param/robot_should_move_dist_th')
        global TAKE_ROBOT_PLACE_DIST_TH
        TAKE_ROBOT_PLACE_DIST_TH = rospy.get_param('/guiding/tuning_param/take_robot_place_dist_th')
        global LOST_PERCEPTION_TIMEOUT
        LOST_PERCEPTION_TIMEOUT = rospy.get_param('/guiding/tuning_param/lost_perception_timeout')
        global OBSERVE_LANDMARK_TIMEOUT
        OBSERVE_LANDMARK_TIMEOUT = rospy.get_param('/guiding/tuning_param/observe_landmark_timeout')
        global HWU_DIAL
        HWU_DIAL = rospy.get_param('/guiding/dialogue/hwu')

    def wait_for_services_and_as(self):

        rospy.loginfo("waiting for action servers and services")

        services_status = {
            self.stand_pose_srv: False,
            self.say_srv: False,
            self.get_route_srv: False,
            self.route_verbalization_srv: False,
            self.get_individual_info_srv: False,
            self.is_visible_srv: False,
            self.can_look_at_srv: False,
            self.look_at_srv: False,
            self.can_point_at_srv: False,
            self.point_at_srv: False,
            self.rest_arm_srv: False,
            self.has_mesh_srv: False,
            self.monitor_humans_srv: False,
            self.start_fact_srv: False,
            self.end_fact_srv: False,
            self.find_alternate_id_srv: False}

        clients = {
            "move_to": actionlib.SimpleActionClient(rospy.get_param("/guiding/action_servers/move_to"), MoveBaseAction),
            "rotate": actionlib.SimpleActionClient(rospy.get_param("/guiding/action_servers/rotate"), RotateAction),
            "dialogue": actionlib.SimpleActionClient(rospy.get_param("/guiding/action_servers/dialogue"),
                                                     dialogue_actionAction),
            "svp_planner": actionlib.SimpleActionClient(rospy.get_param("/guiding/action_servers/pointing_planner"),
                                                        PointingAction)}

        servers_status = {
            "move_to": False,
            "rotate": False,
            "dialogue": False,
            "svp_planner": False}

        if rospy.get_param('/guiding/dialogue/hwu'):
            services_status[self.dialogue_inform_srv] = False
            services_status[self.dialogue_query_srv] = False

        not_started = True
        first = True

        while not_started and not rospy.is_shutdown():

            not_started = False
            old_services_status = copy.deepcopy(services_status)
            old_servers_status = copy.deepcopy(servers_status)

            for srv in services_status:
                try:
                    rospy.wait_for_service(srv, timeout=2)
                    services_status[srv] = True
                except rospy.ROSException:
                    services_status[srv] = False
                    not_started = True

            for srver in servers_status:
                server_started = clients[srver].wait_for_server(timeout=rospy.Duration(2))
                if server_started:
                    servers_status[srver] = True
                else:
                    servers_status[srver] = False
                    not_started = True

            if old_services_status != services_status or old_servers_status != servers_status or first:
                first = False
                for srv, status in services_status.iteritems():
                    if status:
                        rospy.loginfo(srv + " status: OK")
                    else:
                        rospy.logerr(srv + " status: Not started")

                for srver, status in servers_status.iteritems():
                    if status:
                        rospy.loginfo(srver + " status: OK")
                    else:
                        rospy.logerr(srver + " status: Not started")

        if rospy.is_shutdown():
            exit(0)

    def build_state_machines(self):

        # Add States of Guiding Container
        with self.guiding_sm:
            smach.StateMachine.add('GetRouteRegion', GetRouteRegion(),
                                   transitions={'in_region': 'AskShowPlace', 'out_region': 'AskShowDirection',
                                                'unknown': 'task_failed', 'preempted': 'preempted',
                                                'aborted': 'task_failed'})

            smach.StateMachine.add('AskShowPlace', AskShowPlace(),
                                   transitions={'get_answer': 'GetAnswerShow', 'show': 'Show',
                                                'not_show': 'task_succeeded', 'preempted': 'preempted'})

            smach.StateMachine.add('AskShowDirection', AskShowDirection(),
                                   transitions={'get_answer': 'GetAnswerShow', 'yes': 'AskStairsOrElevator',
                                                'not_show': 'task_succeeded', 'preempted': 'preempted'})

            smach.StateMachine.add('GetAnswerShow', GetAnswer(),
                                   transitions={'succeeded': 'DispatchYesNo', 'aborted': 'task_failed',
                                                'preempted': 'GetAnswerShow'})

            smach.StateMachine.add('DispatchYesNo', DispatchYesNo(),
                                   transitions={'ask_stairs_or_elevator': 'AskStairsOrElevator', 'show': 'Show',
                                                'no': 'task_succeeded', 'preempted': 'preempted'})

            smach.StateMachine.add('AskStairsOrElevator', AskStairsOrElevator(),
                                   transitions={'get_answer': 'GetAnswerStairs', 'stairs': 'Show',
                                                'elevator': 'GetRouteRegionBis', 'no_stairs': 'Show',
                                                'preempted': 'preempted'})

            smach.StateMachine.add('GetAnswerStairs', GetAnswer(),
                                   transitions={'succeeded': 'DispatchStairsElevator', 'aborted': 'task_failed',
                                                'preempted': 'GetAnswerStairs'})

            smach.StateMachine.add('DispatchStairsElevator', DispatchStairsElevator(),
                                   transitions={'stairs': 'Show', 'elevator': 'GetRouteRegionBis',
                                                'preempted': 'preempted'})

            # in_region: useless output here
            smach.StateMachine.add('GetRouteRegionBis', GetRouteRegion(),
                                   transitions={'in_region': 'AskShowPlace', 'out_region': 'Show',
                                                'unknown': 'task_failed', 'preempted': 'preempted',
                                                'aborted': 'task_failed'})

            with self.show_sm:
                smach.StateMachine.add('ShouldCallPointingConfig', ShouldCallPointingConfig(),
                                       transitions={'succeeded': 'PointingConfig',
                                                    'point_not_visible': 'SelectLandmark',
                                                    'preempted': 'preempted'})

                smach.StateMachine.add('PointingConfig', PointingConfig(),
                                       transitions={'succeeded': 'ShouldHumanMove',
                                                    'point_not_visible': 'SelectLandmark',
                                                    'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                look_at_human_assumed_place = smach.StateMachine(
                    outcomes=['look_succeeded', 'look_failed', 'preempted'],
                    input_keys=['human_pose', 'human_look_at_point', 'person_frame'])

                with look_at_human_assumed_place:
                    smach.StateMachine.add('LookAtAssumedPlace', LookAtAssumedPlace(),
                                           transitions={'succeeded': 'look_succeeded', 'preempted': 'preempted',
                                                        'human_lost': 'LookAtHuman',
                                                        'look_again': 'LookAtAssumedPlace'})

                    smach.StateMachine.add('LookAtHuman', LookAtHuman(),
                                           transitions={'succeeded': 'CheckPerceived', 'aborted': 'look_failed',
                                                        'preempted': 'preempted'})

                    smach.StateMachine.add('CheckPerceived', CheckPerceived(),
                                           transitions={'succeeded': 'look_succeeded', 'aborted': 'look_failed',
                                                        'wait': 'LookAtHuman', 'preempted': 'preempted'})

                if HUMAN_FOLLOW:
                    smach.StateMachine.add('ShouldHumanMove', ShouldHumanMove(),
                                           transitions={'human_move': 'AskHumanToFollow',
                                                        'no': 'ShouldRobotMoveX',
                                                        'robot_first': 'AskHumanToMoveAfter', 'aborted': 'show_failed',
                                                        'preempted': 'preempted'})

                    smach.StateMachine.add('AskHumanToFollow', AskHumanToFollow(),
                                           transitions={'succeeded': 'MoveToPoseY', 'preempted': 'preempted'})
                else:
                    smach.StateMachine.add('ShouldHumanMove', ShouldHumanMove(),
                                           transitions={'human_move': 'PointAndLookAtHumanFuturePlace',
                                                        'no': 'ShouldRobotMoveX',
                                                        'robot_first': 'AskHumanToMoveAfter', 'aborted': 'show_failed',
                                                        'preempted': 'preempted'})

                    smach.StateMachine.add('PointAndLookAtHumanFuturePlace', PointAndLookAtHumanFuturePlace(),
                                           transitions={'succeeded': 'LookAtHumanAssumedPlaceX',
                                                        'aborted': 'show_failed',
                                                        'preempted': 'preempted'})

                    smach.StateMachine.add('LookAtHumanAssumedPlaceX', look_at_human_assumed_place,
                                           transitions={'look_succeeded': 'AreLandmarksVisibleFromHuman',
                                                        'look_failed': 'HumanLost', 'preempted': 'preempted'})

                    smach.StateMachine.add('AreLandmarksVisibleFromHuman', AreLandmarksVisibleFromHuman(),
                                           transitions={'landmarks_visible': 'PointingConfigForRobot',
                                                        'landmark_s_not_visible': 'PointingConfig',
                                                        'aborted': 'show_failed'})

                    smach.StateMachine.add('PointingConfigForRobot', PointingConfigForRobot(),
                                           transitions={'succeeded': 'ShouldRobotMoveX', 'aborted': 'show_failed'})

                smach.StateMachine.add('ShouldRobotMoveX', ShouldRobotMove(),
                                       transitions={'yes': 'MoveToPoseX', 'no': 'LookAtHumanX',
                                                    'preempted': 'preempted', 'aborted': 'show_failed'})

                smach.StateMachine.add('MoveToPoseX', MoveToPose(),
                                       transitions={'succeeded': 'LookAtHumanX',
                                                    'preempted': 'preempted', 'aborted': 'show_failed'})

                smach.StateMachine.add('LookAtHumanX', LookAtHuman(),
                                       transitions={'succeeded': 'SelectLandmark', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                smach.StateMachine.add('AskHumanToMoveAfter', AskHumanToMoveAfter(),
                                       transitions={'succeeded': 'MoveToPoseY', 'preempted': 'preempted'})

                smach.StateMachine.add('HumanLost', HumanLost(), transitions={'human_lost': 'show_failed'})

                smach.StateMachine.add('MoveToPoseY', MoveToPose(),
                                       transitions={'succeeded': 'LookAtHumanAssumedPlaceY',
                                                    'preempted': 'preempted', 'aborted': 'show_failed'})

                smach.StateMachine.add('LookAtHumanAssumedPlaceY', look_at_human_assumed_place,
                                       transitions={'look_succeeded': 'SelectLandmark', 'preempted': 'preempted',
                                                    'look_failed': 'HumanLost'})

                smach.StateMachine.add('SelectLandmark', SelectLandmark(),
                                       transitions={'point_not_visible': 'PointNotVisible',
                                                    'point_direction': 'PointAndLookAtLandmark',
                                                    'aborted': 'show_failed',
                                                    'point_target': 'PointAndLookAtLandmark', 'preempted': 'preempted'})

                smach.StateMachine.add('PointNotVisible', PointNotVisible(),
                                       transitions={'succeeded': 'IsOver', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                smach.StateMachine.add('PointAndLookAtLandmark', PointAndLookAtLandmark(),
                                       transitions={'succeeded': 'CheckLandmarkSeen', 'aborted': 'show_failed',
                                                    'preempted': 'preempted'})

                # smach.StateMachine.add('PointAndLookAtLandmark', PointAndLookAtLandmark(),
                #                        transitions={'succeeded': 'ObserveLandmarkSeen', 'aborted': 'show_failed',
                #                                     'preempted': 'preempted'})

                # To observe if the human has seen the target. If the human does not see it after x seconds,
                # the container is terminated and transition to ask if he has seen. If the robot observes the human
                # looking

                # LookAtHuman state if observe target seen
                # smach.StateMachine.add('LookAtHuman', LookAtHuman(),
                #                        transitions={'succeeded': 'CheckLandmarkSeen', 'aborted': 'show_failed',
                #                                     'preempted': 'preempted'})

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

                with self.check_landmark_seen_sm:
                    self.check_landmark_seen_sm.add('AskSeen', AskSeen(), transitions={'get_answer': 'GetAnswerCL',
                                                                                       'seen': 'yes',
                                                                                       'no': 'AskPointAgain',
                                                                                       'preempted': 'preempted'})

                    self.check_landmark_seen_sm.add('AskPointAgain', AskPointAgain(),
                                                    transitions={'get_answer': 'GetAnswerCL', 'pointing': 'pointing',
                                                                 'no': 'no', 'preempted': 'preempted'})

                    self.check_landmark_seen_sm.add('GetAnswerCL', GetAnswer(),
                                                    transitions={'succeeded': 'DispatchYesNoCL',
                                                                 'preempted': 'GetAnswerCL',
                                                                 'aborted': 'failure'})

                    self.check_landmark_seen_sm.add('DispatchYesNoCL', DispatchYesNoCL(),
                                                    transitions={'yes': 'yes',
                                                                 'no': 'no',
                                                                 'ask_point_again': 'AskPointAgain',
                                                                 'pointing': 'pointing', 'preempted': 'preempted'})

                smach.StateMachine.add('CheckLandmarkSeen', self.check_landmark_seen_sm,
                                       transitions={'yes': 'IsOver', 'no': 'IsOver',
                                                    'pointing': 'PointAndLookAtLandmark',
                                                    'failure': 'show_failed', 'preempted': 'preempted'})

                smach.StateMachine.add('IsOver', IsOver(),
                                       transitions={'succeeded': 'show_succeeded', 'point_direction': 'SelectLandmark',
                                                    'preempted': 'preempted', 'aborted': 'show_failed'})

            smach.StateMachine.add('Show', self.show_sm, transitions={'show_succeeded': 'task_succeeded',
                                                                      'show_failed': 'task_failed',
                                                                      'preempted': 'preempted'})

        guiding_monitoring = smach.Concurrence(outcomes=['task_succeeded', 'task_failed', 'preempted', 'human_lost'],
                                               default_outcome='task_failed',
                                               outcome_cb=self.g_m_c_out_cb,
                                               input_keys=['person_frame', 'human_look_at_point'],
                                               child_termination_cb=self.g_m_c_term_cb)
        with guiding_monitoring:
            smach.Concurrence.add('GUIDING', self.guiding_sm)
            smach.Concurrence.add('HUMAN_MONITOR',
                                  smach_ros.MonitorState(rospy.get_param("/guiding/topics/current_facts"),
                                                         FactArrayStamped,
                                                         self.human_perceive_monitor_cb,
                                                         input_keys=['person_frame'],
                                                         output_keys=['duration_lost', 'human_lost']))

        with self.top_sm:
            smach.StateMachine.add('GUIDING_MONITORING', guiding_monitoring,
                                   transitions={'task_succeeded': 'task_succeeded', 'task_failed': 'task_failed',
                                                'preempted': 'preempted', 'human_lost': 'HumanLost'})
            smach.StateMachine.add('HumanLost', HumanLost(), transitions={'human_lost': 'task_failed'})

    def register_sm_cb(self):
        self.guiding_sm.register_transition_cb(self.guiding_transition_cb, cb_args=[self.guiding_sm])
        self.guiding_sm.register_start_cb(self.guiding_start_cb, cb_args=[])
        self.guiding_sm.register_termination_cb(self.term_cb, cb_args=[])
        self.show_sm.register_start_cb(self.guiding_start_cb, cb_args=[])
        self.show_sm.register_transition_cb(self.guiding_start_cb, cb_args=[self.show_sm])
        self.check_landmark_seen_sm.register_transition_cb(self.guiding_transition_cb,
                                                           cb_args=[self.check_landmark_seen_sm])

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
            if GuidingAction.action_server.is_active():
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
            if next_state not in ('MoveToPoseX', 'MoveToPoseY', 'PointNotVisible', 'PointAndLookAtLandmark',
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

                if rospy.get_param('/guiding/tuning_param/stop_when_human_lost'):
                    # to stop the state machine if human not perceived after X time
                    if rospy.Duration(self.duration_lost) > rospy.Duration(LOST_PERCEPTION_TIMEOUT):
                        return False
            # pas joli de faire ca ici
            # try:
            #     alt_id = GuidingAction.services_proxy["find_alternate_id"](userdata.person_frame)
            #     if alt_id.success:
            #         monitor_humans_request = MonitorHumansRequest()
            #         monitor_humans_request.action = "REMOVE"
            #         try:
            #             GuidingAction.services_proxy["monitor_humans"](monitor_humans_request)
            #             global person_frame
            #             person_frame = alt_id.result_id
            #             monitor_humans_request.action = "ADD"
            #             monitor_humans_request.humans_to_monitor = [alt_id.result_id]
            #             GuidingAction.services_proxy["monitor_humans"](monitor_humans_request)
            #         except rospy.ServiceException, e:
            #             rospy.logerr("monitor human exception")
            #         rospy.logwarn(alt_id.result_id)
            # except rospy.ServiceException, e:
            #     rospy.logerr("find alternate id exception")

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
        elif outcome_map['GUIDING'] == 'preempted':
            return 'preempted'
        elif outcome_map['GUIDING'] == 'task_failed':
            return 'task_failed'
        elif outcome_map['HUMAN_MONITOR'] == 'invalid':
            return 'human_lost'
        else:
            return 'task_failed'

    def human_tracking_term_cb(self, outcome_map):
        """Callback that terminates the HumanTracking concurrent container in either of those cases:

        * the human has not reached his pose and the robot finishes to perform the look at (if the condition
          outcome_map['LookAtHumanTrack'] == 'succeeded' is not set, the LookAtHumanTrack state will be preempted if the
          StopTrackingCondition finishes first)
        * the human has reached his pose
        * the human is not perceived anymore

        TODO: add the preempted and aborted cases
        """
        rospy.logerr("outcome HumanMonitorTrack %s", outcome_map['HumanMonitorTrack'])
        if (outcome_map['StopTrackingCondition'] == 'continue_tracking' and outcome_map[
            'LookAtHumanTrack'] == 'succeeded') and outcome_map['HumanMonitorTrack'] == 'valid' \
                or outcome_map['StopTrackingCondition'] == 'succeeded' or outcome_map['HumanMonitorTrack'] == 'invalid' \
                or outcome_map['Timer2'] == 'stop':
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

    # ------ Callback to handle the monitoring of if the target is visible by the human or not ------ #

    # Return 'invalid' if the target is visible by the human
    def human_monitor_target_seen_cb(self, ud, msg):
        for fact in msg.facts:
            if fact.predicate == "isVisibleBy" \
                    and fact.object_name == ud.person_frame \
                    and fact.subject_name == ud.goal_frame:
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

    # ------ Callbacks to handle the action server ------ #

    def preempt_cb(self):
        if GuidingAction.action_server.is_new_goal_available():
            rospy.logwarn("A new goal arrived. The state machine stopped in state %s", next_state)
            goal = self.SavedGoal(self.guiding_sm.userdata.target_frame, self.top_sm.userdata.person_frame)

            top_ud = smach.UserData()
            top_ud.update(self.top_sm.userdata)

            guiding_ud = smach.UserData()
            guiding_ud.update(self.guiding_sm.userdata)

            show_ud = smach.UserData()
            show_ud.update(self.show_sm.userdata)

            saved_goal = self.SavedEnv(goal, next_state, top_ud, guiding_ud, show_ud)
            self.waiting_goals.append(saved_goal)

        if GuidingAction.action_server.is_preempt_requested() and GuidingAction.action_server.is_active():
            rospy.logerr("aborted")
            self.top_sm.request_preempt()

            GuidingAction.action_server.set_aborted(self._result)
            # GuidingAction.action_server.set_preempted()

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

        self.get_params()
        GuidingAction.marker_handling.remove_all_markers()

        start_waiting_goal = False
        if len(self.waiting_goals) != 0:
            for i in self.waiting_goals:
                if goal.place_frame in getattr(i, "goal") \
                        and goal.person_frame in getattr(i, "goal"):

                    self.top_sm.userdata = getattr(i, "top_ud")
                    self.guiding_sm.userdata = getattr(i, "guiding_ud")
                    self.show_sm.userdata = getattr(i, "show_ud")

                    self.top_sm.set_initial_state([getattr(i, "interrupted_state")])

                    try:
                        rospy.logwarn("An InvalidTransitionError is raised, no worry, it has to be ignored")
                        self.top_sm.check_consistency()

                    except smach.InvalidTransitionError:
                        self.top_sm.set_initial_state(['GUIDING_MONITORING'])
                        self.guiding_sm.set_initial_state([getattr(i, "interrupted_state")])
                        try:
                            rospy.logwarn("An InvalidTransitionError is raised, no worry, it has to be ignored")
                            self.guiding_sm.check_consistency()
                        except smach.InvalidTransitionError:
                            self.guiding_sm.set_initial_state(['Show'])
                            self.show_sm.set_initial_state([getattr(i, "interrupted_state")])
                    start_waiting_goal = True
                    self.waiting_goals.remove(i)

        if not start_waiting_goal:
            self.top_sm.set_initial_state(['GUIDING_MONITORING'])
            self.guiding_sm.set_initial_state(['GetRouteRegion'])
            self.show_sm.set_initial_state(['ShouldCallPointingConfig'])
            # userdata which needs to be initialized
            self.top_sm.userdata.person_frame = goal.person_frame
            self.guiding_sm.userdata.target_frame = goal.place_frame
            self.guiding_sm.userdata.persona = "lambda"
            look_at_point = PointStamped()
            look_at_point.header.frame_id = self.top_sm.userdata.person_frame
            self.top_sm.userdata.human_look_at_point = look_at_point

            self.guiding_sm.userdata.route = None
            self.guiding_sm.userdata.goal_frame = ""
            self.guiding_sm.userdata.direction = ""
            self.guiding_sm.userdata.last_outcome = None
            self.guiding_sm.userdata.landmarks_to_point = []

            self.show_sm.userdata.second_pointing_config = False

            PointingConfig.has_been_in_state = False

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
        GuidingAction.marker_handling.reset_variables()
        LookAtAssumedPlace.does_not_see = 0
        CheckPerceived.does_not_see = 0
        SelectLandmark.target_pointed = False
        GetAnswer.repeat_question = 0

        look_at_point = PointStamped()
        look_at_point.header.frame_id = self.top_sm.userdata.person_frame
        self.top_sm.userdata.human_look_at_point = look_at_point

        rospy.logwarn("State machine starting with goal %s %s", self.top_sm.userdata.person_frame,
                      self.guiding_sm.userdata.target_frame)

        outcome = self.top_sm.execute()

        # if self.human_lost:
        #     try:
        #         GuidingAction.services_proxy["end_fact"](WORLD, self.fact_id)
        #     except rospy.ServiceException, e:
        #         rospy.logerr("end fact exception")

        # Before shutdown
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

        if GuidingAction.action_server.is_active():
            GuidingAction.action_server.set_succeeded(self._result)

        self.sis.stop()

    @staticmethod
    def stand_pose():
        rospy.loginfo("stand pose")
        go_to_posture_request = GoToPostureRequest()
        go_to_posture_request.posture_name = "StandInit"
        go_to_posture_request.speed = 1
        GuidingAction.services_proxy["stand_pose"](go_to_posture_request)


class MarkerHandling(object):
    def __init__(self):
        self.visualization_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        self.orange_count = 0
        self.blue_count = 0
        self.green_count = 0
        self.yellow_count = 0
        self.marker_id = 0

    def orange(self):
        if self.orange_count == 0:
            return ColorRGBA(0.9, 0.7, 0.4, 1.0)
        elif self.orange_count == 1:
            return ColorRGBA(0.9, 0.6, 0, 1.0)
        else:
            return ColorRGBA(0.9, 0.5, 0.1, 1.0)

    def yellow(self):
        if self.yellow_count == 0:
            return ColorRGBA(1, 1, 0.1, 1.0)
        elif self.yellow_count == 1:
            return ColorRGBA(1, 0.9, 0.2, 1.0)
        else:
            return ColorRGBA(1, 0.8, 0.1, 1.0)

    def blue(self):
        if self.blue_count == 0:
            return ColorRGBA(0, 0.6, 0.9, 1.0)
        elif self.blue_count == 1:
            return ColorRGBA(0, 0.3, 0.7, 1.0)
        else:
            return ColorRGBA(0, 0, 1, 1.0)

    def green(self):
        if self.green_count == 0:
            return ColorRGBA(0, 0.8, 0.3, 1.0)
        elif self.green_count == 1:
            return ColorRGBA(0, 0.6, 0, 1.0)
        else:
            return ColorRGBA(0, 0.4, 0.2, 1.0)

    def purple(self):
        return ColorRGBA(0.5, 0.2, 0.8, 1.0)

    def reset_variables(self):
        self.orange_count = 0
        self.blue_count = 0
        self.green_count = 0
        self.yellow_count = 0
        self.marker_id = 0

    def publish_marker(self, frame, pose, color_type):
        marker = Marker()
        marker.ns = "supervisor"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = self.marker_id
        marker.pose = pose
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        if color_type == LOOK_AT_ASSUMED_PLACE:
            marker.color = self.green()
        elif color_type == LOOK_AT_HUMAN:
            marker.color = self.blue()
        elif color_type == ROBOT_POSE:
            marker.color = self.yellow()
        elif color_type == HUMAN_POSE:
            marker.color = self.orange()
        elif color_type == LOOK_AT_LANDMARK:
            marker.color = self.purple()
        marker.header.frame_id = frame
        self.marker_id += 1
        self.visualization_publisher.publish(marker)

    def remove_all_markers(self):
        marker = Marker()
        marker.ns = "supervisor"
        marker.action = Marker.DELETEALL
        self.visualization_publisher.publish(marker)


NO_INTERFACE_LEN = 3
DIRECTION_INDEX = 2


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
                             input_keys=['target_frame', 'human_look_at_point', 'persona', 'route_2_shop_wo_sign'],
                             io_keys=['route'],
                             output_keys=['stairs', 'goal_frame', 'direction', 'route_2_shop_wo_sign', 'signpost'])

    def get_name(self):
        return self.__class__.__name__

    def select_best_route(self, routes, costs, goals):
        """Select the best route among the list of routes, according to the cost of each one"""
        best_route = []
        goal_best_route = ""
        min_cost = None
        if len(routes) > 0:
            if len(routes) == 1:
                best_route = routes[0]
                goal_best_route = goals[0]
            else:
                for i in range(0, len(routes), 1):
                    if min_cost is None:
                        min_cost = costs[i]
                        best_route = routes[i]
                        goal_best_route = goals[i]
                    else:
                        if costs[i] < min_cost:
                            best_route = routes[i]
                            min_cost = costs[i]
                            goal_best_route = goals[i]

        return {'route': best_route, 'goal': goal_best_route}

    def execute(self, userdata):
        """Calls the route planner which returns a list of possible routes. Then select the best one among those.

        If the list of route is empty, it means the place is unknown.

        If the list of route has only one element, it means the target place is in the same region than the robot.

        If the list of route has more than one element, it means the target place is in a different region.
        """

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        try:

            get_route = GuidingAction.services_proxy["get_route"](
                ROBOT_PLACE,
                userdata.target_frame,
                userdata.persona,
                SIGNPOST,
                None)

        except rospy.ServiceException, e:
            rospy.logerr(e)
            return 'aborted'

        # if the routes list returned by the route planner is not empty
        if len(get_route.routes) != 0:

            select_best_route_result = self.select_best_route(get_route.routes, get_route.costs, get_route.goals)

            userdata.route = select_best_route_result['route'].route

            userdata.goal_frame = select_best_route_result['goal']

            get_up = GuidingAction.services_proxy["get_individual_info"]("getUp", select_best_route_result['goal'])

            if "signpost" in get_up.values:
                userdata.signpost = True
                # test if place in region
                get_route = GuidingAction.services_proxy["get_route"](
                    ROBOT_PLACE,
                    userdata.target_frame,
                    userdata.persona,
                    False,
                    None)
                select_best_route_result = self.select_best_route(get_route.routes, get_route.costs, get_route.goals)
                userdata.route_2_shop_wo_sign = select_best_route_result['route'].route
            else:
                userdata.route_2_shop_wo_sign = select_best_route_result['route'].route
                userdata.signpost = False

        else:
            userdata.route = []

        if any("stairs" in x for x in userdata.route):
            userdata.stairs = True
        else:
            userdata.stairs = False

        # if the routes list returned by the route planner is empty, problem with target or robot place
        if len(userdata.route) == 0:
            return 'unknown'
        else:

            if len(userdata.route_2_shop_wo_sign) == NO_INTERFACE_LEN:
                userdata.direction = ""
                return 'in_region'
            else:
                userdata.direction = userdata.route_2_shop_wo_sign[DIRECTION_INDEX]
                return 'out_region'


class AskShowPlace(smach.State):
    """Case where the robot is in the same region than the place

    Ask to the human : TARGET_NAME is nearby. Would you like me to show you the shop ?

    (Then, write "ask_show_target" to userdata.question_asked, it will be used by the state L{DispatchYesNo}.)

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
        smach.State.__init__(self, outcomes=['get_answer', 'show', 'not_show', 'preempted'],
                             input_keys=['goal_frame', 'human_look_at_point'],
                             output_keys=['question_asked'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the speech service to ask the question"""

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        target_name = GuidingAction.services_proxy["get_individual_info"]("getName", userdata.target_frame)

        # If a verbalized/label name exists in the ontology, it is used
        if target_name.code == 0:
            target_name_value = target_name.values[0]

        # If there is not, the 'technical' name is used
        else:
            target_name_value = userdata.goal_frame

        if HWU_DIAL:

            # say : {value} + "is nearby. Would you like me to guide you ?"
            answer = GuidingAction.services_proxy["dialogue_query"]("clarification.route_same_region",
                                                                    json.dumps(target_name_value))
            if answer.result == 'true':
                return 'show'
            else:
                return 'not_show'
        else:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                target_name_value +
                                                " is nearby. Would you like me to show you the place ?",
                                                SPEECH_PRIORITY)
            userdata.question_asked = 'ask_show_target'
            return 'get_answer'


class AskShowDirection(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['get_answer', 'yes', 'not_show', 'preempted'],
                             input_keys=['target_frame', 'human_look_at_point'],
                             output_keys=['question_asked'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        target_name = GuidingAction.services_proxy["get_individual_info"]("getName", userdata.target_frame)
        if target_name.code == 0:
            target_name_value = target_name.values[0]

        # If there is not, the 'technical' name is used
        else:
            target_name_value = userdata.target_frame

        if HWU_DIAL:
            # say: it's not here. Would you like me to indicate you the way ?
            answer = GuidingAction.services_proxy["dialogue_query"]("clarification.route_different_region", "")
            if answer.result == 'true':
                return 'yes'
            else:
                return 'not_show'
        else:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                target_name_value + " is not here",
                                                SPEECH_PRIORITY)
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                "Would you like me to indicate you the way ?",
                                                SPEECH_PRIORITY)

            userdata.question_asked = 'ask_show_direction'
            return 'get_answer'


class AskStairsOrElevator(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['get_answer', 'no_stairs', 'stairs', 'elevator', 'preempted'],
                             input_keys=['stairs', 'human_look_at_point'],
                             output_keys=['question_asked'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        if userdata.stairs:
            if HWU_DIAL:
                # say: It's on the first floor. Would you like to take the stairs or the elevator ?
                answer = GuidingAction.services_proxy["dialogue_query"]("clarification.route_stairs_elevator", "")
                if answer.result == 'true':
                    return 'stairs'
                else:
                    return 'elevator'
            else:
                GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                    "It's on the first floor.",
                                                    SPEECH_PRIORITY)
                GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                    "Would you like to take the stairs or the elevator ?",
                                                    SPEECH_PRIORITY)

                userdata.question_asked = 'ask_stairs_or_elevator'
                return 'get_answer'
        else:
            return 'no_stairs'


class ShouldCallPointingConfig(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['point_not_visible', 'succeeded', 'preempted'],
                             input_keys=['goal_frame', 'direction'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        count = 0

        has_mesh_result = GuidingAction.services_proxy["has_mesh"](WORLD, userdata.goal_frame)
        if not has_mesh_result.success:
            rospy.logerr("has_mesh service failed")
        else:
            # if there is no mesh, the target_frame remains empty
            if not has_mesh_result.has_mesh:
                count += 1

        has_mesh_result = GuidingAction.services_proxy["has_mesh"](WORLD, userdata.direction)
        if not has_mesh_result.success:
            rospy.logerr("has_mesh service failed")
        else:
            # if there is no mesh, the target_frame remains empty
            if not has_mesh_result.has_mesh:
                count += 1

        if count == 2:
            return 'point_not_visible'
        else:
            return 'succeeded'


class SVPPlannerClient(smach_ros.SimpleActionState):

    def define_svp_planner_lm(self, goal, direction):
        goal_lm = ""

        direction_lm = direction

        # Check if it exists an associated mesh to the goal
        has_mesh_result = GuidingAction.services_proxy["has_mesh"](WORLD, goal)
        if not has_mesh_result.success:
            rospy.logerr("has_mesh service failed")
        else:
            # if there is no mesh, the target_frame remains empty
            if has_mesh_result.has_mesh:
                goal_lm = goal
            else:
                rospy.logwarn("goal frame has no mesh")
                goal_lm = direction_lm
                direction_lm = ""

        # Check if it exists an associated mesh to the direction
        has_mesh_result = GuidingAction.services_proxy["has_mesh"](WORLD, direction)
        if not has_mesh_result.success:
            rospy.logerr("has_mesh service failed")
        else:
            if not has_mesh_result.has_mesh:
                rospy.logwarn("direction has no mesh")
                direction_lm = ""

        return {'goal': goal_lm, 'direction': direction_lm}


class PointingConfig(SVPPlannerClient):
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

    has_been_in_state = False

    def __init__(self):
        """Constructor for PointingConfig state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        self.mob_param = rospy.get_param('/pointing_planner/settings/mobhum')
        smach_ros.SimpleActionState.__init__(self, rospy.get_param("/guiding/action_servers/pointing_planner"),
                                             PointingAction, goal_cb=self.pointing_config_goal_cb,
                                             feedback_cb=self.pointing_config_feedback_cb,
                                             result_cb=self.pointing_config_result_cb,
                                             input_keys=['goal_frame', 'person_frame', 'route', 'direction',
                                                         'human_look_at_point', 'second_pointing_config'],
                                             output_keys=['target_pose', 'landmarks_to_point', 'human_pose',
                                                          'second_pointing_config'],
                                             server_wait_timeout=rospy.Duration(5))

    def get_name(self):
        return self.__class__.__name__

    def pointing_config_goal_cb(self, userdata, goal):
        rospy.set_param('/pointing_planner/try_no_move', rospy.get_param('/guiding/tuning_param/try_no_move'))
        rospy.set_param('/pointing_planner/settings/mobrob', 1.0)
        rospy.set_param('/pointing_planner/settings/mobhum', 1.0)

        if not PointingConfig.has_been_in_state:
            PointingConfig.has_been_in_state = True
        else:
            userdata.second_pointing_config = True
            PointingConfig.has_been_in_state = False

        landmarks = self.define_svp_planner_lm(userdata.goal_frame, userdata.direction)

        pointing_planner_goal = PointingGoal()
        pointing_planner_goal.human = userdata.person_frame
        pointing_planner_goal.target_landmark = landmarks['goal']
        pointing_planner_goal.direction_landmark = landmarks['direction']
        rospy.loginfo(pointing_planner_goal)
        return pointing_planner_goal

    def pointing_config_feedback_cb(self, userdata, feedback):
        rospy.logdebug("feedback: %s", feedback)

    @staticmethod
    @smach.cb_interface(outcomes=['point_not_visible'],
                        input_keys=['goal_frame', 'landmarks_to_point', 'route', 'direction'],
                        output_keys=['target_pose', 'landmarks_to_point', 'human_pose'])
    def pointing_config_result_cb(userdata, status, result):
        userdata.landmarks_to_point = []
        if status == actionlib.GoalStatus.SUCCEEDED:
            userdata.target_pose = result.robot_pose
            userdata.human_pose = result.human_pose
            GuidingAction.marker_handling.publish_marker(result.human_pose.header.frame_id, result.human_pose.pose,
                                                         HUMAN_POSE)
            GuidingAction.marker_handling.publish_marker(result.robot_pose.header.frame_id, result.robot_pose.pose,
                                                         ROBOT_POSE)

            if userdata.goal_frame in result.pointed_landmarks:
                userdata.landmarks_to_point.append(userdata.goal_frame)
            if userdata.direction in result.pointed_landmarks:
                userdata.landmarks_to_point.append(userdata.direction)
            rospy.loginfo("landmarks to point : %s", userdata.landmarks_to_point)

            if len(userdata.landmarks_to_point) == 0:
                return 'point_not_visible'
            else:
                return 'succeeded'
        else:
            rospy.logwarn("final status : %s", status)
            return 'aborted'


class PointingConfigForRobot(SVPPlannerClient):
    def __init__(self):
        """Constructor for PointingConfig state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach_ros.SimpleActionState.__init__(self, rospy.get_param("/guiding/action_servers/pointing_planner"),
                                             PointingAction, goal_cb=self.pointing_config_goal_cb,
                                             feedback_cb=self.pointing_config_feedback_cb,
                                             result_cb=self.pointing_config_result_cb,
                                             input_keys=['goal_frame', 'person_frame', 'route', 'direction',
                                                         'human_look_at_point'],
                                             output_keys=['target_pose'],
                                             server_wait_timeout=rospy.Duration(5))

    def get_name(self):
        return self.__class__.__name__

    def pointing_config_goal_cb(self, userdata, goal):
        rospy.set_param('/pointing_planner/try_no_move', True)
        rospy.set_param('/pointing_planner/settings/mobhum', 0.0)
        rospy.set_param('/pointing_planner/settings/mobrob', 1.0)

        landmarks = self.define_svp_planner_lm(userdata.goal_frame, userdata.direction)

        pointing_planner_goal = PointingGoal()
        pointing_planner_goal.human = userdata.person_frame
        pointing_planner_goal.target_landmark = landmarks['goal']
        pointing_planner_goal.direction_landmark = landmarks['direction']
        return pointing_planner_goal

    def pointing_config_feedback_cb(self, userdata, feedback):
        pass

    @staticmethod
    @smach.cb_interface(input_keys=['goal_frame'],
                        output_keys=['target_pose'])
    def pointing_config_result_cb(userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            userdata.target_pose = result.robot_pose
            GuidingAction.marker_handling.publish_marker(result.robot_pose.header.frame_id, result.robot_pose.pose,
                                                         ROBOT_POSE)
            return 'succeeded'
        else:
            return 'aborted'


class AreLandmarksVisibleFromHuman(smach.State):
    def __init__(self):
        """Constructor for IsTargetVisibleFrom state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['landmarks_visible', 'landmark_s_not_visible', 'preempted', 'aborted'],
                             input_keys=['person_frame', 'landmarks_to_point', 'second_pointing_config', 'goal_frame',
                                         'direction'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        visible = False
        if userdata.goal_frame in userdata.landmarks_to_point:
            visible = GuidingAction.services_proxy["is_visible"](
                userdata.person_frame, userdata.goal_frame).is_visible
            rospy.logwarn("goal visibility %s", visible)

        # if there is 2 landmarks to point and we know the first one is visible
        if userdata.direction in userdata.landmarks_to_point:
            if userdata.goal_frame in userdata.landmarks_to_point and not visible:
                visible = False
            else:
                visible = GuidingAction.services_proxy["is_visible"](
                    userdata.person_frame, userdata.direction).is_visible
                rospy.logwarn("direction visibility %s", visible)

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        if visible:
            return 'landmarks_visible'
        elif userdata.second_pointing_config:
            rospy.logwarn("human misplaced for second time")
            return 'aborted'
        else:
            return 'landmark_s_not_visible'


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
        smach.State.__init__(self, outcomes=['robot_first', 'human_move', 'no', 'preempted', 'aborted'],
                             input_keys=['person_frame', 'human_pose'], output_keys=['dist_r_h_fut'])
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calculates the distance between the actual human position and the one returned by the pointing planner,
        the distance between the actual robot position and the human's one returned by the pointing planner, then
        estimates the outcome to return."""
        try:
            # calculate the distance between the human actual pose and the human wanted pose
            trans = self._tfBuffer.lookup_transform('map', userdata.person_frame, rospy.Time(0))
            distance_h_now_h_future = math.sqrt(
                (trans.transform.translation.x - userdata.human_pose.pose.position.x) ** 2 +
                (trans.transform.translation.y - userdata.human_pose.pose.position.y) ** 2)
            # rospy.logwarn("dist human now future : %f", distance_h_now_h_future)

            if self.preempt_requested():
                rospy.loginfo(self.get_name() + " preempted")
                self.service_preempt()
                return 'preempted'

            # if the distance is superior to the threshold, the human will be asked to move
            if distance_h_now_h_future > HUMAN_SHOULD_MOVE_DIST_TH:
                try:
                    # calculate the distance between the robot actual pose and the human wanted pose
                    trans = self._tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
                    distance_r_now_h_future = math.sqrt(
                        (trans.transform.translation.x - userdata.human_pose.pose.position.x) ** 2 +
                        (trans.transform.translation.y - userdata.human_pose.pose.position.y) ** 2)
                    # rospy.logwarn("dist robot now human future : %f", distance_r_now_h_future)
                    # if the distance is inferior to the threshold, the human should take the actual robot place,
                    # therefore the robot should move first
                    if distance_r_now_h_future < TAKE_ROBOT_PLACE_DIST_TH:
                        return 'robot_first'
                    # the human wanted pose is at a certain distance of the robot, the human can move first
                    else:
                        userdata.dist_r_h_fut = distance_r_now_h_future
                        return 'human_move'

                except Exception as e:
                    rospy.logerr(e)
                    return 'aborted'

            else:
                return 'no'

        except Exception as e:
            rospy.logerr(e)
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
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['human_look_at_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the speech service to ask the human to move"""

        if HWU_DIAL:
            # say "I am going to move, so you can come to my current place. You will better see from here."
            GuidingAction.services_proxy["dialogue_inform"]("verbalisation.prompt_user_move_to_robot", "")
        else:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                "I am going to move, so you can come to my current place."
                                                "You will better see from here.",
                                                SPEECH_PRIORITY)

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


class AskHumanToFollow(smach.State):

    def __init__(self):

        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'],
                             input_keys=['human_look_at_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the speech service to ask the human to move"""

        # if HWU_DIAL:
        #     # say "I am going to move, so you can come to my current place. You will better see from here."
        #     GuidingAction.services_proxy["dialogue_inform"]("verbalisation.prompt_user_move_to_robot", "")
        # else:
        GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                            "Come with me, you will better see from here.",
                                            SPEECH_PRIORITY)

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
                             input_keys=['landmark_to_point', 'human_look_at_point', 'human_pose', 'dist_r_h_fut',
                                         'second_pointing_config'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the speech, point_at and look_at services"""
        try:
            if userdata.second_pointing_config:
                if HWU_DIAL:
                    # say "I am sorry, maybe I was not clear the first time. You should come there"
                    GuidingAction.services_proxy["dialogue_inform"]("verbalisation.prompt_user_move_repeat", "")
                else:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                        "I am sorry, maybe I was not clear the first time. "
                                                        "You should come there", SPEECH_PRIORITY)
            else:
                dist_str = str(round(userdata.dist_r_h_fut*10)/10)
                if HWU_DIAL:
                    # say "I need you to make a few steps.... You will see better what I am about to show you.
                    # Can you go there ? At about X meters from me"
                    GuidingAction.services_proxy["dialogue_inform"]("verbalisation.prompt_user_move",
                                                                    json.dumps(dist_str))
                else:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                        "I need you to make a few steps.... You will see better "
                                                        "what I am about to show you. Can you go there ? At about " +
                                                        dist_str + "meters from me.",
                                                        SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr(e)

        human_point_stamped = PointStamped()
        human_point_stamped.point = userdata.human_pose.pose.position
        human_point_stamped.header = userdata.human_pose.header
        can_look_at_resp = GuidingAction.services_proxy["can_look_at"](human_point_stamped)
        can_point_at_resp = GuidingAction.services_proxy["can_point_at"](human_point_stamped)

        if not can_look_at_resp.success or not can_point_at_resp.success:
            if not can_look_at_resp.success:
                rospy.logwarn("cannot look")
                GuidingAction.sm_test_rotation.userdata.rotation = can_look_at_resp.angle

            if not can_point_at_resp.success:
                rospy.logwarn("cannot point")
                GuidingAction.sm_test_rotation.userdata.rotation = can_point_at_resp.angle

            GuidingAction.sm_test_rotation.userdata.rotation = can_point_at_resp.angle
            GuidingAction.sm_test_rotation.set_initial_state(['Rotate'])
            GuidingAction.sm_test_rotation.execute()

        coord_signal = CoordinationSignal()
        coord_signal.header.frame_id = userdata.human_pose.header.frame_id
        target = TargetWithExpiration()
        target.target.header.frame_id = userdata.human_pose.header.frame_id
        target.target.point = userdata.human_pose.pose.position
        target.duration = rospy.Duration(2)
        coord_signal.targets.append(target)
        coord_signal.priority = 100
        coord_signal.expiration = rospy.Time() + rospy.Duration(2)
        # coord_signal.regex_end_condition = "isPerceiving\(robot," + userdata.human_pose.header.frame_id + "\)"
        coord_signal.predicate = "isWaiting(robot," + userdata.human_pose.header.frame_id + ")"
        GuidingAction.coord_signals_publisher.publish(coord_signal)

        point_at_request = PointAtRequest()
        point_at_request.point.header = userdata.human_pose.header
        point_at_request.point.point = userdata.human_pose.pose.position
        point_at = GuidingAction.services_proxy["point_at"](point_at_request)

        rospy.sleep(POINTING_DURATION)

        GuidingAction.services_proxy["rest_arm"]("Arms")

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        if point_at.success:
            return 'succeeded'
        else:
            rospy.logerr("pointing service failed")
            return 'aborted'


class Gesture(smach.State):

    def send_coordination_signal(self, frame, duration, point_stamped=None, priority=100, end_condition="",
                                 predicate=""):
        coord_signal = CoordinationSignal()
        coord_signal.header.frame_id = frame
        target = TargetWithExpiration()
        if point_stamped is not None:
            target.target = point_stamped
        else:
            target.target.header.frame_id = frame
        target.duration = duration
        coord_signal.targets.append(target)
        coord_signal.priority = priority
        coord_signal.expiration = rospy.Time() + duration
        if end_condition != "":
            coord_signal.regex_end_condition = end_condition
        if predicate != "":
            coord_signal.predicate = predicate
        GuidingAction.coord_signals_publisher.publish(coord_signal)

    def call_can_point_at(self, frame):
        point_stamped = PointStamped()
        point_stamped.header.frame_id = frame
        can_point_at_resp = GuidingAction.services_proxy["can_point_at"](point_stamped)
        return can_point_at_resp

    def call_can_look_at(self, frame):
        point_stamped = PointStamped()
        point_stamped.header.frame_id = frame
        can_look_at_resp = GuidingAction.services_proxy["can_look_at"](point_stamped)
        return can_look_at_resp

    def point_at(self, frame):
        point_at_request = PointAtRequest()
        point_at_request.point.header.frame_id = frame
        point_at_resp = GuidingAction.services_proxy["point_at"](point_at_request)
        return point_at_resp.success

    def execute_rotation_deic_gest(self, angle):
        GuidingAction.sm_test_rotation.userdata.rotation = angle
        GuidingAction.sm_test_rotation.set_initial_state(['Rotate'])
        GuidingAction.sm_test_rotation.execute()


class LookAtAssumedPlace(Gesture):
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
        smach.State.__init__(self, outcomes=['succeeded', 'look_again', 'preempted', 'human_lost'],
                             input_keys=['human_pose', 'human_look_at_point', 'person_frame'])
        self.first = False
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Calls the look_at service, check if the human is perceived and may speak according to the perception
        result"""

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        look_at = PointStamped()
        look_at.header.frame_id = copy.deepcopy(userdata.human_pose.header.frame_id)
        look_at.point = copy.deepcopy(userdata.human_pose.pose.position)
        trans = self._tfBuffer.lookup_transform('map', userdata.person_frame, rospy.Time(0))
        look_at.point.z += trans.transform.translation.z
        can_look_at_resp = GuidingAction.services_proxy["can_look_at"](look_at)

        if not can_look_at_resp.success:
            rospy.logwarn("cannot look")

            self.execute_rotation_deic_gest(can_look_at_resp.angle)

        self.send_coordination_signal(frame=look_at.header.frame_id, point_stamped=look_at, duration=rospy.Duration(1))
        pose = Pose()
        pose.position = look_at.point
        GuidingAction.marker_handling.publish_marker(look_at.header.frame_id, pose, LOOK_AT_ASSUMED_PLACE)

        rospy.sleep(1)

        if not human_perceived:
            if LookAtAssumedPlace.does_not_see < 5:
                LookAtAssumedPlace.does_not_see += 1
                return 'look_again'
            else:
                LookAtAssumedPlace.does_not_see = 0
                return 'human_lost'
        else:
            LookAtAssumedPlace.does_not_see = 0
            return 'succeeded'


class CheckPerceived(smach.State):
    does_not_see = 0

    def __init__(self):
        """Constructor for LookAtHumanAssumedPlace state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['succeeded', 'wait', 'preempted', 'aborted'],
                             input_keys=['human_pose', 'human_look_at_point', 'person_frame'])
        self.first = False
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        CheckPerceived.does_not_see += 1
        if not human_perceived:
            if CheckPerceived.does_not_see < 6:
                if CheckPerceived.does_not_see == 1:
                    if HWU_DIAL:
                        GuidingAction.services_proxy["dialogue_inform"]("verbalisation. user_not_visible_prompt", "")
                    else:
                        GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                            "I am sorry, I cannot see you. Where are you ?",
                                                            SPEECH_PRIORITY)
                        rospy.sleep(0.5)
                # if self.does_not_see == 2:
                #     GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                #                                         "I still cannot see you but I will assume that you are here",
                #                                         SPEECH_PRIORITY)
                return 'wait'
            else:
                CheckPerceived.does_not_see = 0
                return 'aborted'
        else:
            CheckPerceived.does_not_see = 0
            return 'succeeded'


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


class ShouldRobotMove(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['yes', 'no', 'preempted', 'aborted'],
                             input_keys=['target_pose'])
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'
        try:
            trans = self._tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            distance_r_now_r_future = math.sqrt(
                (trans.transform.translation.x - userdata.target_pose.pose.position.x) ** 2 +
                (trans.transform.translation.y - userdata.target_pose.pose.position.y) ** 2)
            if distance_r_now_r_future > ROBOT_SHOULD_MOVE_DIST_TH:
                return 'yes'
            else:
                return 'no'
        except Exception as e:
            rospy.logerr(e)
            return 'aborted'


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
        smach_ros.SimpleActionState.__init__(self, rospy.get_param("/guiding/action_servers/move_to"),
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
        # try:
        #     if HWU_DIAL:
        #         # say : "Now wait, I am going to move."
        #         GuidingAction.services_proxy["dialogue_inform"]("verbalisation.robot_move", '')
        #     else:
        #         GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Now wait, I am going to move.",
        #                                             SPEECH_PRIORITY)
        #
        # except rospy.ServiceException, e:
        #     rospy.logerr("speech exception")
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


class GetRotationAngle(SVPPlannerClient):
    def __init__(self):
        """Constructor for PointingConfig state

        It calls the super constructor of L{smach.State} and define the outcomes, the input_keys, the output_keys
        and the io_keys of the state.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        self.direction_landmark = ""
        self.mob_param = rospy.get_param('/pointing_planner/settings/mobhum')
        smach_ros.SimpleActionState.__init__(self, rospy.get_param("/guiding/action_servers/pointing_planner"),
                                             PointingAction, goal_cb=self.pointing_config_goal_cb,
                                             feedback_cb=self.pointing_config_feedback_cb,
                                             result_cb=self.pointing_config_result_cb,
                                             input_keys=['goal_frame', 'person_frame', 'route',
                                                         'human_look_at_point', 'direction'],
                                             output_keys=['rotation'],
                                             server_wait_timeout=rospy.Duration(5))

    def get_name(self):
        return self.__class__.__name__

    def pointing_config_goal_cb(self, userdata, goal):
        rospy.set_param('/pointing_planner/try_no_move', True)
        rospy.set_param('/pointing_planner/settings/mobrob', 0.0)
        rospy.set_param('/pointing_planner/settings/mobhum', 0.0)

        landmarks = self.define_svp_planner_lm(userdata.goal_frame, userdata.direction)

        pointing_planner_goal = PointingGoal()
        pointing_planner_goal.human = userdata.person_frame
        pointing_planner_goal.target_landmark = landmarks['goal']
        pointing_planner_goal.direction_landmark = landmarks['direction']
        return pointing_planner_goal

    def pointing_config_feedback_cb(self, userdata, feedback):
        pass

    @staticmethod
    @smach.cb_interface(input_keys=['goal_frame', 'rotation'],
                        output_keys=['rotation'])
    def pointing_config_result_cb(userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            # write in the userdata
            quaternion_stamped = QuaternionStamped()
            quaternion_stamped.header = result.robot_pose.header
            quaternion_stamped.quaternion = result.robot_pose.pose.orientation
            userdata.rotation = quaternion_stamped
            return 'succeeded'
        else:
            return 'aborted'


class RotateRobot(smach_ros.SimpleActionState):
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
        smach_ros.SimpleActionState.__init__(self, rospy.get_param("/guiding/action_servers/rotate"),
                                             RotateAction, goal_cb=self.rotate_goal_cb,
                                             feedback_cb=self.rotate_feedback_cb,
                                             result_cb=self.rotate_result_cb,
                                             input_keys=['rotation', 'goal_frame'],
                                             server_wait_timeout=rospy.Duration(2)),

    def get_name(self):
        return self.__class__.__name__

    def rotate_goal_cb(self, userdata, goal):
        """Announces to the human that it is about to move. Then, creates a MoveBaseGoal to the to the action server
        with the pose the robot should reach."""

        rotate_goal = RotateGoal()
        if GuidingAction.sm_test_rotation.get_initial_states() == ['Rotate']:
            angle = QuaternionStamped()
            angle.header.frame_id = 'base_footprint'
            angle.quaternion.x, angle.quaternion.y, angle.quaternion.z, angle.quaternion.w = \
                transform.quaternion_from_euler(0, 0, userdata.rotation)

            rotate_goal.rotation = angle
        else:
            rotate_goal.rotation = userdata.rotation
            # if svp planner did not return a solution to the rotation
            if userdata.rotation.header.frame_id == "":
                point_stamped = PointStamped()
                point_stamped.header.frame_id = userdata.goal_frame

                can_point_at_resp = GuidingAction.services_proxy["can_point_at"](point_stamped)
                angle = QuaternionStamped()
                angle.header.frame_id = 'base_footprint'
                angle.quaternion.x, angle.quaternion.y, angle.quaternion.z, angle.quaternion.w = \
                    transform.quaternion_from_euler(0, 0, can_point_at_resp.angle)
                rotate_goal.rotation = angle
        return rotate_goal

    def rotate_feedback_cb(self, userdata, feedback):
        # rospy.loginfo(feedback.base_position)
        pass

    def rotate_result_cb(self, userdata, status, result):
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
                             input_keys=['goal_frame', 'route', 'landmarks_to_point', 'human_look_at_point', 'direction'],
                             output_keys=['landmark_to_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """It computes what should be point at."""

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        # if it is the first pass in the state, the target has not been pointed at yet
        if not SelectLandmark.target_pointed:
            SelectLandmark.target_pointed = True
            # the target is defined as the next landmark to point at, it's written in the userdata
            userdata.landmark_to_point = (userdata.goal_frame, LANDMARK_TYPE_TARGET)
            # if the target has been returned by the pointing planner
            if userdata.goal_frame in userdata.landmarks_to_point:
                return 'point_target'
            # if the pointing planner decided that the target is not visible (no target in landmarks_to_point)
            else:
                return 'point_not_visible'
        # second pass in the state, the direction has to be pointed
        else:
            userdata.landmark_to_point = (userdata.direction, LANDMARK_TYPE_DIRECTION)
            if userdata.direction in userdata.landmarks_to_point:
                return 'point_direction'
            else:
                return 'point_not_visible'


class Pointing(Gesture):

    TARGET_SAME_AREA = 0
    TARGET_DIFF_AREA_NO_SIGN = 1
    TARGET_DIFF_AREA_W_SIGN = 2
    DIRECTION = 3

    def should_rotate_to_act(self, frame):
        can_point_at = self.call_can_point_at(frame).success
        can_look_at = self.call_can_look_at(frame).success
        if not can_look_at or not can_point_at:
            if not can_look_at:
                rospy.logwarn("cannot look")
            if not can_point_at:
                rospy.logwarn("cannot point")
            return True

    def execute_rotation_try_svp_angle(self, userdata):
        GuidingAction.sm_test_rotation.userdata.route = userdata.route
        GuidingAction.sm_test_rotation.userdata.goal_frame = userdata.goal_frame
        GuidingAction.sm_test_rotation.userdata.person_frame = userdata.person_frame
        GuidingAction.sm_test_rotation.userdata.human_look_at_point = userdata.human_look_at_point
        GuidingAction.sm_test_rotation.userdata.direction = userdata.direction
        GuidingAction.sm_test_rotation.set_initial_state(['GetRotationAngle'])
        outcome = GuidingAction.sm_test_rotation.execute()
        return outcome

    def select_speech_case(self, landmark_to_point_type, route, signpost):
        case = Pointing.TARGET_SAME_AREA
        # if the direction does not exist (target in same area)
        if landmark_to_point_type == LANDMARK_TYPE_TARGET and \
                len(route) == NO_INTERFACE_LEN:
            case = Pointing.TARGET_SAME_AREA
        # if both exists (target in a different area)
        elif landmark_to_point_type == LANDMARK_TYPE_TARGET:
            if not signpost:
                case = Pointing.TARGET_DIFF_AREA_NO_SIGN
            else:
                case = Pointing.TARGET_DIFF_AREA_W_SIGN
        # point the direction if it exists, no matter the existence of the target
        elif landmark_to_point_type == LANDMARK_TYPE_DIRECTION:
            case = Pointing.DIRECTION

        return case

    def speech(self, case, visible, userdata):
        if HWU_DIAL:
            if case == Pointing.TARGET_SAME_AREA or case == Pointing.DIRECTION:
                route_description = GuidingAction.services_proxy["route_verbalization"](
                    userdata.route, ROBOT_PLACE, userdata.landmark_to_point[LANDMARK_NAME]).region_route
                GuidingAction.services_proxy["dialogue_inform"]("execute.route_description_plain",
                                                                json.dumps(route_description))
            elif case == Pointing.TARGET_DIFF_AREA_NO_SIGN:
                GuidingAction.services_proxy["dialogue_inform"]("verbalisation.location_show",
                                                                json.dumps([userdata.landmark_to_point[
                                                                                LANDMARK_NAME], case]))
        else:
            if case == Pointing.TARGET_SAME_AREA or case == case == Pointing.DIRECTION:
                route_description = GuidingAction.services_proxy["route_verbalization"](
                    userdata.route_2_shop_wo_sign, ROBOT_PLACE, userdata.target_frame).region_route
                GuidingAction.services_proxy["say"](userdata.human_look_at_point, route_description,
                                                    SPEECH_PRIORITY)
                rospy.logwarn(route_description)
            elif case == Pointing.TARGET_DIFF_AREA_NO_SIGN:
                if visible:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Look, it's there",
                                                        SPEECH_PRIORITY)
                else:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point, "It's in this direction",
                                                        SPEECH_PRIORITY)
            elif case == Pointing.TARGET_DIFF_AREA_W_SIGN:
                if visible:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Follow the sign there",
                                                        SPEECH_PRIORITY)
                else:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point, "The sign you should follow is in"
                                                                                      "this direction",
                                                        SPEECH_PRIORITY)


class PointNotVisible(Pointing):
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
                             input_keys=['landmark_to_point', 'human_look_at_point', 'route_2_shop_wo_sign',
                                         'goal_frame', 'person_frame', 'persona', 'signpost', 'target_frame'])
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Performs the pointing in the landmark direction if it is possible, otherwise announces it will show the
        passage """

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        # if the landmark has a tf frame
        if self._tfBuffer.can_transform('map', userdata.landmark_to_point[LANDMARK_NAME], rospy.Time(0)):

            can_point_at_resp = self.call_can_point_at(userdata.landmark_to_point[LANDMARK_NAME])

            if not can_point_at_resp.success:
                rospy.logwarn("try to rotate because cannot point")
                self.execute_rotation_deic_gest(can_point_at_resp.angle)

            point_at_success = self.point_at(userdata.landmark_to_point[LANDMARK_NAME])

            case = self.select_speech_case(userdata.landmark_to_point[LANDMARK_TYPE], userdata.route_2_shop_wo_sign,
                                           userdata.signpost)

            self.speech(case, False, userdata)

            rospy.sleep(POINTING_DURATION)
            GuidingAction.services_proxy["rest_arm"]("Arms")

            if point_at_success:
                return 'succeeded'
            else:
                return 'aborted'
        else:
            rospy.logwarn(userdata.landmark_to_point[LANDMARK_NAME] + " has no tf transform")
            return 'succeeded'


class PointAndLookAtLandmark(Pointing):
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
                             input_keys=['person_frame', 'goal_frame', 'landmark_to_point', 'human_look_at_point',
                                         'route', 'persona', 'route_2_shop_wo_sign', 'direction', 'signpost',
                                         'target_frame'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Point, speak, look, init pose"""

        if self.should_rotate_to_act(userdata.landmark_to_point[LANDMARK_NAME]):
            self.execute_rotation_try_svp_angle(userdata)

        self.send_coordination_signal(userdata.landmark_to_point[LANDMARK_NAME], rospy.Duration(1))
        pose = Pose()
        GuidingAction.marker_handling.publish_marker(userdata.landmark_to_point[LANDMARK_NAME], pose, LOOK_AT_LANDMARK)

        point_at_success = self.point_at(userdata.landmark_to_point[LANDMARK_NAME])

        case = self.select_speech_case(userdata.landmark_to_point[LANDMARK_TYPE], userdata.route_2_shop_wo_sign,
                                       userdata.signpost)

        self.speech(case, True, userdata)

        rospy.sleep(POINTING_DURATION)

        GuidingAction.services_proxy["rest_arm"]("Arms")

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        if point_at_success:
            return 'succeeded'
        else:
            rospy.logwarn("point at success false")
            return 'aborted'


class LookAtHuman(Gesture):
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

        can_look_at_resp = self.call_can_look_at(userdata.person_frame)
        if not can_look_at_resp.success:
            rospy.logwarn("cannot look")
            GuidingAction.sm_test_rotation.userdata.rotation = can_look_at_resp.angle
            GuidingAction.sm_test_rotation.set_initial_state(['Rotate'])
            GuidingAction.sm_test_rotation.execute()

        self.send_coordination_signal(userdata.person_frame, rospy.Duration(0, 1))
        pose = Pose()
        GuidingAction.marker_handling.publish_marker(userdata.person_frame, pose, LOOK_AT_HUMAN)

        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        rospy.sleep(0.5)

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
        smach.State.__init__(self, outcomes=['get_answer', 'seen', 'no', 'preempted'],
                             input_keys=['human_look_at_point'], output_keys=['question_asked'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Speak"""
        userdata.question_asked = 'ask_seen'
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'
        try:
            if HWU_DIAL:
                # say : "Have you seen where you have to go ?"
                answer = GuidingAction.services_proxy["dialogue_query"]("clarification.route_confirm", "")
                if answer.result == 'true':
                    return 'seen'
                else:
                    # say : "Oh it's too bad, I'm sorry I wasn't good enough"
                    GuidingAction.services_proxy["dialogue_inform"]("verbalisation.apology", '')
                    return 'no'
            else:
                GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                    "Have you seen where you have to go ?", SPEECH_PRIORITY)
                return 'get_answer'
        except rospy.ServiceException, e:
            rospy.logerr(e)


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
        smach.State.__init__(self, outcomes=['get_answer', 'pointing', 'no', 'preempted'],
                             input_keys=['human_look_at_point'], output_keys=['question_asked'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Speak"""
        userdata.question_asked = 'ask_point_again'
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        try:
            if HWU_DIAL:
                # say : Should I show you the direction again ?
                answer = GuidingAction.services_proxy["dialogue_query"]("clarification.route_repeat", "")
                if answer.result == 'true':
                    return 'pointing'
                else:
                    # say : "Ok, too bad. Good luck !"
                    GuidingAction.services_proxy["dialogue_inform"]("verbalisation.disengage", '')
                    return 'no'
            else:
                GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                    "Should I show you the direction again ?", SPEECH_PRIORITY)
                return 'get_answer'
        except rospy.ServiceException, e:
            rospy.logerr(e)


class GetAnswer(smach_ros.SimpleActionState):
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
        """Constructor for GetAnswer state

        It calls the super constructor of L{smach.State} and defines the outcomes, the input_keys, the output_keys,
        the 3 action state callbacks, each one for the goal, the feedback and the result, a execution timeout (no answer
        from the action server since X seconds) and a server timeout.

        """
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach_ros.SimpleActionState.__init__(self, rospy.get_param("/guiding/action_servers/dialogue"),
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
        dialogue_goal = dialogue_actionGoal()
        # the action server has to catch the words 'yes' and 'no'
        dialogue_goal.subjects = ['yes', 'no', 'stairs', 'elevator']
        dialogue_goal.enable_only_subject = True
        return dialogue_goal

    def dialogue_feedback_cb(self, userdata, feedback):
        # the human said something else than yes or no, an unexpected word
        self._activate_time = rospy.Time.now()
        try:
            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                "I'm sorry I did not understand what you said",
                                                SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr(e)
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
                        try:
                            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                                "Sorry I can't hear you, can you repeat ?",
                                                                SPEECH_PRIORITY)
                        except rospy.ServiceException, e:
                            rospy.logerr(e)
                    # if it is the second time to ask to repeat
                    elif self.repeat_question == 1:
                        try:
                            GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                                "I am really sorry I still can not hear you, "
                                                                "can you speak louder ?",
                                                                SPEECH_PRIORITY)
                        except rospy.ServiceException, e:
                            rospy.logerr(e)

                    self.repeat_question += 1
                    action_success = 'preempted'
                # it already asked 2 times to repeat
                else:
                    userdata.result_word = result.subject
                    try:
                        GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                            "I did not hear, sorry, I give up",
                                                            SPEECH_PRIORITY)
                    except rospy.ServiceException, e:
                        rospy.logerr(e)
                    action_success = 'aborted'
            # if the node received a kill request
            else:
                action_success = 'aborted'
        else:
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
        smach.State.__init__(self, outcomes=['show', 'ask_stairs_or_elevator', 'no', 'preempted'],
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
                    rospy.logerr(e)
                next_action = 'no'
        elif userdata.question_asked == 'ask_show_direction':
            if userdata.result_word == 'yes':
                next_action = 'ask_stairs_or_elevator'
            else:
                try:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Ok, I hope you will get there!",
                                                        SPEECH_PRIORITY)
                except rospy.ServiceException, e:
                    rospy.logerr(e)
                next_action = 'no'

        return next_action


class DispatchStairsElevator(smach.State):
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
        smach.State.__init__(self, outcomes=['stairs', 'elevator', 'preempted'],
                             input_keys=['question_asked', 'result_word', 'human_look_at_point'],
                             output_keys=['persona'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Dispatch the question-answer couples"""
        next_action = 'preempted'
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            next_action = 'preempted'
        elif userdata.question_asked == 'ask_stairs_or_elevator':
            if userdata.result_word == 'elevator':
                userdata.persona = "old"
                next_action = 'elevator'
            else:
                next_action = 'stairs'
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
                    rospy.logerr(e)
                next_action = 'yes'
            else:
                try:
                    GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                        "Oh it's too bad, I'm sorry I wasn't good enough",
                                                        SPEECH_PRIORITY)
                except rospy.ServiceException, e:
                    rospy.logerr(e)
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
                             input_keys=['landmark_to_point', 'landmarks_to_point', 'last_outcome', 'route', 'direction',
                                         'human_look_at_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        """Computes if there is the direction to point or if the task is over"""
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'

        if userdata.direction != "" and userdata.landmark_to_point[LANDMARK_TYPE] == LANDMARK_TYPE_TARGET:
            return 'point_direction'
        # if the human does want to see the landmark again
        elif userdata.last_outcome == 'no':
            return 'aborted'
        # what had to be shown is shown properly
        else:
            if not HWU_DIAL:
                GuidingAction.services_proxy["say"](userdata.human_look_at_point, "Have a nice day", SPEECH_PRIORITY)
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


class Timer2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])

    def execute(self, userdata):
        activate_time = rospy.Time.now()
        while not self.preempt_requested():
            rospy.sleep(0.1)
            if rospy.Time.now() - activate_time > rospy.Duration(5):
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
            if HWU_DIAL:
                # say : "I saw that you have seen where you have to go. Awesome !"
                GuidingAction.services_proxy["dialogue_inform"]("verbalisation.user_visible_confirmation", '')
            else:
                GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                    "I saw that you have seen where you have to go. Awesome !",
                                                    SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr(e)
        if self.preempt_requested():
            rospy.loginfo(self.get_name() + " preempted")
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


class HumanLost(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of " + self.get_name() + " state")
        smach.State.__init__(self, outcomes=['human_lost'], input_keys=['human_look_at_point'])

    def get_name(self):
        return self.__class__.__name__

    def execute(self, userdata):
        try:
            if HWU_DIAL:
                # say : "I am sorry, I am not able to see you anymore. Bye bye."
                GuidingAction.services_proxy["dialogue_inform"]("verbalisation.user_not_visible_disengage", '')
            else:
                GuidingAction.services_proxy["say"](userdata.human_look_at_point,
                                                    "Oh no, you left ! Too bad.",
                                                    SPEECH_PRIORITY)
        except rospy.ServiceException, e:
            rospy.logerr(e)

        return 'human_lost'


if __name__ == '__main__':
    if rospy.get_param('guiding/debug/log_level') == "DEBUG":
        rospy.init_node('guiding_action_server', log_level=rospy.DEBUG)
    else:
        rospy.init_node('guiding_action_server')
    server = GuidingAction('/guiding_task')
    rospy.on_shutdown(server.stand_pose)
    rospy.spin()

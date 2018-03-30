#! /usr/bin/env python

import rospy
import actionlib
import smach
import smach_ros
from nao_interaction_msgs.srv import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

import actionlib_tutorials.msg

import tasks.msg

from dialogue_as.msg import *

from deictic_gestures.srv import *


class ShowAction(object):
    _result = tasks.msg.taskResult()

    feedback = tasks.msg.taskFeedback()
    action_server = None
    dialogue_client = None
    services_proxy = {}

    def __init__(self, name):
        self._action_name = name

        ShowAction.action_server = actionlib.SimpleActionServer(self._action_name, tasks.msg.taskAction,
                                                                execute_cb=self.execute_cb,
                                                                auto_start=False)

        rospy.loginfo("waiting for service /naoqi_driver/tts/say")
        rospy.wait_for_service("/naoqi_driver/tts/say")

        rospy.loginfo("waiting for service /naoqi_driver/robot_posture/go_to_posture")
        rospy.wait_for_service("/naoqi_driver/robot_posture/go_to_posture")

        rospy.loginfo("waiting for service /deictic_gestures/get_pointing_config")
        rospy.wait_for_service('/deictic_gestures/get_pointing_config')

        rospy.loginfo("waiting for service /deictic_gestures/look_at")
        rospy.wait_for_service('/deictic_gestures/look_at')

        rospy.loginfo("waiting for service /deictic_gestures/point_at")
        rospy.wait_for_service('/deictic_gestures/point_at')

        rospy.loginfo("waiting for service /activate")
        rospy.wait_for_service('/activate')

        rospy.loginfo("waiting for service /deactivate")
        rospy.wait_for_service('/deactivate')


        ShowAction.services_proxy = {
            "stand_pose": rospy.ServiceProxy('/naoqi_driver/robot_posture/go_to_posture',
                                      nao_interaction_msgs.srv.GoToPosture),
            "say": rospy.ServiceProxy('/naoqi_driver/tts/say', nao_interaction_msgs.srv.Say),
            "get_pointing_config": rospy.ServiceProxy('/deictic_gestures/get_pointing_config', GetPointingConfig),
            "look_at": rospy.ServiceProxy('/deictic_gestures/look_at', LookAt),
            "point_at": rospy.ServiceProxy('/deictic_gestures/point_at', PointAt),
            "activate_dialogue": rospy.ServiceProxy('/activate', Trigger),
            "deactivate_dialogue": rospy.ServiceProxy('/deactivate', Trigger)}

        rospy.loginfo("start action server")
        ShowAction.action_server.start()
        rospy.loginfo("action server started")

        ShowAction.services_proxy["deactivate_dialogue"]()

    def h_m_c_child_term_cb(self, outcome_map):
        # if outcome_map['HUMAN_MONITOR'] == 'valid':
        #     return False
        # else:
        #     return True
        return False

    def h_m_c_out_cb(self, outcome_map):
        if outcome_map['GUIDING'] == 'task_succeeded':
            return 'task_succeeded'
        else:
            return 'task_failed'

    def human_monitor_cb(self, ud, msg):
        if msg.data:
            return True
        else:
            return False

    def guiding_start_cb(self, userdata, initial_states, *cb_args):
        userdata.last_guiding_state = initial_states[0]

    def guiding_transition_cb(self, userdata, active_states, *cb_args):
        if active_states[0] != 'Failure':
            userdata.last_guiding_state = active_states[0]

    def execute_cb(self, goal):

        ShowAction.feedback.current_step = "Start state machine"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        guiding_sm = smach.StateMachine(outcomes=['task_succeeded', 'task_failed', 'preempted'],
                                        output_keys=['last_guiding_state'])
        guiding_sm.userdata.person_frame = goal.person_frame
        guiding_sm.userdata.place_frame = goal.place_frame
        guiding_sm.userdata.last_guiding_state = None

        # Open the container
        with guiding_sm:

            smach.StateMachine.add('PointingConfig', PointingConfig(),
                                   transitions={'succeeded': 'MoveToPose', 'aborted': 'Failure'})

            smach.StateMachine.add('MoveToPose', MoveToPose(),
                                   transitions={'succeeded': 'LookAtLandmark',
                                                'preempted': 'Failure', 'aborted': 'Failure'})

            smach.StateMachine.add('LookAtLandmark', LookAtLandmark(),
                                   transitions={'succeeded': 'PointAtLandmark', 'aborted': 'Failure'})

            smach.StateMachine.add('PointAtLandmark', PointAtLandmark(),
                                   transitions={'succeeded': 'LookAtHuman', 'aborted': 'Failure'})

            smach.StateMachine.add('LookAtHuman', LookAtHuman(),
                                   transitions={'succeeded': 'CheckLandmarkSeen', 'aborted': 'Failure'})

            check_landmark_seen_sm = smach.StateMachine(outcomes=['yes', 'no', 'pointing', 'preempted', 'failure'])

            with check_landmark_seen_sm:
                check_landmark_seen_sm.add('AskSeen', AskSeen(), transitions={'succeeded': 'GetYesNo'})
                check_landmark_seen_sm.add('AskPointAgain', AskPointAgain(), transitions={'succeeded': 'GetYesNo'})
                check_landmark_seen_sm.add('GetYesNo', GetYesNo(),
                                           transitions={'succeeded': 'DispatchYesNo', 'preempted': 'GetYesNo',
                                                        'aborted': 'failure'})
                check_landmark_seen_sm.add('DispatchYesNo', DispatchYesNo(),
                                           transitions={'yes': 'yes',
                                                        'no': 'no',
                                                        'ask_point_again': 'AskPointAgain',
                                                        'pointing': 'pointing'})

            smach.StateMachine.add('CheckLandmarkSeen', check_landmark_seen_sm,
                                    transitions={'yes': 'task_succeeded', 'no': 'Failure',
                                                 'pointing': 'PointAtLandmark',
                                                 'failure': 'Failure'})

            smach.StateMachine.add('Failure', Failure(), transitions={'aborted': 'task_failed'})

        guiding_sm.register_transition_cb(self.guiding_transition_cb, cb_args=[])
        guiding_sm.register_start_cb(self.guiding_start_cb, cb_args=[])

        human_monitor_concurrence = smach.Concurrence(outcomes=['task_succeeded', 'task_failed'],
                                                      default_outcome='task_failed',
                                                      outcome_cb=self.h_m_c_out_cb,
                                                      child_termination_cb=self.h_m_c_child_term_cb)

        with human_monitor_concurrence:
            smach.Concurrence.add('GUIDING', guiding_sm)
            #smach.Concurrence.add('HUMAN_MONITOR', smach_ros.MonitorState("/sm_reset", Bool, self.human_monitor_cb))

        top_sm = smach.StateMachine(outcomes=['task_succeeded', 'task_failed'])
        with top_sm:
            smach.StateMachine.add('CONCURRENCE', human_monitor_concurrence)

        sis = smach_ros.IntrospectionServer('server_name', top_sm, '/SM_ROOT')
        sis.start()
        smach_ros.set_preempt_handler(top_sm)
        outcome = top_sm.execute()

        ShowAction.feedback.current_step = "Go to stand pose"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)

        rospy.loginfo("State machine outcome : %s ", outcome)
        if outcome == 'task_succeeded':
            self._result.success = True
        else:
            self._result.success = False
            if guiding_sm.userdata.last_guiding_state is None:
                self._result.failure_reason = "SM did not start"
            else:
                self._result.failure_reason = guiding_sm.userdata.last_guiding_state

        ShowAction.action_server.set_succeeded(self._result)

        # sis.stop()


    def stand_pose(self):
        rospy.loginfo("stand pose")
        go_to_posture_request = GoToPostureRequest()
        go_to_posture_request.posture_name = "StandInit"
        go_to_posture_request.speed = 1
        ShowAction.services_proxy["stand_pose"](go_to_posture_request)


class PointingConfig(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of PointingConfig state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['place_frame', 'person_frame'],
                             output_keys=['target_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PointingConfig')
        ShowAction.feedback.current_step = "get_pointing_config"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        target_point_stamped = PointStamped()
        target_point_stamped.header.frame_id = userdata.place_frame

        get_pointing_config = ShowAction.services_proxy["get_pointing_config"](
            userdata.person_frame,
            target_point_stamped,
            rospy.get_param("/pointing_config/distance_to_robot"),
            rospy.get_param("/pointing_config/alpha"))
        userdata.target_pose = get_pointing_config.result_pose

        if self.preempt_requested():
            rospy.loginfo("PointingConfig preempted")
            self.service_preempt()
            return 'preempted'

        if get_pointing_config.success:
            return 'succeeded'
        else:
            return 'aborted'


class MoveToPose(smach_ros.SimpleActionState):
    def __init__(self):
        rospy.loginfo("Initialization of MoveToPose state")
        smach_ros.SimpleActionState.__init__(self, 'm_move_to_test', MoveBaseAction, goal_cb=self.move_to_goal_cb,
                                             feedback_cb=self.move_to_feedback_cb,
                                             result_cb=self.move_to_result_cb,
                                             input_keys=['target_pose'],
                                             server_wait_timeout=rospy.Duration(2.0)),

    def move_to_goal_cb(self, userdata, goal):
        rospy.loginfo('Executing state MoveToPose')
        ShowAction.services_proxy["say"]("I'm going to show you the shop")
        ShowAction.feedback.current_step = "moving"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        move_to_goal = MoveBaseGoal()
        move_to_goal.target_pose = userdata.target_pose
        return move_to_goal

    def move_to_feedback_cb(self, userdata, feedback):
        rospy.loginfo(feedback.base_position)

    def move_to_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        else:
            return 'aborted'


class LookAtLandmark(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of LookAtLandmark state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=['place_frame'])

    def execute(self, userdata):
        ShowAction.services_proxy["say"]("Look")
        ShowAction.feedback.current_step = "Look at the landmark"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        look_at_request = LookAtRequest()
        look_at_request.point.header.frame_id = userdata.place_frame
        look_at = ShowAction.services_proxy["look_at"](look_at_request)
        if self.preempt_requested():
            rospy.loginfo("PointingConfig preempted")
            self.service_preempt()
            return 'preempted'

        if look_at.success:
            return 'succeeded'
        else:
            return 'aborted'


class PointAtLandmark(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of PointAtLandmark state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=['place_frame'])

    def execute(self, userdata):
        ShowAction.feedback.current_step = "Point at the landmark"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        point_at_request = PointAtRequest()
        point_at_request.point.header.frame_id = userdata.place_frame
        point_at = ShowAction.services_proxy["point_at"](point_at_request)
        if self.preempt_requested():
            rospy.loginfo("PointingConfig preempted")
            self.service_preempt()
            return 'preempted'

        if point_at.success:
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
            rospy.loginfo("PointingConfig preempted")
            self.service_preempt()
            return 'preempted'

        if look_at.success:
            return 'succeeded'
        else:
            return 'aborted'


class AskSeen(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of AskSeen state")
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], output_keys=['question_asked'])

    def execute(self, userdata):
        rospy.loginfo('Executing state AskSeen')
        ShowAction.services_proxy["say"]("Have you seen the shop ?")
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
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], output_keys=['question_asked'])

    def execute(self, userdata):
        rospy.loginfo('Executing state AskPointAgain')
        ShowAction.services_proxy["say"]("Should I show you the shop again ?")
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
        smach_ros.SimpleActionState.__init__(self, 'dialogue_as', dialogue_as.msg.dialogue_actionAction,
                                             goal_cb=self.dialogue_goal_cb,
                                             feedback_cb=self.dialogue_feedback_cb,
                                             result_cb=self.dialogue_result_cb,
                                             output_keys=['result_word'],
                                             exec_timeout=rospy.Duration(5.0),
                                             server_wait_timeout=rospy.Duration(1.0))

    def dialogue_goal_cb(self, userdata, goal):
        rospy.loginfo('Executing state CheckLandmarkSeen')
        ShowAction.services_proxy["activate_dialogue"]()
        ShowAction.feedback.current_step = "Receive yes no answer"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        dialogue_goal = dialogue_actionGoal()
        dialogue_goal.keywords = ['yes', 'no']
        return dialogue_goal

    def dialogue_feedback_cb(self, userdata, feedback):
        self._activate_time = rospy.Time.now()
        ShowAction.feedback.current_step = "Human said something else thant yes or no"
        ShowAction.action_server.publish_feedback(ShowAction.feedback)
        ShowAction.services_proxy["say"]("I'm sorry I did not understand, you should say yes or no")
        self._activate_time = rospy.Time.now()
        self._repeat_question = 0

    def dialogue_result_cb(self, userdata, status, result):
        action_success = ''
        if status == actionlib.GoalStatus.SUCCEEDED:
            ShowAction.feedback.current_step = "Human said yes or no"
            ShowAction.action_server.publish_feedback(ShowAction.feedback)
            userdata.result_word = result.word
            action_success = 'succeeded'
            self._repeat_question = 0
        elif status == actionlib.GoalStatus.PREEMPTED:
            if not self.preempt_requested():
                if self._repeat_question < 2:
                    if self._repeat_question == 0:
                        ShowAction.feedback.current_step = "Robot did not hear for the first time"
                        ShowAction.action_server.publish_feedback(ShowAction.feedback)
                        ShowAction.services_proxy["say"]("Sorry I can't hear you, can you repeat ?")
                    elif self._repeat_question == 1:
                        ShowAction.feedback.current_step = "Robot did not hear for the second time"
                        ShowAction.services_proxy["say"]("I am really sorry I still can not hear you, can you speak louder ?")

                    self._repeat_question += 1
                    action_success = 'preempted'
                else:
                    ShowAction.feedback.current_step = "Robot did not hear and gave up"
                    ShowAction.action_server.publish_feedback(ShowAction.feedback)
                    userdata.result_word = result.word
                    ShowAction.services_proxy["say"]("I did not hear, sorry, I give up")
                    action_success = 'aborted'
            else:
                ShowAction.feedback.current_step = "GetYesNo state preempted"
                ShowAction.action_server.publish_feedback(ShowAction.feedback)
                action_success = 'aborted'
        else:
            ShowAction.feedback.current_step = "Dialog goal status aborted"
            ShowAction.action_server.publish_feedback(ShowAction.feedback)

            action_success = 'aborted'

        #ne fonctionne pas. Retourne "DEACTIVATED ERROR" sur dialogue_control.py
        ShowAction.services_proxy["deactivate_dialogue"]()

        return action_success

class DispatchYesNo(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of DispatchYesNo state")
        smach.State.__init__(self, outcomes=['yes', 'preempted', 'ask_point_again', 'pointing', 'no'],
                            input_keys=['question_asked', 'result_word'])

    def execute(self, userdata):
        next_action = ''
        if self.preempt_requested():
            rospy.loginfo("DispatchYesNo preempted")
            self.service_preempt()
            next_action = 'preempted'
        if userdata.question_asked == 'ask_seen':
            if userdata.result_word == 'yes':
                ShowAction.services_proxy["say"]("Awesome !")
                next_action = 'yes'
            else:
                ShowAction.services_proxy["say"]("Oh it's too bad, I'm sorry I wasn't good enough")
                next_action = 'ask_point_again'
        elif userdata.question_asked == 'ask_point_again':
            if userdata.result_word == 'yes':
                next_action = 'pointing'
            else:
                ShowAction.services_proxy["say"]("Ok, too bad. Good luck !")
                next_action = 'no'

        return next_action


class Failure(smach.State):
    def __init__(self):
        rospy.loginfo("Initialization of Failure state")
        smach.State.__init__(self, outcomes=['aborted'], input_keys=['last_guiding_state'])

    def execute(self, userdata):
        if userdata.last_guiding_state == 'PointingConfig':
            rospy.loginfo("Pepper can't get the pointing configuration")
        elif userdata.last_guiding_state == 'MoveToPose':
            rospy.loginfo("Pepper can't reach the pointing configuration")
        elif userdata.last_guiding_state == 'LookAtLandmark':
            rospy.loginfo("Pepper can't look at the landmark")
        elif userdata.last_guiding_state == 'LookAtHuman':
            rospy.loginfo("Pepper can't look at the human")
        elif userdata.last_guiding_state == 'PointAtLandmark':
            rospy.loginfo("Pepper can't point at the landmark")
        elif userdata.last_guiding_state == 'CheckLandmarkSeen':
            rospy.loginfo("Pepper can't check if the landmark has been seen")

        return 'aborted'


if __name__ == '__main__':
    rospy.init_node('show_action_server')
    server = ShowAction(rospy.get_name())
    rospy.on_shutdown(server.stand_pose)
    rospy.spin()

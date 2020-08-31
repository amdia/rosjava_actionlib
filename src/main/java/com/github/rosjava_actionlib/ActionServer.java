/**
 * Copyright 2020 Spyros Koukas
 * Copyright 2015 Ekumen www.ekumenlabs.com
 * <p>
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * <p>
 * http://www.apache.org/licenses/LICENSE-2.0
 * <p>
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.github.rosjava_actionlib;

import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;
import actionlib_msgs.GoalStatusArray;
import org.apache.commons.lang3.exception.ExceptionUtils;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.internal.message.Message;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.TimeUnit;

/**
 * Class to encapsulate the actiolib server's communication and goal management.
 *
 * @author Spyros Koukas
 * @author Ernesto Corbellini ecorbellini@ekumenlabs.com
 */
public final class ActionServer<T_ACTION_GOAL extends Message,
        T_ACTION_FEEDBACK extends Message,
        T_ACTION_RESULT extends Message> {
    private static final Log LOGGER = LogFactory.getLog(ActionServer.class);

    //default status_frequency is 5Hz for python and cpp
    private static final long DEFAULT_STATUS_TICK_PERIOD_MILLIS = 200;
    private static final long DEFAULT_STATUS_TICK_DELAY_MILLIS = 200;

    /**
     * Keeps the status of each goal
     *
     * @param <T_ACTION_GOAL_TYPE> the T_ACTION_GOAL type
     */
    private static final class ServerGoal<T_ACTION_GOAL_TYPE extends Message> {
        private final T_ACTION_GOAL_TYPE goal;
        private final ServerStateMachine stateMachine = new ServerStateMachine();

        private ServerGoal(final T_ACTION_GOAL_TYPE goal) {
            this.goal = goal;
        }
    }


    private final String actionGoalType;
    private final String actionResultType;
    private final String actionFeedbackType;
    private Subscriber<T_ACTION_GOAL> goalSubscriber = null;
    private Subscriber<GoalID> cancelSubscriber = null;

    private Publisher<T_ACTION_RESULT> resultPublisher = null;
    private Publisher<T_ACTION_FEEDBACK> feedbackPublisher = null;
    private Publisher<GoalStatusArray> statusPublisher = null;
    private final ConnectedNode node;
    private final String actionName;
    private final List<ActionServerListener> callbackStatusTargets = new CopyOnWriteArrayList<>();

    private Timer statusTick = new Timer();
    private ConcurrentHashMap<String, ServerGoal<T_ACTION_GOAL>> goalIdToGoalStatusMap = new ConcurrentHashMap<>(1);

    /**
     * Constructor.
     *
     * @param node               Object representing a node connected to a ROS master.
     * @param actionName         String that identifies the name of this action. This name
     *                           is used for naming the ROS topics.
     * @param actionGoalType     String holding the type for the action goal message.
     * @param actionFeedbackType String holding the type for the action feedback
     *                           message.
     * @param actionResultType   String holding the type for the action result
     *                           message.
     */
    public ActionServer(final ConnectedNode node
            , final String actionName
            , final String actionGoalType
            , final String actionFeedbackType
            , final String actionResultType) {
        this.node = node;
        this.actionName = actionName;
        this.actionGoalType = actionGoalType;
        this.actionFeedbackType = actionFeedbackType;
        this.actionResultType = actionResultType;

        this.connect(node);
    }

    /**
     * Attach an {@link ActionServerListener} to this {@link ActionServer}. The {@link ActionServerListener} provides callback methods for each
     * incoming message and event.
     * Multiple listeners can be added.
     *
     * @param target An object that implements the ActionServerListener interface.
     *               This object will receive the callbacks with the events.
     */
    public final void attachListener(final ActionServerListener target) {
        if (target != null) {
            this.callbackStatusTargets.add(target);
        }
    }

    /**
     * Removes the {@link ActionServerListener}, if already attached
     *
     * @return the old {@link ActionServerListener}s
     */
    public final List<ActionServerListener> detachAllActionServerListeners() {
        final List<ActionServerListener> oldActionServerListeners = new ArrayList<>(this.callbackStatusTargets);
        this.callbackStatusTargets.clear();
        return oldActionServerListeners;
    }

    /**
     * Publish the current status information for the tracked goals on the /status topic.
     *
     * @param status GoalStatusArray message containing the status to send.
     *
     * @see actionlib_msgs.GoalStatusArray
     */
    public final void sendStatus(final GoalStatusArray status) {
        this.statusPublisher.publish(status);
    }

    /**
     * Publish a feedback message on the /feedback topic.
     *
     * @param feedback An action feedback message to send.
     */
    public final void sendFeedback(final T_ACTION_FEEDBACK feedback) {
        this.feedbackPublisher.publish(feedback);
    }

    /**
     * Publish result message on the /result topic.
     *
     * @param result The action result message to send.
     */
    public final void sendResult(final T_ACTION_RESULT result) {
        this.resultPublisher.publish(result);
    }

    /**
     * Publish the action server topics: /status, /feedback, /result
     *
     * @param node The object representing a node connected to a ROS master.
     */
    private final void publishServer(final ConnectedNode node) {
        this.statusPublisher = node.newPublisher(this.getActionStatusTopic(), GoalStatusArray._TYPE);
        this.feedbackPublisher = node.newPublisher(this.getActionFeedbackTopic(), actionFeedbackType);
        this.resultPublisher = node.newPublisher(this.getActionResultTopic(), actionResultType);
        this.statusTick.scheduleAtFixedRate(new TimerTask() {
            @Override
            public final void run() {
                ActionServer.this.sendStatusTick();
            }
        }, DEFAULT_STATUS_TICK_DELAY_MILLIS, DEFAULT_STATUS_TICK_PERIOD_MILLIS);
    }

    /**
     * @return
     */
    public final String[] getPublishedTopics() {
        final String[] publishedTopics = new String[3];
        publishedTopics[0] = this.getActionResultTopic();
        publishedTopics[1] = this.getActionFeedbackTopic();
        publishedTopics[2] = this.getActionStatusTopic();
        return publishedTopics;
    }

    /**
     * @return
     */
    public final String[] getSubscribedTopics() {
        final String[] subscribedTopics = new String[2];
        subscribedTopics[0] = this.getActionGoalTopic();
        subscribedTopics[1] = this.getActionCancelTopic();

        return subscribedTopics;
    }

    /**
     * Subscribed topic
     *
     * @return
     */
    public final String getActionGoalTopic() {
        return this.actionName + "/goal";
    }

    /**
     * Subscribed topic
     *
     * @return
     */
    public final String getActionCancelTopic() {
        return this.actionName + "/cancel";
    }

    /**
     * Published topic
     *
     * @return
     */
    public final String getActionStatusTopic() {
        return this.actionName + "/status";
    }

    /**
     * Published topic
     *
     * @return
     */
    public final String getActionFeedbackTopic() {
        return this.actionName + "/feedback";
    }

    /**
     * Published topic
     *
     * @return
     */
    public final String getActionResultTopic() {
        return this.actionName + "/result";
    }


    /**
     * Stop publishing the action server topics.
     */
    private final void unpublishServer() {
        if (this.statusPublisher != null) {
            this.statusPublisher.shutdown(5, TimeUnit.SECONDS);
            this.statusPublisher = null;
        }
        if (this.feedbackPublisher != null) {
            this.feedbackPublisher.shutdown(5, TimeUnit.SECONDS);
            this.feedbackPublisher = null;
        }
        if (this.resultPublisher != null) {
            this.resultPublisher.shutdown(5, TimeUnit.SECONDS);
            this.resultPublisher = null;
        }
    }

    /**
     * Subscribe to the action client's topics: goal and cancel.
     *
     * @param node The ROS node connected to the master.
     */
    private final void subscribeToClient(final ConnectedNode node) {
        this.goalSubscriber = node.newSubscriber(this.getActionGoalTopic(), actionGoalType);
        this.cancelSubscriber = node.newSubscriber(this.getActionCancelTopic(), GoalID._TYPE);

        this.goalSubscriber.addMessageListener(new MessageListener<>() {
            @Override
            public void onNewMessage(final T_ACTION_GOAL message) {
                gotGoal(message);
            }
        });

        this.cancelSubscriber.addMessageListener(new MessageListener<GoalID>() {
            @Override
            public void onNewMessage(final GoalID message) {
                gotCancel(message);
            }
        });
    }

    /**
     * Unsubscribe from the client's topics.
     */
    private final void unsubscribeToClient() {
        if (this.goalSubscriber != null) {
            this.goalSubscriber.shutdown(5, TimeUnit.SECONDS);
            this.goalSubscriber = null;
        }
        if (this.cancelSubscriber != null) {
            this.cancelSubscriber.shutdown(5, TimeUnit.SECONDS);
            this.cancelSubscriber = null;
        }
    }

    /**
     * Called when a message is received from the subscribed goal topic.
     */
    public final void gotGoal(final T_ACTION_GOAL goal) {
        final String goalIdString = getGoalId(goal).getId();

        // start tracking this newly received goal
        this.goalIdToGoalStatusMap.put(goalIdString, new ServerGoal<>(goal));
        for (final ActionServerListener actionServerListener : this.callbackStatusTargets) {
            // Propagate the callback

            // inform the user of a received message
            actionServerListener.goalReceived(goal);
            // ask the user to accept the goal
            final boolean accepted = actionServerListener.acceptGoal(goal);
            if (accepted) {
                // the user accepted the goal
                this.goalIdToGoalStatusMap.get(goalIdString).stateMachine.transition(ServerStateMachine.Events.ACCEPT);

            } else {
                // the user rejected the goal
                this.goalIdToGoalStatusMap.get(goalIdString).stateMachine.transition(ServerStateMachine.Events.REJECT);
            }
        }
    }

    /**
     * Called when we get a message on the subscribed cancel topic.
     */
    public final void gotCancel(final GoalID gid) {
        for (final ActionServerListener actionServerListener : this.callbackStatusTargets) {
            // Propagate the callback
            actionServerListener.cancelReceived(gid);
        }
    }

    /**
     * Publishes the current status on the server's status topic.
     * This is used like a heartbeat to update the status of every tracked goal.
     */
    public final void sendStatusTick() {
        final GoalStatusArray status = this.statusPublisher.newMessage();
        final List<GoalStatus> goalStatusList = new ArrayList<>();

        try {
            for (final Iterator<ServerGoal<T_ACTION_GOAL>> sgIterator = this.goalIdToGoalStatusMap.values().iterator(); sgIterator.hasNext(); ) {
                final ServerGoal<T_ACTION_GOAL> serverGoal = sgIterator.next();
                final GoalStatus goalStatus = node.getTopicMessageFactory().newFromType(GoalStatus._TYPE);
                goalStatus.setGoalId(getGoalId(serverGoal.goal));
                goalStatus.setStatus((byte) serverGoal.stateMachine.getState());
                goalStatusList.add(goalStatus);
            }
        } catch (final Exception exception) {
            LOGGER.error(ExceptionUtils.getStackTrace(exception));
        } catch (final Throwable throwable) {
            LOGGER.error(ExceptionUtils.getStackTrace(throwable));
        }

        status.setStatusList(goalStatusList);
        sendStatus(status);
    }

    public final T_ACTION_RESULT newResultMessage() {
        return resultPublisher.newMessage();
    }

    public final T_ACTION_FEEDBACK newFeedbackMessage() {
        return feedbackPublisher.newMessage();
    }

    /**
     * Returns the goal ID object related to a given action goal.
     *
     * @param goal An action goal message.
     *
     * @return The goal ID object.
     */
    public final GoalID getGoalId(final T_ACTION_GOAL goal) {

        final GoalID gid = ActionLibMessagesUtils.getSubMessageFromMessage(goal, "getGoalId");
        return gid;
    }

    /**
     * Get the current state of the referenced goal.
     *
     * @param goalId String representing the ID of the goal.
     *
     * @return The current state of the goal or -100 if the goal ID is not tracked.
     *
     * @see actionlib_msgs.GoalStatus
     */
    public final byte getGoalState(final String goalId) {
        byte ret = 0;

        if (this.goalIdToGoalStatusMap.containsKey(goalId)) {
            ret = this.goalIdToGoalStatusMap.get(goalId).stateMachine.getState();
        } else {
            ret = -100;
        }
        return ret;
    }

    /**
     * Express a succeed event for this goal. The state of the goal will be updated.
     */
    public final void setSucceed(final String goalIdString) {
        this.goalIdToGoalStatusMap.get(goalIdString).stateMachine.transition(ServerStateMachine.Events.SUCCEED);
    }

    /**
     * Express a preempted event for this goal. The state of the goal will be updated.
     */
    public final void setPreempt(final String goalIdString) {

        this.goalIdToGoalStatusMap.get(goalIdString).stateMachine.transition(ServerStateMachine.Events.CANCEL_REQUEST);
        this.goalIdToGoalStatusMap.get(goalIdString).stateMachine.transition(ServerStateMachine.Events.CANCEL);
    }

    /**
     * Express an aborted event for this goal. The state of the goal will be updated.
     */
    public final void setAbort(final String goalIdString) {
        this.goalIdToGoalStatusMap.get(goalIdString).stateMachine.transition(ServerStateMachine.Events.ABORT);
    }

    /**
     * Set goal ID and state information to the goal status message.
     *
     * @param goalStatus GoalStatus message.
     * @param gidString  String identifying the goal.
     *
     * @see actionlib_msgs.GoalStatus
     */
    public final void setGoalStatus(final GoalStatus goalStatus, final String gidString) {
        final ServerGoal<T_ACTION_GOAL> serverGoal = this.goalIdToGoalStatusMap.get(gidString);
        goalStatus.setGoalId(getGoalId(serverGoal.goal));
        goalStatus.setStatus(serverGoal.stateMachine.getState());
    }

    /**
     * Publishes the server's topics and suscribes to the client's topics.
     */
    private final void connect(final ConnectedNode node) {
        publishServer(node);
        subscribeToClient(node);
    }

    /**
     * Finish the action server.
     * Unregister publishers and listeners.
     * The programmer need to ensure that this method is called when the program has finished using this {@link ActionServer}
     */
    public final void finish() {
        this.callbackStatusTargets.clear();
        unpublishServer();
        unsubscribeToClient();
    }

}

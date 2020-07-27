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
import org.ros.internal.message.Message;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.HashMap;
import java.util.Timer;
import java.util.TimerTask;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

/**
 * Class to encapsulate the actiolib server's communication and goal management.
 *
 * @author Ernesto Corbellini ecorbellini@ekumenlabs.com
 */
public final class ActionServer<T_ACTION_GOAL extends Message,
        T_ACTION_FEEDBACK extends Message,
        T_ACTION_RESULT extends Message> {

    /**
     *
     */
    private final class ServerGoal {
        private final T_ACTION_GOAL goal;
        private final ServerStateMachine state = new ServerStateMachine();

        private ServerGoal(final T_ACTION_GOAL goal) {
            this.goal = goal;
        }
    }

    private T_ACTION_GOAL actionGoal;
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
    private ActionServerListener callbackTarget = null;
    private Timer statusTick = new Timer();
    private HashMap<String, ServerGoal> goalTracker = new HashMap<String,
            ServerGoal>(1);

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
     * Attach a listener to this actionlib server. The listener must implement the
     * ActionServerListener interface which provides callback methods for each
     * incoming message and event.
     *
     * @param target An object that implements the ActionServerListener interface.
     *               This object will receive the callbacks with the events.
     */
    public void attachListener(final ActionServerListener target) {
        this.callbackTarget = target;
    }

    /**
     * Publish the current status information for the tracked goals on the /status topic.
     *
     * @param status GoalStatusArray message containing the status to send.
     *
     * @see actionlib_msgs.GoalStatusArray
     */
    public void sendStatus(final GoalStatusArray status) {
        this.statusPublisher.publish(status);
    }

    /**
     * Publish a feedback message on the /feedback topic.
     *
     * @param feedback An action feedback message to send.
     */
    public void sendFeedback(final T_ACTION_FEEDBACK feedback) {
        this.feedbackPublisher.publish(feedback);
    }

    /**
     * Publish result message on the /result topic.
     *
     * @param result The action result message to send.
     */
    public void sendResult(final T_ACTION_RESULT result) {
        this.resultPublisher.publish(result);
    }

    /**
     * Publish the action server topics: /status, /feedback, /result
     *
     * @param node The object representing a node connected to a ROS master.
     */
    private void publishServer(final ConnectedNode node) {
        this.statusPublisher = node.newPublisher(actionName + "/status", GoalStatusArray._TYPE);
        this.feedbackPublisher = node.newPublisher(actionName + "/feedback", actionFeedbackType);
        this.resultPublisher = node.newPublisher(actionName + "/result", actionResultType);
        this.statusTick.scheduleAtFixedRate(new TimerTask() {
            @Override
            public final void run() {
                ActionServer.this.sendStatusTick();
            }
        }, 2000, 200); //default status_frequency is 5Hz for python and cpp
    }

    /**
     * Stop publishing the action server topics.
     */
    private void unpublishServer() {
        if (statusPublisher != null) {
            statusPublisher.shutdown(5, TimeUnit.SECONDS);
            statusPublisher = null;
        }
        if (feedbackPublisher != null) {
            feedbackPublisher.shutdown(5, TimeUnit.SECONDS);
            feedbackPublisher = null;
        }
        if (resultPublisher != null) {
            resultPublisher.shutdown(5, TimeUnit.SECONDS);
            resultPublisher = null;
        }
    }

    /**
     * Subscribe to the action client's topics: goal and cancel.
     *
     * @param node The ROS node connected to the master.
     */
    private void subscribeToClient(ConnectedNode node) {
        goalSubscriber = node.newSubscriber(actionName + "/goal", actionGoalType);
        cancelSubscriber = node.newSubscriber(actionName + "/cancel", GoalID._TYPE);

        goalSubscriber.addMessageListener(new MessageListener<T_ACTION_GOAL>() {
            @Override
            public void onNewMessage(T_ACTION_GOAL message) {
                gotGoal(message);
            }
        });

        cancelSubscriber.addMessageListener(new MessageListener<GoalID>() {
            @Override
            public void onNewMessage(GoalID message) {
                gotCancel(message);
            }
        });
    }

    /**
     * Unsubscribe from the client's topics.
     */
    private void unsubscribeToClient() {
        if (goalSubscriber != null) {
            goalSubscriber.shutdown(5, TimeUnit.SECONDS);
            goalSubscriber = null;
        }
        if (cancelSubscriber != null) {
            cancelSubscriber.shutdown(5, TimeUnit.SECONDS);
            cancelSubscriber = null;
        }
    }

    /**
     * Called when we get a message on the subscribed goal topic.
     */
    public void gotGoal(T_ACTION_GOAL goal) {
        boolean accepted = false;
        String goalIdString = getGoalId(goal).getId();

        // start tracking this newly received goal
        goalTracker.put(goalIdString, new ServerGoal(goal));
        // Propagate the callback
        if (callbackTarget != null) {
            // inform the user of a received message
            callbackTarget.goalReceived(goal);
            // ask the user to accept the goal
            accepted = callbackTarget.acceptGoal(goal);
            if (accepted) {
                // the user accepted the goal
                this.goalTracker.get(goalIdString).state.transition(ServerStateMachine.Events.ACCEPT);

            } else {
                // the user rejected the goal
                this.goalTracker.get(goalIdString).state.transition(ServerStateMachine.Events.REJECT);

            }
        }
    }

    /**
     * Called when we get a message on the subscribed cancel topic.
     */
    public void gotCancel(GoalID gid) {
        // Propagate the callback
        if (callbackTarget != null) {
            callbackTarget.cancelReceived(gid);
        }
    }

    /**
     * Publishes the current status on the server's status topic.
     * This is used like a heartbeat to update the status of every tracked goal.
     */
    public void sendStatusTick() {
        GoalStatusArray status = statusPublisher.newMessage();
        GoalStatus goalStatus;
        Vector<GoalStatus> goalStatusList = new Vector<GoalStatus>();

        try {
            for (java.util.Iterator<ServerGoal> sgIterator = goalTracker.values().iterator(); sgIterator.hasNext(); ) {
                ServerGoal sg = sgIterator.next();
                goalStatus = node.getTopicMessageFactory().newFromType(GoalStatus._TYPE);
                goalStatus.setGoalId(getGoalId(sg.goal));
                goalStatus.setStatus((byte) sg.state.getState());
                goalStatusList.add(goalStatus);
            }
        } catch (java.util.ConcurrentModificationException exception) {
            exception.printStackTrace(System.out);
        } catch (Throwable throwable) {
            throwable.printStackTrace(System.out);
        }

        status.setStatusList(goalStatusList);
        sendStatus(status);
    }

    public T_ACTION_RESULT newResultMessage() {
        return resultPublisher.newMessage();
    }

    public T_ACTION_FEEDBACK newFeedbackMessage() {
        return feedbackPublisher.newMessage();
    }

    /**
     * Returns the goal ID object related to a given action goal.
     *
     * @param goal An action goal message.
     *
     * @return The goal ID object.
     */
    public GoalID getGoalId(T_ACTION_GOAL goal) {

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
    public byte getGoalState(String goalId) {
        byte ret = 0;

        if (goalTracker.containsKey(goalId)) {
            ret = goalTracker.get(goalId).state.getState();
        } else {
            ret = -100;
        }
        return ret;
    }

    /**
     * Express a succeed event for this goal. The state of the goal will be updated.
     */
    public void setSucceed(String goalIdString) {
        this.goalTracker.get(goalIdString).state.transition(ServerStateMachine.Events.SUCCEED);
    }

    /**
     * Express a preempted event for this goal. The state of the goal will be updated.
     */
    public void setPreempt(String goalIdString) {

        goalTracker.get(goalIdString).state.transition(ServerStateMachine.Events.CANCEL_REQUEST);
        goalTracker.get(goalIdString).state.transition(ServerStateMachine.Events.CANCEL);
    }

    /**
     * Express an aborted event for this goal. The state of the goal will be updated.
     */
    public void setAbort(String goalIdString) {
        this.goalTracker.get(goalIdString).state.transition(ServerStateMachine.Events.ABORT);
    }

    /**
     * Set goal ID and state information to the goal status message.
     *
     * @param goalStatus GoalStatus message.
     * @param gidString  String identifying the goal.
     *
     * @see actionlib_msgs.GoalStatus
     */
    public void setGoalStatus(final GoalStatus goalStatus, final String gidString) {
        final ServerGoal serverGoal = this.goalTracker.get(gidString);
        goalStatus.setGoalId(getGoalId(serverGoal.goal));
        goalStatus.setStatus(serverGoal.state.getState());
    }

    /**
     * Publishes the server's topics and suscribes to the client's topics.
     */
    private void connect(ConnectedNode node) {
        publishServer(node);
        subscribeToClient(node);
    }

    /**
     * Finish the action server. Unregister publishers and listeners.
     */
    public void finish() {
        callbackTarget = null;
        unpublishServer();
        unsubscribeToClient();
    }

    protected void finalize() {
        finish();
    }
}

/**
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

import org.apache.commons.lang3.NotImplementedException;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import java.util.*;
import java.util.stream.Collectors;


/**
 * State machine for the action client.
 *
 * @author Ernesto Corbellini ecorbellini@ekumenlabs.com
 */
final class ClientStateMachine {

    /**
     * A ClientStateMachine should always have an existing state.
     *
     * @param initialState
     */
    ClientStateMachine(final ClientState initialState) {
        Objects.requireNonNull(initialState);
        this.state = initialState;
    }

    private ClientState latestGoalStatus;
    private ClientState state;
    private Log log = LogFactory.getLog(ActionClient.class);


    /**
     * @param state
     */
    final synchronized void setState(final ClientState state) {
        Objects.requireNonNull(state);
        log.info("ClientStateMachine - State changed from " + this.state + " to " + state);
        this.state = state;
    }

     final synchronized ClientState getState() {
        return this.state;
    }

    /**
     * @deprecated enforce Enum usage
     * Update the state of the client based on the current state and the goal state.
     */
    @Deprecated
     final synchronized void updateStatus(int status) {
        if (this.state != ClientState.DONE) {
            this.latestGoalStatus = ClientState.from(status);
        }
    }

    /**
     * Update the state of the client based on the current state and the goal state.
     * @param status
     */
    final synchronized void updateStatus(ClientState status) {
        if (this.state != ClientState.DONE) {
            this.latestGoalStatus = status;
        }
    }

    /**
     * Update the state of the client upon the received status of the goal.
     *
     * @param goalStatus Status of the goal.
     */
     final synchronized void transition(int goalStatus) {
        List<ClientState> nextStates;
        Iterator<ClientState> iterStates;

        // transition to next states
        nextStates = getTransition(goalStatus);
        iterStates = nextStates.iterator();

        log.info("ClientStateMachine - State transition invoked.");

        while (iterStates.hasNext()) {
            this.state = iterStates.next();
        }
    }

    /**
     * Get the next state transition depending on the current client state and the
     * goal state.
     *
     * @param goalStatus The current status of the tracked goal.
     *
     * @return A vector with the list of next states. The states should be
     * transitioned in order. This is necessary because if we loose a state update
     * we might still be able to infer the actual transition history that took us
     * to the final goal state.
     */
    final List<Integer> getTransitionInteger(int goalStatus) {
        return getTransition(goalStatus).stream().map(ClientState::getValue).collect(Collectors.toList());
    }


    final List<ClientState> getTransition(int goalStatus) {
        List<ClientState> stateList = new LinkedList<>();

        switch (this.state) {
            case WAITING_FOR_GOAL_ACK:
                switch (goalStatus) {
                    case actionlib_msgs.GoalStatus.PENDING:
                        stateList.add(ClientState.PENDING);
                        break;
                    case actionlib_msgs.GoalStatus.ACTIVE:
                        stateList.add(ClientState.ACTIVE);
                        break;
                    case actionlib_msgs.GoalStatus.REJECTED:
                        stateList.add(ClientState.PENDING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLING:
                        stateList.add(ClientState.PENDING);
                        stateList.add(ClientState.RECALLING);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLED:
                        stateList.add(ClientState.PENDING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTED:
                        stateList.add(ClientState.ACTIVE);
                        stateList.add(ClientState.PREEMPTING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.SUCCEEDED:
                        stateList.add(ClientState.ACTIVE);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.ABORTED:
                        stateList.add(ClientState.ACTIVE);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTING:
                        stateList.add(ClientState.ACTIVE);
                        stateList.add(ClientState.PREEMPTING);
                        break;
                }
                break;
            case PENDING:
                switch (goalStatus) {
                    case actionlib_msgs.GoalStatus.PENDING:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.ACTIVE:
                        stateList.add(ClientState.ACTIVE);
                        break;
                    case actionlib_msgs.GoalStatus.REJECTED:
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLING:
                        stateList.add(ClientState.RECALLING);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLED:
                        stateList.add(ClientState.RECALLING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTED:
                        stateList.add(ClientState.ACTIVE);
                        stateList.add(ClientState.PREEMPTING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.SUCCEEDED:
                        stateList.add(ClientState.ACTIVE);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.ABORTED:
                        stateList.add(ClientState.ACTIVE);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTING:
                        stateList.add(ClientState.ACTIVE);
                        stateList.add(ClientState.PREEMPTING);
                        break;
                }
                break;
            case ACTIVE:
                switch (goalStatus) {
                    case actionlib_msgs.GoalStatus.PENDING:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.ACTIVE:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.REJECTED:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLING:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLED:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTED:
                        stateList.add(ClientState.PREEMPTING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.SUCCEEDED:
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.ABORTED:
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTING:
                        stateList.add(ClientState.PREEMPTING);
                        break;
                }
                break;
            case WAITING_FOR_RESULT:
                switch (goalStatus) {
                    case actionlib_msgs.GoalStatus.PENDING:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.ACTIVE:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.REJECTED:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.RECALLING:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLED:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTED:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.SUCCEEDED:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.ABORTED:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTING:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                }
                break;
            case WAITING_FOR_CANCEL_ACK:
                switch (goalStatus) {
                    case actionlib_msgs.GoalStatus.PENDING:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.ACTIVE:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.REJECTED:
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLING:
                        stateList.add(ClientState.RECALLING);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLED:
                        stateList.add(ClientState.RECALLING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTED:
                        stateList.add(ClientState.PREEMPTING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.SUCCEEDED:
                        stateList.add(ClientState.PREEMPTING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.ABORTED:
                        stateList.add(ClientState.PREEMPTING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTING:
                        stateList.add(ClientState.PREEMPTING);
                        break;
                }
                break;
            case RECALLING:
                switch (goalStatus) {
                    case actionlib_msgs.GoalStatus.PENDING:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.ACTIVE:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.REJECTED:
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLING:
                        stateList.add(ClientState.RECALLING);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLED:
                        stateList.add(ClientState.RECALLING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTED:
                        stateList.add(ClientState.PREEMPTING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.SUCCEEDED:
                        stateList.add(ClientState.PREEMPTING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.ABORTED:
                        stateList.add(ClientState.PREEMPTING);
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTING:
                        stateList.add(ClientState.PREEMPTING);
                        break;
                }
                break;
            case PREEMPTING:
                switch (goalStatus) {
                    case actionlib_msgs.GoalStatus.PENDING:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.ACTIVE:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.REJECTED:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLING:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLED:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTED:
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.SUCCEEDED:
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.ABORTED:
                        stateList.add(ClientState.WAITING_FOR_RESULT);
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTING:
                        // no transition
                        break;
                }
                break;
            case DONE:
                switch (goalStatus) {
                    case actionlib_msgs.GoalStatus.PENDING:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.ACTIVE:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.REJECTED:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.RECALLING:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                    case actionlib_msgs.GoalStatus.RECALLED:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTED:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.SUCCEEDED:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.ABORTED:
                        // no transition
                        break;
                    case actionlib_msgs.GoalStatus.PREEMPTING:
                        stateList.add(ClientState.INVALID_TRANSITION);
                        break;
                }
                break;
        }
        return stateList;
    }

    /**
     * Cancel action goal. The goal can only be cancelled if its in certain
     * states. If it can be cancelled the state will be changed to
     * WAITING_FOR_CANCEL_ACK.
     *
     * @return True if the goal can be cancelled, false otherwise.
     */
    final boolean cancel() {
        ArrayList<ClientState> cancellableStates = new ArrayList<>(Arrays.asList(ClientState.WAITING_FOR_GOAL_ACK,
                ClientState.PENDING, ClientState.ACTIVE));
        boolean shouldCancel = cancellableStates.contains(state);

        if (shouldCancel) {
            state = ClientState.WAITING_FOR_CANCEL_ACK;
        }
        return shouldCancel;
    }

    final void resultReceived() {
        if (state == ClientState.WAITING_FOR_RESULT) {
            state = ClientState.DONE;
        } else {
            state = ClientState.ERROR;
        }
    }

    // TODO: implement method
    final void markAsLost() {
        throw new NotImplementedException("Not Implemented");
    }

    final boolean isRunning() {
        return state.getValue() >= 0 && state.getValue() < 7;
    }

    final int getLatestGoalStatusInteger() {
        return latestGoalStatus.getValue();
    }

    final ClientState getLatestGoalStatus() {
        return latestGoalStatus;
    }
}

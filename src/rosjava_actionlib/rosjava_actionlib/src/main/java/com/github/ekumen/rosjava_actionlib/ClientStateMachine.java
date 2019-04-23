/**
 * Copyright 2015 Ekumen www.ekumenlabs.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.github.ekumen.rosjava_actionlib;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.Vector;


/**
 * State machine for the action client.
 * @author Ernesto Corbellini ecorbellini@ekumenlabs.com
  */
final class ClientStateMachine {

  private int latestGoalStatus;
  private int state;
  private int nextState;
  private Log log = LogFactory.getLog(ActionClient.class);

  public final synchronized void setState(final int state) {
    log.info("ClientStateMachine - State changed from " + this.state + " to " + state);
    this.state = state;
  }

  public final synchronized int getState() {
    return this.state;
  }

  /**
   * Update the state of the client based on the current state and the goal state.
   */
  public final synchronized void updateStatus(int status)
  {
    if (this.state != ActionLibClientStates.DONE)
    {
      this.latestGoalStatus = status;
    }
  }

  /**
   * Update the state of the client upon the received status of the goal.
   * @param goalStatus Status of the goal.
   */
  public final synchronized void transition(int goalStatus)
  {
    Vector<Integer> nextStates;
    Iterator<Integer> iterStates;

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
   * @param goalStatus The current status of the tracked goal.
   * @return A vector with the list of next states. The states should be
   * transitioned in order. This is necessary because if we loose a state update
   * we might still be able to infer the actual transition history that took us
   * to the final goal state.
   */
  public final Vector<Integer> getTransition(int goalStatus)
  {
    final Vector<Integer> stateList = new Vector<Integer>();

    switch (this.state)
    {
      case ActionLibClientStates.WAITING_FOR_GOAL_ACK:
        switch (goalStatus)
        {
          case actionlib_msgs.GoalStatus.PENDING:
            stateList.add(ActionLibClientStates.PENDING);
            break;
          case actionlib_msgs.GoalStatus.ACTIVE:
            stateList.add(ActionLibClientStates.ACTIVE);
            break;
          case actionlib_msgs.GoalStatus.REJECTED:
            stateList.add(ActionLibClientStates.PENDING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.RECALLING:
            stateList.add(ActionLibClientStates.PENDING);
            stateList.add(ActionLibClientStates.RECALLING);
            break;
          case actionlib_msgs.GoalStatus.RECALLED:
            stateList.add(ActionLibClientStates.PENDING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTED:
            stateList.add(ActionLibClientStates.ACTIVE);
            stateList.add(ActionLibClientStates.PREEMPTING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.SUCCEEDED:
            stateList.add(ActionLibClientStates.ACTIVE);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.ABORTED:
            stateList.add(ActionLibClientStates.ACTIVE);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTING:
            stateList.add(ActionLibClientStates.ACTIVE);
            stateList.add(ActionLibClientStates.PREEMPTING);
            break;
        }
        break;
      case ActionLibClientStates.PENDING:
        switch (goalStatus)
        {
          case actionlib_msgs.GoalStatus.PENDING:
            // no transition
            break;
          case actionlib_msgs.GoalStatus.ACTIVE:
            stateList.add(ActionLibClientStates.ACTIVE);
            break;
          case actionlib_msgs.GoalStatus.REJECTED:
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.RECALLING:
            stateList.add(ActionLibClientStates.RECALLING);
            break;
          case actionlib_msgs.GoalStatus.RECALLED:
            stateList.add(ActionLibClientStates.RECALLING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTED:
            stateList.add(ActionLibClientStates.ACTIVE);
            stateList.add(ActionLibClientStates.PREEMPTING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.SUCCEEDED:
            stateList.add(ActionLibClientStates.ACTIVE);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.ABORTED:
            stateList.add(ActionLibClientStates.ACTIVE);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTING:
            stateList.add(ActionLibClientStates.ACTIVE);
            stateList.add(ActionLibClientStates.PREEMPTING);
            break;
        }
        break;
      case ActionLibClientStates.ACTIVE:
        switch (goalStatus)
        {
          case actionlib_msgs.GoalStatus.PENDING:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.ACTIVE:
            // no transition
            break;
          case actionlib_msgs.GoalStatus.REJECTED:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.RECALLING:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.RECALLED:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTED:
            stateList.add(ActionLibClientStates.PREEMPTING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.SUCCEEDED:
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.ABORTED:
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTING:
            stateList.add(ActionLibClientStates.PREEMPTING);
            break;
        }
        break;
      case ActionLibClientStates.WAITING_FOR_RESULT:
        switch (goalStatus)
        {
          case actionlib_msgs.GoalStatus.PENDING:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.ACTIVE:
            // no transition
            break;
          case actionlib_msgs.GoalStatus.REJECTED:
            // no transition
            break;
          case actionlib_msgs.GoalStatus.RECALLING:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
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
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
        }
        break;
      case ActionLibClientStates.WAITING_FOR_CANCEL_ACK:
        switch (goalStatus)
        {
          case actionlib_msgs.GoalStatus.PENDING:
            // no transition
            break;
          case actionlib_msgs.GoalStatus.ACTIVE:
            // no transition
            break;
          case actionlib_msgs.GoalStatus.REJECTED:
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.RECALLING:
            stateList.add(ActionLibClientStates.RECALLING);
            break;
          case actionlib_msgs.GoalStatus.RECALLED:
            stateList.add(ActionLibClientStates.RECALLING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTED:
            stateList.add(ActionLibClientStates.PREEMPTING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.SUCCEEDED:
            stateList.add(ActionLibClientStates.PREEMPTING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.ABORTED:
            stateList.add(ActionLibClientStates.PREEMPTING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTING:
            stateList.add(ActionLibClientStates.PREEMPTING);
            break;
        }
        break;
      case ActionLibClientStates.RECALLING:
        switch (goalStatus)
        {
          case actionlib_msgs.GoalStatus.PENDING:
            // no transition
            break;
          case actionlib_msgs.GoalStatus.ACTIVE:
            // no transition
            break;
          case actionlib_msgs.GoalStatus.REJECTED:
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.RECALLING:
            stateList.add(ActionLibClientStates.RECALLING);
            break;
          case actionlib_msgs.GoalStatus.RECALLED:
            stateList.add(ActionLibClientStates.RECALLING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTED:
            stateList.add(ActionLibClientStates.PREEMPTING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.SUCCEEDED:
            stateList.add(ActionLibClientStates.PREEMPTING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.ABORTED:
            stateList.add(ActionLibClientStates.PREEMPTING);
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTING:
            stateList.add(ActionLibClientStates.PREEMPTING);
            break;
        }
        break;
      case ActionLibClientStates.PREEMPTING:
        switch (goalStatus)
        {
          case actionlib_msgs.GoalStatus.PENDING:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.ACTIVE:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.REJECTED:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.RECALLING:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.RECALLED:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTED:
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.SUCCEEDED:
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.ABORTED:
            stateList.add(ActionLibClientStates.WAITING_FOR_RESULT);
            break;
          case actionlib_msgs.GoalStatus.PREEMPTING:
            // no transition
            break;
        }
        break;
      case ActionLibClientStates.DONE:
        switch (goalStatus)
        {
          case actionlib_msgs.GoalStatus.PENDING:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.ACTIVE:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
            break;
          case actionlib_msgs.GoalStatus.REJECTED:
            // no transition
            break;
          case actionlib_msgs.GoalStatus.RECALLING:
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
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
            stateList.add(ActionLibClientStates.INVALID_TRANSITION);
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
   * @return True if the goal can be cancelled, false otherwise.
   */
  public final boolean cancel() {
    ArrayList<Integer> cancellableStates = new ArrayList<>(Arrays.asList(ActionLibClientStates.WAITING_FOR_GOAL_ACK,
        ActionLibClientStates.PENDING, ActionLibClientStates.ACTIVE));
    boolean shouldCancel = cancellableStates.contains(state);

    if (shouldCancel) {
      state = ActionLibClientStates.WAITING_FOR_CANCEL_ACK;
    }
    return shouldCancel;
  }

  /**
   *
   */
  public void resultReceived() {
    if (state == ActionLibClientStates.WAITING_FOR_RESULT) {
      state = ActionLibClientStates.DONE;
    }
  }

//  /**
//   * Not implemented
//   */
//  // TODO: implement method
//  public void markAsLost()
//    {
//      throw new NotImplementedException("markAsLost not yet implemented");
//    }

  /**
   *
   * @return
   */
  public int getLatestGoalStatus() {
    return latestGoalStatus;
  }
}

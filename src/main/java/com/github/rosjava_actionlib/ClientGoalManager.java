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

import org.ros.internal.message.Message;

/**
 * Class that binds and action goal with a state machine to track its state.
 *
 * @author Ernesto Corbellini ecorbellini@ekumenlabs.com
 */
public class ClientGoalManager<T_ACTION_GOAL extends Message> {
    private ActionGoal<T_ACTION_GOAL> actionGoal = null;
    private final ClientStateMachine stateMachine;

    /**
     * Getter for stateMachine
     *
     * @return stateMachine
     **/
    final ClientStateMachine getStateMachine() {
        return stateMachine;
    }

    /**
     * Getter for actionGoal
     *
     * @return actionGoal
     **/
    final ActionGoal<T_ACTION_GOAL> getActionGoal() {
        return actionGoal;
    }

    /**
     * @param actionGoal
     */
    public ClientGoalManager(ActionGoal<T_ACTION_GOAL> actionGoal) {
        this.actionGoal = actionGoal;
        this.stateMachine = new ClientStateMachine(ClientState.ERROR);
    }

    /**
     * @param actionGoal
     */
    public void setGoal(final ActionGoal<T_ACTION_GOAL> actionGoal) {
        this.actionGoal = actionGoal;
        if (this.stateMachine.isRunning()) {
            this.stateMachine.setState(ClientState.ERROR);
        }
        this.stateMachine.resetToState(ClientState.WAITING_FOR_GOAL_ACK);
    }

    /**
     * @param actionGoal
     */
    public final void setGoal(final T_ACTION_GOAL actionGoal) {
        final ActionGoal<T_ACTION_GOAL> actionGoalWrapper = new ActionGoal();
        actionGoalWrapper.setActionGoalMessage(actionGoal);
        this.setGoal(actionGoalWrapper);
    }

    /**
     * @return
     */
    public final boolean cancelGoal() {
        return this.stateMachine.cancel();
    }

    /**
     * Signal that the result has been received.
     */
    public final void resultReceived() {
        this.stateMachine.resultReceived();
    }

    /**
     * @param status
     */
    public final void updateStatus(int status) {
        this.stateMachine.transition(status);
    }


    /**
     * @return
     */
    public final ClientState getGoalState() {
        return this.stateMachine.getState();
    }
}

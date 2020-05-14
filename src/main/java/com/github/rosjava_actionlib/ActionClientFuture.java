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
import com.google.common.base.Stopwatch;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.internal.message.Message;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

/**
 * @param <T_GOAL>
 * @param <T_FEEDBACK>
 * @param <T_RESULT>
 */
public final class ActionClientFuture<T_GOAL extends Message, T_FEEDBACK extends Message, T_RESULT extends Message>
        implements ActionFuture<T_GOAL, T_FEEDBACK, T_RESULT>,
        ActionClientListener<T_FEEDBACK, T_RESULT> {
    private static Log log = LogFactory.getLog(ActionClientFuture.class);
    private final GoalID goalid;
    private final ActionClient<T_GOAL, T_FEEDBACK, T_RESULT> actionClient;
    private final ClientGoalManager goalManager = new ClientGoalManager(new ActionGoal<T_GOAL>());
    private T_FEEDBACK latestFeedback = null;
    private T_RESULT result = null;


    /**
     * @param ac
     * @param goal
     * @param <T_GOAL>
     * @param <T_FEEDBACK>
     * @param <T_RESULT>
     *
     * @return
     */
    static final <T_GOAL extends Message, T_FEEDBACK extends Message, T_RESULT extends Message>
    ActionFuture<T_GOAL, T_FEEDBACK, T_RESULT>
    createFromGoal(ActionClient<T_GOAL, T_FEEDBACK, T_RESULT> ac, T_GOAL goal) {
        final GoalID goalId = ac.getGoalId(goal);
        final ActionClientFuture<T_GOAL, T_FEEDBACK, T_RESULT> ret = new ActionClientFuture<>(ac, goalId);
        if (ac.isActive()) {
            log.warn("current goal STATE:" + ac.getGoalState() + "=" + ac.getGoalState().getValue());
        }
        ret.goalManager.setGoal(goal);
        ac.sendGoalWire(goal);
        ac.attachListener(ret);
        return ret;

    }

    /**
     * @param actionClient
     * @param goalID
     */
    private ActionClientFuture(final ActionClient<T_GOAL, T_FEEDBACK, T_RESULT> actionClient, final GoalID goalID) {
        this.actionClient = actionClient;
        this.goalid = goalID;
    }

    /**
     * @return
     */
    @Override
    public final T_FEEDBACK getLatestFeedback() {
        return latestFeedback;
    }

    /**
     * @return
     */
    @Override
    public final ClientState getCurrentState() {
        return goalManager.getStateMachine().getState();
    }

    /**
     * @param bln is currently ignored
     *
     * @return always returns true
     */
    @Override
    public final boolean cancel(final boolean bln) {
        this.actionClient.sendCancel(this.goalid);
        this.goalManager.cancelGoal();
        return true;
    }

    /**
     * @return
     */
    @Override
    public final boolean isCancelled() {
        if (goalManager.getStateMachine().isRunning()) {
            return result == null;
        } else {
            return false;
        }
    }

    /**
     * @return
     */
    @Override
    public final boolean isDone() {
        return !goalManager.getStateMachine().isRunning();
    }

    /**
     * @return
     *
     * @throws InterruptedException
     * @throws ExecutionException
     */
    @Override
    public final T_RESULT get() throws InterruptedException, ExecutionException {
        while (goalManager.getStateMachine().isRunning()) {
            Thread.sleep(100);
        }
        disconnect();
        return result;
    }

    @Override
    public final T_RESULT get(final long timeout, final TimeUnit timeUnit) throws InterruptedException, ExecutionException, TimeoutException {

        final Stopwatch stopwatch = Stopwatch.createStarted();
        while (goalManager.getStateMachine().isRunning()) {
            if (stopwatch.elapsed(timeUnit) > timeout) {
                throw new TimeoutException();
            }
            Thread.sleep(50);
        }
        stopwatch.stop();

        disconnect();
        return result;
    }

    /**
     * @param t_result
     */
    @Override
    public final void resultReceived(final T_RESULT t_result) {
        final ActionResult result = new ActionResult(t_result);
        if (log.isDebugEnabled()) {
            log.debug("Received result: " + result.getGoalStatusMessage().getGoalId().getId());
        }
        if (result.getGoalStatusMessage().getGoalId().getId().equals(goalid.getId())) {
            goalManager.updateStatus(result.getGoalStatusMessage().getStatus());
            goalManager.resultReceived();

            this.result = t_result;
            disconnect();

        } else {
            if (log.isWarnEnabled()) {
                log.warn("Result with Unknown id:" + result.getGoalStatusMessage().getGoalId().getId() + ", waiting for " + goalid.getId());
            }
        }


    }

    @Override
    public final void feedbackReceived(final T_FEEDBACK t_feedback) {
        final ActionFeedback actionFeedback = new ActionFeedback(t_feedback);

        if (actionFeedback.getGoalStatusMessage().getGoalId().getId().equals(goalid.getId())) {
            goalManager.updateStatus(actionFeedback.getGoalStatusMessage().getStatus());
            this.latestFeedback = t_feedback;
        }

    }

    /**
     * @param status The status message received from the server.
     */
    @Override
    public final void statusReceived(final GoalStatusArray status) {
        final String thisGoalId=this.goalid.getId();
        status.getStatusList().stream()
                .filter(goalStatus -> thisGoalId.equals(goalStatus.getGoalId().getId()))
                .map(GoalStatus::getStatus)
                .forEach(this.goalManager::updateStatus);
    }

    /**
     *
     */
    private final void disconnect() {
        this.actionClient.detachListener(this);
    }

    @Override
    public final Future<Boolean> toBooleanFuture() {
        final ActionClientFuture<T_GOAL, T_FEEDBACK, T_RESULT> self = this;
        return new Future<>() {
            @Override
            public final boolean cancel(boolean bln) {

                return self.cancel(bln);
            }

            @Override
            public final boolean isCancelled() {
                return self.isCancelled();
            }

            @Override
            public final boolean isDone() {
                return self.isDone();
            }

            @Override
            public final Boolean get() throws InterruptedException, ExecutionException {
                return self.get() != null;
            }

            @Override
            public final Boolean get(long timeout, TimeUnit timeUnit) throws InterruptedException, ExecutionException, TimeoutException {
                return this.get(timeout, timeUnit) != null;
            }
        };

    }

    /**
     * @return
     */
    @Override
    public final Future<Void> toVoidFuture() {

        final ActionClientFuture<T_GOAL, T_FEEDBACK, T_RESULT> self = this;

        return new Future<>() {
            @Override
            public final boolean cancel(boolean bln) {
                return self.cancel(bln);
            }

            @Override
            public final boolean isCancelled() {
                return self.isCancelled();
            }

            @Override
            public final boolean isDone() {
                return self.isDone();
            }

            @Override
            public final Void get() throws InterruptedException, ExecutionException {
                self.get();
                return null;
            }

            @Override
            public final Void get(final long timeout, final TimeUnit timeUnit) throws InterruptedException, ExecutionException, TimeoutException {
                self.get(timeout, timeUnit);
                return null;
            }
        };

    }

}

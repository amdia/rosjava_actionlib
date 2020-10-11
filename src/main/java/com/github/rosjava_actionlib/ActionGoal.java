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

import actionlib_msgs.GoalID;
import org.ros.internal.message.Message;
import org.ros.message.Time;
import std_msgs.Header;

import java.util.Objects;

/**
 * Class to encapsulate the action goal object.
 *
 * @author Spyros Koukas
 * @author Ernesto Corbellini ecorbellini@ekumenlabs.com
 */
public final class ActionGoal<T_ACTION_GOAL extends Message> {


    private T_ACTION_GOAL goalMessage = null;
    //Getters and setters name

    private static final String SET_GOAL_METHOD_NAME = "setGoal";
    private static final String GET_GOAL_METHOD_NAME = "getGoal";

    private static final String SET_STAMP_METHOD_NAME = "setStamp";
    private static final String GET_STAMP_METHOD_NAME = "getStamp";

    private static final String SET_ID_METHOD_NAME = "setId";
    private static final String GET_ID_METHOD_NAME = "getId";

    private static final String SET_SEQ_METHOD_NAME = "setSeq";
    private static final String GET_SEQ_METHOD_NAME = "getSeq";

    private static final String GET_GOAL_ID_METHOD_NAME = "getGoalId";
    private static final String GET_HEADER_METHOD_NAME = "getHeader";

    /**
     * @param actionGoalMessage
     */
    public ActionGoal(final T_ACTION_GOAL actionGoalMessage) {
        this.goalMessage = actionGoalMessage;
    }

    /**
     *
     */
    public ActionGoal() {
    }

    /**
     * Return the sequence number of the action goal message's header.
     *
     * @return The sequence number of the std_msgs.Header or -1 if there is an error.
     *
     * @see std_msgs.Header
     */
    public final int getHeaderSequence() {

        final Header header = getHeaderMessage();
        final int headerSequence = header == null ? -1 :
                ActionLibMessagesUtils.getSubMessageFromMessage(header,GET_SEQ_METHOD_NAME);

        return headerSequence;
    }

    /**
     * Set the sequence number of the action goal message's header.
     *
     * @param seq The sequence number for the std_msgs.Header.
     *
     * @see std_msgs.Header
     */
    public final void setHeaderSequence(final int seq) {
        final Header header = getHeaderMessage();
        if (header != null) {
            ActionLibMessagesUtils.setSubMessageFromMessage(header, Integer.valueOf(seq), SET_SEQ_METHOD_NAME);

        }
    }

    /**
     * Return the time stamp of the action goal message's header.
     *
     * @return The time stamp (org.ros.message.Time) of the std_msgs.Header or null otherwise.
     *
     * @see org.ros.message.Time
     */
    public final Time getHeaderTimestamp() {
        Time time = null;
        final Header header = getHeaderMessage();
        if (header != null) {
            time = ActionLibMessagesUtils.getSubMessageFromMessage(header, GET_STAMP_METHOD_NAME);
        }
        return time;
    }


    /**
     * Sets the time stamp for the action goal message's header.
     *
     * @param time The time stamp (org.ros.message.Time) of the std_msgs.Header.
     *
     * @see org.ros.message.Time
     */
    public final void setHeaderTimestamp(final Time time) {
        final Header header = getHeaderMessage();
        if (header != null) {
            ActionLibMessagesUtils.setSubMessageFromMessage(header, time, SET_STAMP_METHOD_NAME);
        }
    }


    /**
     * Return the standard actionlib header message for this action goal.
     *
     * @return The std_msgs.Header object or null otherwise.
     *
     * @see std_msgs.Header
     */
    public final Header getHeaderMessage() {
        Header header = null;
        if (this.goalMessage != null) {
            header = ActionLibMessagesUtils.getSubMessageFromMessage(this.goalMessage, GET_HEADER_METHOD_NAME);

        }
        return header;
    }

    /**
     * @return
     */
    public final String getGoalId() {
        String id = null;
        final GoalID gid = getGoalIdMessage();
        if (gid != null) {
            id = ActionLibMessagesUtils.getSubMessageFromMessage(gid, GET_ID_METHOD_NAME);
        }
        return id;
    }

    /**
     * Set the action goal's goal ID string and timestamp.
     *
     * @param id   Identification string for this goal.
     * @param time Time stamp (org.ros.message.Time).
     */
    public final void setGoalId(final String id, final Time time) {
        final GoalID gid = getGoalIdMessage();
        if (gid != null) {
            ActionLibMessagesUtils.setSubMessageFromMessage(gid, id, SET_ID_METHOD_NAME);
            ActionLibMessagesUtils.setSubMessageFromMessage(gid, time, SET_STAMP_METHOD_NAME);
        }
    }


    /**
     * Return the actionlib GoalID message for this action goal.
     *
     * @return The actionlib_msgs.GoalID object or null otherwise.
     *
     * @see actionlib_msgs.GoalID
     */
    public final GoalID getGoalIdMessage() {
        GoalID goalID = null;
        if (this.goalMessage != null) {
            goalID = ActionLibMessagesUtils.getSubMessageFromMessage(this.goalMessage, GET_GOAL_ID_METHOD_NAME);
        }
        return goalID;
    }


    /**
     *
     * @return
     */
    public final T_ACTION_GOAL getActionGoalMessage() {
        return this.goalMessage;
    }

    /**
     *
     * @param actionGoalMessage
     */
    public final void setActionGoalMessage(final T_ACTION_GOAL actionGoalMessage) {
        this.goalMessage = actionGoalMessage;
    }

    /**
     * @return
     */
    public final Message getGoalMessage() {
        Message message = null;
        if (this.goalMessage != null) {
            message = ActionLibMessagesUtils.getSubMessageFromMessage(this.goalMessage, GET_GOAL_METHOD_NAME);
        }
        return message;
    }

    /**
     * @param message
     */
    public final void setGoalMessage(final Message message) {
        if (this.goalMessage != null) {
            ActionLibMessagesUtils.setSubMessageFromMessage(this.goalMessage, message, SET_GOAL_METHOD_NAME);
        }
    }

    @Override
    public final boolean equals(final Object object) {
        if (this == object) return true;
        if (!(object instanceof ActionGoal)) return false;
        final ActionGoal<?> that = (ActionGoal<?>) object;
        return getGoalMessage().equals(that.getGoalMessage());
    }

    @Override
    public final int hashCode() {
        return Objects.hash(getGoalMessage());
    }


}

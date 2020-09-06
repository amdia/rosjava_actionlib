/*
 * Copyright (C) 202 Spyros Koukas, spyroskoukas(at)hotmail.com
 * Copyright (C) 2015 Ernesto Corbellini, ecorbellini@ekumenlabs.com
 * Copyright (C) 2011 Alexander Perzylo, Technische Universität München
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava_actionlib;

import actionlib_msgs.GoalID;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;

import java.util.concurrent.atomic.AtomicLong;

/**
 * The GoalIDGenerator may be used to create unique GoalIDs.
 * <p>
 * <p>
 * The node's nodeName will be used and the time on the node .
 *
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * @author Spyros Koukas
 */
final class GoalIDGenerator {
    /**
     * A global ID which provide a count for each goal id.
     */
    private static final AtomicLong goalCount = new AtomicLong(0);

    /**
     * Unique nodeName to prepend to the goal id. This will generally be a fully
     * qualified connectedNode nodeName.
     */
    private final ConnectedNode connectedNode;
    private final String nodeNamePlusSeparator;
    private static final String separator = "-";


    /**
     * Constructor to create a GoalIDGenerator using a unique nodeName to prepend to
     * the goal id. This will generally be a fully qualified connectedNode nodeName.
     *
     * @param connectedNode The connectedNode used to generate IDs. The connectedNode's full nodeName should be
     *                      unique in the system.
     */
    GoalIDGenerator(final ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        this.nodeNamePlusSeparator = connectedNode.getName().toString() + separator;
    }

    /**
     * Creates a GoalID object with an unique id and a timestamp of the current
     * time.
     *
     * @return GoalID object
     */
    final String generateID(final GoalID goalId) {
        final Time currentTime = connectedNode.getCurrentTime();
        final String id = this.nodeNamePlusSeparator + goalCount.incrementAndGet();
        goalId.setId(id);
        goalId.setStamp(currentTime);

        return id;
    }
}

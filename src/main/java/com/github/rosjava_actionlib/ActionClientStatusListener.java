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

import actionlib_msgs.GoalStatusArray;


/**
 * Listener interface to receive the incoming Status messages from the ActionLib server.
 * A client should implement this interface if it wants to receive the callbacks
 * with information from the server.
 *
 * @author Spyros Koukas
 * @author Ernesto Corbellini ecorbellini@ekumenlabs.com
 *
 */
public interface ActionClientStatusListener{


    /**
     * Called when a status message is received from the server.
     *
     * @param status The status message received from the server.
     * @see GoalStatusArray
     */
    void statusReceived(GoalStatusArray status);
}

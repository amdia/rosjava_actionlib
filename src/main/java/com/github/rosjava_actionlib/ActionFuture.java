/**
 * Copyright 2020 Spyros Koukas
 *
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

import java.util.concurrent.Future;

/**
 * @author Spyros Koukas
 * @param <T_ACTION_GOAL>
 * @param <T_ACTION_FEEDBACK>
 * @param <T_ACTION_RESULT>
 */
public interface ActionFuture<T_ACTION_GOAL extends Message, T_ACTION_FEEDBACK extends Message, T_ACTION_RESULT extends Message> extends Future<T_ACTION_RESULT> {

    /**
     *
     * @return
     */
    public T_ACTION_FEEDBACK getLatestFeedback();

    /**
     *
     * @return
     */
    public ClientState getCurrentState();

    /**
     *
     * @return
     */
    public Future<Void> toVoidFuture();

    /**
     * 
     * @return
     */
    public Future<Boolean> toBooleanFuture();
}


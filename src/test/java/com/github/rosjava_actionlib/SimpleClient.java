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
import actionlib_msgs.GoalStatus;
import actionlib_msgs.GoalStatusArray;
import actionlib_tutorials.*;
import org.apache.commons.lang3.exception.ExceptionUtils;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.message.Duration;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import java.util.List;


/**
 * Class to test the actionlib client.
 *
 * @author Ernesto Corbellini ecorbellini@ekumenlabs.com
 * @author Spyros Koukas
 */
public class SimpleClient extends AbstractNodeMain implements ActionClientListener<FibonacciActionFeedback, FibonacciActionResult> {
    static {
        // comment this line if you want logs activated
        System.setProperty("org.apache.commons.logging.Log", "org.apache.commons.logging.impl.NoOpLog");
    }

    private static Log LOGGER = LogFactory.getLog(SimpleClient.class);

    private ActionClient actionClient = null;
    private volatile boolean resultReceived = false;
    private volatile boolean isStarted = false;


    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("fibonacci_test_client");
    }


    /**
     * Getter for isStarted
     *
     * @return isStarted
     **/
    public void waitForStart() {
        while (!this.isStarted) {
            try {
                Thread.sleep(5);
            } catch (final InterruptedException ie) {
                LOGGER.error(ExceptionUtils.getStackTrace(ie));
            } catch (final Exception e) {
                LOGGER.error(ExceptionUtils.getStackTrace(e));
            }
        }
    }

    public void startTasks() {
        Duration serverTimeout = new Duration(20);
        boolean serverStarted;
        

        // Attach listener for the callbacks
        actionClient.addListener(this);
        System.out.println("\nWaiting for action server to start...");
        serverStarted = actionClient.waitForActionServerToStart(new Duration(20));
        if (serverStarted) {
            System.out.println("Action server started.bouh\n");
        } else {
            System.out.println("No actionlib server found after waiting for " + serverTimeout.totalNsecs() / 1e9 + " seconds!");
            System.exit(1);
        }
        try {
			Thread.sleep(3000);
		} catch (InterruptedException ex) {
		}
        // Create Fibonacci goal message
        final FibonacciActionGoal goalMessage = (FibonacciActionGoal) actionClient.newGoalMessage();
        final FibonacciGoal fibonacciGoal = goalMessage.getGoal();
        // set Fibonacci parameter
        fibonacciGoal.setOrder(6);
        System.out.println("Sending goal...");
        actionClient.sendGoal(goalMessage);
        final GoalID gid1 = goalMessage.getGoalId();
        System.out.println("Sent goal with ID: " + gid1.getId());
        System.out.println("Waiting for goal to complete...");
        int c = 0;
        
        while (actionClient.getGoalState() != ClientState.DONE && c<1000) {
            sleep(1);
            c++;
        }
//        System.out.println("Goal completed!\n");

        System.out.println("Sending a new goal...");
        actionClient.sendGoal(goalMessage);
        final GoalID gid2 = goalMessage.getGoalId();
        System.out.println("Sent goal with ID: " + gid2.getId());
        while (actionClient.getGoalState() != ClientState.DONE) {
        	sleep(1);
    	}
        System.out.println("Goal completed!\n");
//        System.out.println("Cancelling this goal...");
//        actionClient.sendCancel(gid2);
//        while (actionClient.getGoalState() != ClientState.DONE) {
//            sleep(1);
//        }
//        System.out.println("Goal cancelled succesfully.\n");
//        System.out.println("Bye!");

    }


    @Override
    public void onStart(ConnectedNode node) {
        actionClient = new ActionClient<FibonacciActionGoal, FibonacciActionFeedback, FibonacciActionResult>(node, "/fibonacci", FibonacciActionGoal._TYPE, FibonacciActionFeedback._TYPE, FibonacciActionResult._TYPE);

        this.isStarted = true;
        // to uncomment to run in roslaunch
        startTasks();
//        System.exit(0);
    }

    @Override
    public void resultReceived(FibonacciActionResult message) {
        FibonacciResult result = message.getResult();
        int[] sequence = result.getSequence();
        int i;

        resultReceived = true;
        System.out.print("Got Fibonacci result sequence: ");
        for (i = 0; i < sequence.length; i++)
            System.out.print(Integer.toString(sequence[i]) + " ");
        System.out.println("");
    }

    @Override
    public void feedbackReceived(FibonacciActionFeedback message) {
        FibonacciFeedback result = message.getFeedback();
        int[] sequence = result.getSequence();
        int i;

        System.out.print("Feedback from Fibonacci server: ");
        for (i = 0; i < sequence.length; i++)
            System.out.print(Integer.toString(sequence[i]) + " ");
        System.out.print("\n");
    }

    @Override
    public void statusReceived(final GoalStatusArray status) {
//        if(LOGGER.isInfoEnabled()) {
            List<GoalStatus> statusList = status.getStatusList();
            for (GoalStatus gs : statusList) {
            	System.out.println("GoalID: " + gs.getGoalId().getId() + " -- GoalStatus: " + gs.getStatus() + " -- " + gs.getText());
            }
            System.out.println("Current state of our goal: " + actionClient.getGoalState());
//        }
    }

    /**
     *
     * @param msec
     */
    void sleep(final long msec) {
        try {
            Thread.sleep(msec);
        } catch (InterruptedException ex) {
        }
    }
}

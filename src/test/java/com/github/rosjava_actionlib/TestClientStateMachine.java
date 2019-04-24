package com.github.rosjava_actionlib;

import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.junit.Assert.*;

public class TestClientStateMachine {
    private ClientStateMachine subject;
    private static final ClientState INITIAL_CLIENT_STATE=ClientState.ERROR;
    // Executes before each test.
    @Before
    public void setUp() {
        subject = new ClientStateMachine(ClientState.ERROR);
    }

    @Test
    public void testGetState() {
        ClientState expectedState = ClientState.WAITING_FOR_GOAL_ACK;
        ClientState actualState;
        subject.setState(expectedState);
        actualState = subject.getState();
        assertEquals(expectedState, actualState);
    }

    @Test
    public void testSetState() {
        assertEquals(subject.getState(), INITIAL_CLIENT_STATE);
        ClientState expectedState = ClientState.WAITING_FOR_GOAL_ACK;
        assertNotEquals(INITIAL_CLIENT_STATE, ClientState.WAITING_FOR_GOAL_ACK);
        subject.setState(expectedState);
        assertEquals(expectedState, subject.getState());
    }

    @Test
    public void testUpdateStatusWhenStateIsNotDone() {
        ClientState expectedStatus = ClientState.DONE;
        subject.setState(ClientState.WAITING_FOR_GOAL_ACK);
        assertEquals(ClientState.WAITING_FOR_GOAL_ACK, subject.getLatestGoalStatus());
        subject.updateStatus(expectedStatus);
        assertEquals(expectedStatus, subject.getLatestGoalStatus());
    }

    @Test
    public void testUpdateStatusWhenStateIsDone() {
        subject.setState(ClientState.DONE);
        assertEquals(ClientState.WAITING_FOR_GOAL_ACK, subject.getLatestGoalStatus());
        subject.updateStatus(ClientState.DONE);
        assertEquals(ClientState.WAITING_FOR_GOAL_ACK, subject.getLatestGoalStatus());
    }

    @Test
    public void testCancelOnCancellableStates() {
        checkCancelOnInitialCancellableState(ClientState.WAITING_FOR_GOAL_ACK);
        checkCancelOnInitialCancellableState(ClientState.PENDING);
        checkCancelOnInitialCancellableState(ClientState.ACTIVE);
    }

    @Test
    public void testCancelOnNonCancellableStates() {
        checkCancelOnInitialNonCancellableState(ClientState.INVALID_TRANSITION);
        checkCancelOnInitialNonCancellableState(ClientState.NO_TRANSITION);
        checkCancelOnInitialNonCancellableState(ClientState.WAITING_FOR_RESULT);
        checkCancelOnInitialNonCancellableState(ClientState.WAITING_FOR_CANCEL_ACK);
        checkCancelOnInitialNonCancellableState(ClientState.RECALLING);
        checkCancelOnInitialNonCancellableState(ClientState.PREEMPTING);
        checkCancelOnInitialNonCancellableState(ClientState.DONE);
        checkCancelOnInitialNonCancellableState(ClientState.LOST);
    }

    private void checkCancelOnInitialCancellableState(ClientState initialState) {
        subject.setState(initialState);
        assertTrue("Failed test on initial state " + initialState, subject.cancel());
        assertEquals("Failed test on initial state " + initialState, ClientState.WAITING_FOR_CANCEL_ACK, subject.getState());
    }


    private void checkCancelOnInitialNonCancellableState(ClientState initialState) {
        subject.setState(initialState);
        assertFalse("Failed test on initial state " + initialState, subject.cancel());
        assertEquals("Failed test on initial state " + initialState, initialState, subject.getState());
    }

    @Test
    public void testResultReceivedWhileWaitingForResult() {
        subject.setState(ClientState.WAITING_FOR_RESULT);
        subject.resultReceived();
        assertEquals(ClientState.DONE, subject.getState());
    }

    @Test
    public void testResultReceivedWhileNotWaitingForResult() {
        checkResultReceivedWhileNotWaitingForResult(ClientState.INVALID_TRANSITION);
        checkResultReceivedWhileNotWaitingForResult(ClientState.NO_TRANSITION);
        checkResultReceivedWhileNotWaitingForResult(ClientState.WAITING_FOR_GOAL_ACK);
        checkResultReceivedWhileNotWaitingForResult(ClientState.PENDING);
        checkResultReceivedWhileNotWaitingForResult(ClientState.ACTIVE);
        checkResultReceivedWhileNotWaitingForResult(ClientState.WAITING_FOR_CANCEL_ACK);
        checkResultReceivedWhileNotWaitingForResult(ClientState.RECALLING);
        checkResultReceivedWhileNotWaitingForResult(ClientState.PREEMPTING);
        checkResultReceivedWhileNotWaitingForResult(ClientState.DONE);
        checkResultReceivedWhileNotWaitingForResult(ClientState.LOST);
    }

    private void checkResultReceivedWhileNotWaitingForResult(ClientState state) {
        subject.setState(state);
        subject.resultReceived();
        assertEquals("Failed test on initial state " + state, ClientState.ERROR, subject.getState());
    }

    @Test
    public void testGetTrasition() {

        LinkedList<ClientState>  expected = new LinkedList<>(Arrays.asList(ClientState.PENDING));
        checkGetTransition(ClientState.WAITING_FOR_GOAL_ACK,actionlib_msgs.GoalStatus.PENDING, expected);

        expected = new LinkedList<>(Arrays.asList(ClientState.PENDING,ClientState.WAITING_FOR_RESULT));
        checkGetTransition(ClientState.WAITING_FOR_GOAL_ACK,actionlib_msgs.GoalStatus.REJECTED, expected);
    }

    private void checkGetTransition(ClientState initialState, int goalStatus, List<ClientState> expected) {
        subject.setState(initialState);
        List<Integer> output = subject.getTransitionInteger(goalStatus);
        assertEquals(expected, output);
    }
}

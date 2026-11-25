package first.bearlib.statemachine;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.RobotState;

/**
 * A base class for a state machine utilizing commands v3.
 * States are created with {@code addState()}, with the name and action
 * associated with them as parameters.
 * Using the clean {@code state1.to(state2).when(conditon)}notation,
 * transitions in and out of states can be defined. The{@code start()}method
 * must be run at the end of a constructor in order for proper state
 * machine functionality. This library allows for fluidity in state machine
 * construction. Mechanisms can have more autonomy via their own BearMachine's,
 * or they can be semi autonomous when grouped into an overall state machine via
 * setState() w/in another state machine.
 * 
 * @see State
 * @see Transition
 */
public class BearMachine {

    // The state that executes
    private State currentState;

    // The state to enter on the next update, null if there is none
    private State requestedState;

    /**
     * Creates a {@link State} and registers it with this state machine. Only call
     * this while constructing the robot, not from within a command or opmode.
     * 
     * @param name   The name of the state.
     * @param action The action/behavior associated. Each state needs its own
     *               command instance, as it is canceled when the state is exited.
     */
    public State addState(String name, Command action) {
        State state = new State(this, name, action);
        // execute current state, again on enable since disabling cancels actions
        new Trigger(() -> currentState == state && RobotState.isEnabled()).onTrue(action);
        return state;
    }

    /**
     * Starts the state machine. Call once, at the end of a constructor.
     * 
     * @param initial The starting state.
     */
    public void start(State initial) {
        requireOwned(initial);
        if (currentState != null) {
            throw new IllegalStateException("STATE MACHINE ALREADY STARTED!!!");
        }
        // assign initial state
        currentState = initial;
        // run update in periodic
        Scheduler.getDefault().addPeriodic(this::update);
    }

    /**
     * Enters the next state, canceling the action of the state being exited.
     * Periodics run before triggers are polled, so the old action is gone before
     * the new one is scheduled.
     */
    protected void update() {
        State next = nextState();
        if (next == currentState) {
            return;
        }
        Scheduler.getDefault().cancel(currentState.action());
        currentState = next;
    }

    /**
     * The state to be in, from a request or a transition out of the current state.
     * Requests take priority over transitions.
     */
    private State nextState() {
        if (requestedState != null) {
            State next = requestedState;
            requestedState = null;
            return next;
        }
        for (Transition transition : currentState.transitions()) {
            // when transition condition is true, transition to goal
            if (transition.condition().getAsBoolean()) {
                return transition.goal();
            }
        }
        return currentState;
    }

    /**
     * Throws if a state was not created by this state machine.
     */
    private void requireOwned(State state) {
        if (state.machine() != this) {
            throw new IllegalArgumentException(state.name() + " BELONGS TO A DIFFERENT STATE MACHINE!!!");
        }
    }

    /**
     * Requests a state, entered on the next update. Thought of as entering a state,
     * as state transistions can still occur and override it. States from a
     * different state machine are reported and ignored.
     */
    public void setState(State state) {
        if (state.machine() != this) {
            // can happen mid match, so report instead of crashing
            DriverStationErrors.reportError("IGNORED " + state.name() + ", WRONG STATE MACHINE!!!", true);
            return;
        }
        this.requestedState = state;
    }

    /**
     * Sets the current state of the state machine, runs as a Command.
     */
    public Command setStateCommand(State state) {
        requireOwned(state);
        return Command.noRequirements(co -> {
            setState(state);
        }).named("SET STATE " + state.name().toUpperCase());
    }

    /**
     * The current/active state.
     */
    public State getCurrent() {
        return this.currentState;
    }
}

package first.bearlib.statemachine;

import java.util.List;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.framework.RobotBase;

/**
 * A base class for a single mechanism state machine utilizing commands v3.
 * States are declared as objects, with the name and action associated with them
 * as parameters.
 * Using the clean {@code state1.to(state2).when(conditon)}notation,
 * transitions in and out of states can be defined. The{@code configure()}method
 * must be run at the end of a constructor in order for proper state
 * machine functionality.
 * 
 * @see State
 * @see Transition
 */
public class BearState {

    // The state that executes
    private State currentState;

    /**
     * Adds mechanism states to internal list of States.
     * 
     * @param states All the states! first state should be the starting state.
     */
    public void configure(State... states) {
        // assign initial state
        currentState = states[0];
        // initialize state actions
        for (int i = 0; i < states.length; i++) {
            State state = states[i];
            // execute current state
            new Trigger(() -> currentState == state).whileTrue(state.action());
        }
        // run update in periodic
        Scheduler.getDefault().addPeriodic(this::update);
    }

    /**
     * Monitors transitions out of current state for state machine behavior.
     */
    private void update() {
        if (!RobotBase.isAutonomous()) {
            List<Transition> transitions = currentState.transitions();
            for (int i = 0; i < transitions.size(); i++) {
                Transition transition = transitions.get(i);
                // when transition condition is true, transition to goal
                if (transition.condition().getAsBoolean()) {
                    this.currentState = transition.goal();
                    // return so no uneeded iterating
                    return;
                }
            }
        }
    }

    /**
     * Sets the current state of the state machine.
     */
    public Command setState(State state) {
        return Command.noRequirements(co -> {
            currentState = state;
        }).named("SET STATE " + state.name().toUpperCase());
    }
}

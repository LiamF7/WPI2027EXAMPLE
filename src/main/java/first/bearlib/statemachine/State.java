package first.bearlib.statemachine;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import org.wpilib.command3.Command;
import first.bearlib.statemachine.Transition.NeedsConditionTransition;

/**
 * A simple action behavior for a mechanism.
 */
public class State {

    private final BearMachine machine;
    private final String name;
    private final Command action;
    private final ArrayList<Transition> transitions;
    private BooleanSupplier end;

    /**
     * A simple action behavior for a mechanism. Created with
     * {@link BearMachine#addState}.
     * 
     * @param machine The state machine this state belongs to.
     * @param name    The name of the state.
     * @param action  The action/behavior associated.
     */
    State(BearMachine machine, String name, Command action) {
        this.machine = machine;
        this.name = name;
        this.action = action;
        this.transitions = new ArrayList<>();
    }

    /**
     * Assigns origin and goal to a {@link Transition}, w/ this being the origin.
     * 
     * @param goal The goal {@link State}.
     */
    public NeedsConditionTransition to(State goal) {
        if (goal.machine != this.machine) {
            throw new IllegalArgumentException(
                    this.name + " AND " + goal.name + " ARE IN DIFFERENT STATE MACHINES!!!");
        }
        return new NeedsConditionTransition(this, goal);
    }

    /**
     * The state machine this state belongs to.
     */
    BearMachine machine() {
        return this.machine;
    }

    /**
     * The name of the {@link State}.
     */
    public String name() {
        return this.name;
    }

    /**
     * The assosciated action/behavior of the {@link State}.
     */
    public Command action() {
        return this.action;
    }

    /**
     * The transitions associated w/ this origin state.
     */
    public ArrayList<Transition> transitions() {
        return this.transitions;
    }

    /**
     * The state end condition.
     */
    public BooleanSupplier end() {
        return this.end;
    }

    /**
     * Assigns an end condition to a state.
     * 
     * @param stateEnd the end condition.
     */
    public State withEnd(BooleanSupplier stateEnd) {
        this.end = stateEnd;
        return this;
    }
}

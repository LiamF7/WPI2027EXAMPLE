package first.bearlib.statemachine;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import org.wpilib.command3.Command;
import first.bearlib.statemachine.Transition.NeedsConditionTransition;

/**
 * A simple action behavior for a mechanism.
 */
public class State {

    private final String name;
    private final Command action;
    private final ArrayList<Transition> transitions;
    private BooleanSupplier end;

    /**
     * A simple action behavior for a mechanism.
     * 
     * @param name   The name of the state.
     * @param action The action/behavior associated.
     */
    public State(String name, Command action) {
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
        return new NeedsConditionTransition(this, goal);
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

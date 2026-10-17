package first.bearlib.statemachine;

import java.util.function.BooleanSupplier;

/**
 * A defined path from an origin state to a goal state, that executes on a
 * given condition.
 */
public class Transition {

    private final State origin;
    private final State goal;
    private final BooleanSupplier condition;

    /**
     * ASSIGN A CONDITION TO THIS TRANSISTION!!!
     */
    public record NeedsConditionTransition(State origin, State goal) {

        /**
         * Assigns a given condition to a {@link Transition}.
         * 
         * @param condition The given condition.
         */
        public Transition when(BooleanSupplier condition) {
            Transition transition = new Transition(origin, goal, condition);
            origin.transitions().add(transition);
            return transition;
        }

        /**
         * Assigns a transition to execute when the origin ends.
         */
        public Transition onEnd(){
           return when(origin.end());
        }
    }

    /**
     * A defined path from an origin state to a goal state, that executes on a
     * given condition.
     * 
     * @param orign    The origin {@link State}.
     * @param goal     The goal {@link State}.
     * @param conditon The given condition
     */
    private Transition(State origin, State goal, BooleanSupplier condition) {
        this.origin = origin;
        this.goal = goal;
        this.condition = condition;
    }

    /**
     * The origin {@link State}.
     */
    public State origin() {
        return this.origin;
    }

    /**
     * The goal {@link State}.
     */
    public State goal() {
        return this.goal;
    }

    /**
     * The given condition.
     */
    public BooleanSupplier condition() {
        return this.condition;
    }
}

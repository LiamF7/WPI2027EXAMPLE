package first.robot.mechanisms.exampleMechanism;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;

import first.robot.Pilot;

public class ExampleState implements Mechanism {

    private final Example example;

    private final StateMachine exampleState = new StateMachine("ExampleState");

    public final State RUN;
    public final State STOP;

    public ExampleState(Example example) {

        this.example = example;

        // define states
        RUN = exampleState.addState(example.runVTCFOC(1000));
        STOP = exampleState.addState(example.stop());

        // report states
        STOP.onEnter(() -> report("STOP"));
        RUN.onEnter(() -> report("RUN"));
    }

    /**
     * Configures the state transistions for the Example mechanism.
     */
    public void configureTeleopStates() {

        // set intial state
        exampleState.setInitialState(STOP);
        // run on pilot run
        STOP.switchTo(RUN).when(Pilot.run());
        // don't run on !pilot run
        RUN.switchTo(STOP).when(Pilot.run().negate());
    }

    public void configureAutonmousStates() {
        exampleState.setInitialState(STOP);
    }

    /**
     * Returns the example state machine for outside reference.
     */
    public StateMachine get() {
        return exampleState;
    }

    /**
     * Reports a transistion into a state.
     * 
     * @param name The name of the entered state.
     */
    private Command report(String name) {
        return run(coroutine -> {
            System.out.println(example.getName() + " -> " + name);
            coroutine.yield();
        }).named("REPORT " + name);
    }
}

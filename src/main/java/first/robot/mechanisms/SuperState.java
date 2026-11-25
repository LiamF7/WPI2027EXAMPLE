package first.robot.mechanisms;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;

import first.bearlib.statemachine.BearMachine;
import first.bearlib.statemachine.State;
import first.robot.mechanisms.elevator.Elevator;
import first.robot.mechanisms.manipulator.Arm;
import first.robot.mechanisms.manipulator.Roller;
import first.robot.teleop.Pilot;

public class SuperState extends BearMachine{

    private final Arm arm;
    private final Roller roller;
    private final Elevator elevator;

    public final State NEUTRAL;
    public final State COLLECT;
    public final State INTAKING;
    public final State SCORE;
    public final State SCORING;
    
    public SuperState(Arm arm, Roller roller, Elevator elevator){
        this.arm = arm;
        this.roller = roller;
        this.elevator = elevator;

        NEUTRAL = addState("NEUTRAL", neutral());
        COLLECT = addState("COLLECT", collect());
        INTAKING = addState("INTAKING", intaking());
        SCORE = addState("SCORE", score());
        SCORING = addState("SCORING", scoring());

        // only run the roller once the arm is out
        COLLECT.to(INTAKING).when(arm::isOut);
        SCORE.to(SCORING).when(arm::isOut);

        start(NEUTRAL);
    }
    
    // These only request mechanism states, so they require no mechanisms. Requiring
    // them would interrupt the actions of the mechanism states.

    /** Returns all mechanisms to rest. */
    private Command neutral(){
        return Command.noRequirements(co -> {
            arm.getState().setState(arm.HOME);
            elevator.getState().setState(elevator.HOME);
            roller.getState().setState(roller.HOLD);
        }).named("SET NEUTRAL");
    }

    /** Moves into position to collect, w/ the roller off. */
    private Command collect(){
        return Command.noRequirements(co -> {
            elevator.getState().setState(elevator.MID);
            arm.getState().setState(arm.FORWARD);
            roller.getState().setState(roller.HOLD);
        }).named("SET COLLECT");
    }

    /** Runs the roller, once in position to collect. */
    private Command intaking(){
        return Command.noRequirements(co -> {
            roller.getState().setState(roller.RUN);
        }).named("SET INTAKING");
    }

    /** Moves into position to score, w/ the roller off. */
    private Command score(){
        return Command.noRequirements(co -> {
            elevator.getState().setState(elevator.HIGH);
            arm.getState().setState(arm.FORWARD);
            roller.getState().setState(roller.HOLD);
        }).named("SET SCORE");
    }

    /** Reverses the roller after a delay, once in position to score. */
    private Command scoring(){
        return Command.noRequirements(co -> {
            co.wait(Seconds.of(1.0));
            // disabling doesn't cancel commands w/o requirements, this runs again on enable
            if(RobotState.isEnabled()){
                roller.getState().setState(roller.REVERSE);
            }
        }).named("SET SCORING");
    }

    public void configureTeleopState(){
        Trigger intakeReleased = Pilot.intake().negate();
        Trigger scoreReleased = Pilot.score().negate();

        NEUTRAL.to(COLLECT).when(Pilot.intake()::getAsBoolean);
        COLLECT.to(NEUTRAL).when(intakeReleased::getAsBoolean);
        INTAKING.to(NEUTRAL).when(intakeReleased::getAsBoolean);

        NEUTRAL.to(SCORE).when(Pilot.score()::getAsBoolean);
        SCORE.to(NEUTRAL).when(scoreReleased::getAsBoolean);
        SCORING.to(NEUTRAL).when(scoreReleased::getAsBoolean);
    }
}

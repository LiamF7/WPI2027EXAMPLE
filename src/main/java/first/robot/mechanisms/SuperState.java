package first.robot.mechanisms;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;

import first.bearlib.statemachine.BearState;
import first.bearlib.statemachine.State;
import first.robot.mechanisms.elevator.Elevator;
import first.robot.mechanisms.manipulator.Arm;
import first.robot.mechanisms.manipulator.Roller;
import first.robot.teleop.Pilot;

public class SuperState extends BearState{

    private final Arm arm;
    private final Roller roller;
    private final Elevator elevator;

    public final State NEUTRAL;
    public final State COLLECT;
    public final State SCORE;
    
    public SuperState(Arm arm, Roller roller, Elevator elevator){
        this.arm = arm;
        this.roller = roller;
        this.elevator = elevator;

        NEUTRAL = new State("NEUTRAL", neutral());
        COLLECT = new State("COLLECT", run());
        SCORE = new State("SCORE", run());

        configure(NEUTRAL, COLLECT, SCORE);
    }
    
    private Command neutral(){
         return Command.requiring(roller, elevator, arm).executing(co -> {
            arm.getState().setState(arm.HOME);
            elevator.getState().setState(elevator.HOME);
            roller.getState().setState(roller.HOLD);
            co.yield();
        }).named("SET SCORE");
    }

    private Command collect(){
      return Command.noRequirements(co -> {
            elevator.getState().setState(elevator.MID);
            arm.getState().setState(arm.FORWARD);
            if(arm.isOut()){
                roller.getState().setState(roller.RUN);
                co.yield();
            }
        }).named("SET COLLECT");
    }

    private Command score(){
        return Command.noRequirements(co -> {
            elevator.getState().setState(elevator.HIGH);
            arm.getState().setState(arm.FORWARD);
            if(arm.isOut()){
                co.wait(Seconds.of(1.0));
                roller.getState().setState(roller.REVERSE);
                co.yield();
            }
        }).named("SET SCORE");
    }

    private Command run(){
        return Command.requiring(roller).executing(co -> {
            roller.getState().setState(roller.RUN);
            co.yield();
        }).named("RUN");
    }

    public void configureTeleopState(){
        NEUTRAL.to(COLLECT).when(Pilot.intake():: getAsBoolean);
        COLLECT.to(SCORE).when(Pilot.score():: getAsBoolean);
        SCORE.to(NEUTRAL).when(Pilot.score().negate():: getAsBoolean);
        COLLECT.to(NEUTRAL).when(Pilot.intake().negate():: getAsBoolean);
    }
}

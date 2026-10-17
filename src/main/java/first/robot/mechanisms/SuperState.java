package first.robot.mechanisms;

import org.wpilib.command3.Command;

import first.bearlib.statemachine.BearState;
import first.bearlib.statemachine.State;
import first.robot.mechanisms.manipulator.Arm;
import first.robot.mechanisms.manipulator.Roller;

public class SuperState extends BearState{

    private final Arm arm;
    private final Roller roller;

    private final State NEUTRAL;
    private final State COLLECT;
    private final State SCORE;
    
    public SuperState(Arm arm, Roller roller){
        this.arm = arm;
        this.roller = roller;

        NEUTRAL = new State("NEUTRAL", neutral());
        COLLECT = new State("COLLECT", collect()).withEnd(roller :: hasPiece);
        SCORE = new State("SCORE", score());

        COLLECT.to(NEUTRAL).onEnd();

        configure(NEUTRAL, COLLECT, SCORE);
    }

    private Command neutral(){
         return Command.noRequirements(co -> {
            arm.getState().setState(arm.HOME);
        }).named("SET SCORE");
    }

    private Command collect(){
      return Command.noRequirements(co -> {
            arm.getState().setState(arm.FORWARD);
            if(arm.isOut()){
                roller.getState().setState(roller.RUN);
            }
        }).named("SET COLLECT");
    }

    private Command score(){
        return Command.noRequirements(co -> {
            arm.getState().setState(arm.FORWARD);
            if(arm.isOut()){
                roller.getState().setState(roller.REVERSE);
            }
        }).named("SET SCORE");
    }
}

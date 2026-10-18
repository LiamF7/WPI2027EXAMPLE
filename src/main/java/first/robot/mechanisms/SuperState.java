package first.robot.mechanisms;

import org.wpilib.command3.Command;

import first.robot.Pilot;
import first.robot.mechanisms.manipulator.Arm;
import first.robot.mechanisms.manipulator.Roller;

public class SuperState {

    private final Arm arm;
    private final Roller roller;
    
    public SuperState(Arm arm, Roller roller){
        this.arm = arm;
        this.roller = roller;

        Pilot.intake().onTrue(collect()).onFalse(neutral());

        Pilot.score().onTrue(score()).onFalse(neutral());
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

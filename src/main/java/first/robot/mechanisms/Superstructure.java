package first.robot.mechanisms;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

import first.robot.mechanisms.manipulator.Arm;
import first.robot.mechanisms.manipulator.Roller;

public class Superstructure implements Mechanism {

    private final Roller roller;
    private final Arm arm;

    public Superstructure(Roller roller, Arm arm) {
        this.roller = roller;
        this.arm = arm;

        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    private void periodic() {
        applyStates();
    }

    // private WantedSuperState handleStateTransitions(){}

    private void applyStates() {}

    private void collect(){
        roller.setState(Roller.WantedState.RUN);
        arm.setState(Arm.WantedState.FORWARD);
            if(roller.hasPiece()){
                roller.setState(Roller.WantedState.STOP);
            }
    }

    private void armOut(){
        arm.setState(Arm.WantedState.FORWARD);
        roller.setState(Roller.WantedState.STOP);
    }

    private void release(){
        arm.setState(Arm.WantedState.FORWARD);
        roller.setState(Roller.WantedState.REVERSE);
    }

    private void home(){
        arm.setState(Arm.WantedState.HOME);
        roller.setState(Roller.WantedState.STOP);
    }
}
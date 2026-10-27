package first.robot.teleop.opmodes;

import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

import first.robot.Robot;
import first.robot.teleop.Pilot;

@Teleop(name = "Teleop")
public class DefaultTeleop implements OpMode{
    
    public DefaultTeleop(Robot robot){
            Pilot.intake().onTrue(robot.superState.setStateCommand(robot.superState.COLLECT))
        .onFalse(robot.superState.setStateCommand(robot.superState.NEUTRAL));

    Pilot.score().onTrue(robot.superState.setStateCommand(robot.superState.SCORE))
        .onFalse(robot.superState.setStateCommand(robot.superState.NEUTRAL));
    }
}

package first.robot.teleop.opmodes;

import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

import first.robot.Robot;

@Teleop(name = "Teleop")
public class DefaultTeleop implements OpMode {

    public DefaultTeleop(Robot robot) {
        robot.superState.configureTeleopState();
    }
}

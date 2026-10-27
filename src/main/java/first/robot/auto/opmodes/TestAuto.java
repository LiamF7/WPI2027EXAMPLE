package first.robot.auto.opmodes;

import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

import first.robot.Robot;

@Autonomous(name = "Test")
public class TestAuto implements OpMode{
    
    public TestAuto(Robot robot){
        robot.scheduler.schedule(robot.auton.getBuilder().build(robot.auton.test));
    }
}

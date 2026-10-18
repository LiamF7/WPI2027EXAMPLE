package first.robot;

import org.wpilib.framework.OpModeRobot;

import first.robot.mechanisms.manipulator.Arm;
import first.robot.mechanisms.manipulator.Roller;
import lombok.Getter;

public class Robot extends OpModeRobot {

  @Getter private final Arm arm;
  @Getter private final Roller roller;

  public Robot() {

    arm = new Arm();
    roller = new Roller();
    System.out.println("<<< ALL SUBSYSTEMS INITIALIZED >>>");

    configureBindings();
  }

  @Override
  public void robotPeriodic() {}

  public void configureBindings() {}
}

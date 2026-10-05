package first.robot;

import org.wpilib.framework.OpModeRobot;

public class Robot extends OpModeRobot {

  public Robot() {

    System.out.println("<<< ALL SUBSYSTEMS INITIALIZED >>>");

    configureBindings();
  }

  @Override
  public void robotPeriodic() {}

  public void configureBindings() {}
}

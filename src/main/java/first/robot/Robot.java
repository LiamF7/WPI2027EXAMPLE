package first.robot;

import org.wpilib.framework.OpModeRobot;

import first.robot.mechanisms.exampleMechanism.Example;
import first.robot.mechanisms.exampleMechanism.ExampleState;

public class Robot extends OpModeRobot {

  private final Example example;
  private final ExampleState exampleState;

  public Robot() {

    this.example = new Example();

    this.exampleState = new ExampleState(example);

    exampleState.configureTeleopStates();
  }

  @Override
  public void robotPeriodic() {
  }
}

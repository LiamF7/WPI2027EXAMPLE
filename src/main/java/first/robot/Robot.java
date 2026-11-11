package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.telemetry.Telemetry;

import first.robot.auto.Auton;
import first.robot.mechanisms.DriveSubsystem;
import first.robot.mechanisms.SuperState;
import first.robot.mechanisms.elevator.Elevator;
import first.robot.mechanisms.manipulator.Arm;
import first.robot.mechanisms.manipulator.Roller;

public class Robot extends OpModeRobot {

  public final Arm arm;

  public final Roller roller;

  public final Elevator elevator;

  public final DriveSubsystem driveSubsystem;

  public final SuperState superState;

  public final Auton auton;

  public final Scheduler scheduler = Scheduler.getDefault();

  public final Gamepad pilot = new Gamepad(0);

  public Robot() {

    this.arm = new Arm();
    this.roller = new Roller();
    this.elevator = new Elevator();
    this.driveSubsystem = new DriveSubsystem();

    System.out.println("--- All Mechanisms Initialized ---");

    this.superState = new SuperState(arm, roller, elevator);
    this.auton = new Auton(superState, driveSubsystem);
  }

  @Override
  public void robotPeriodic() {
    scheduler.run();

    Telemetry.getTable().log("SUPER STATE",superState.getCurrent().name());
  }
}

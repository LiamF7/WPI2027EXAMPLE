package first.robot.teleop;

import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.DriverStation;

/**
 * The primary robot controller. Utilizes triggers to combine driver input with
 * mechansim and robot input. Overall cleaner code as...
 * {@code (new Trigger(pilot.L1().getAsBoolean() && robotPose.inPosistion())).onTrue(shootCommand());}
 * can become {@code Pilot.shoot().onTrue(shootCommand());} when referenced in
 * other classes.
 */
public class Pilot {

    // driver controller
    // private static final CommandNiDsPS5Controller pilot = new CommandNiDsPS5Controller(0);

    private static final CommandGamepad pilot = new CommandGamepad(DriverStation.getGamepad(0)); 

    public Pilot() {}

    /**
     * Trigger for when the pilot wants to intake a piece.
     */
    public static Trigger intake() {
        return pilot.leftBumper();
    }

    /**
     * Trigger for when the pilot wants to score a piece.
     */
    public static Trigger score(){
        return pilot.rightBumper();
    }
}

package first.robot.teleop;

import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandNiDsPS5Controller;

/**
 * The primary robot controller. Utilizes triggers to combine driver input with
 * mechansim and robot input. Overall cleaner code as...
 * {@code (new Trigger(pilot.L1().getAsBoolean() && robotPose.inPosistion())).onTrue(shootCommand());}
 * can become {@code Pilot.shoot().onTrue(shootCommand());} when referenced in
 * other classes.
 */
public class Pilot {

    // driver controller
    private static final CommandNiDsPS5Controller pilot = new CommandNiDsPS5Controller(0);

    public Pilot() {}

    /**
     * Trigger for when the pilot wants to intake a piece.
     */
    public static Trigger intake() {
        return pilot.L1();
    }

    /**
     * Trigger for when the pilot wants to score a piece.
     */
    public static Trigger score(){
        return pilot.R1();
    }
}

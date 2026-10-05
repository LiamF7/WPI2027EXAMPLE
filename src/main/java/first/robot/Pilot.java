package first.robot;

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
     * Trigger for when the pilot presses down the right trigger/R1.
     */
    public static Trigger run() {
        return pilot.R1();
    }
}

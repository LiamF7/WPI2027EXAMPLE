package first.robot.auto;

import org.wpilib.math.controller.PIDController;

import first.bearlib.util.BearFlip;
import first.robot.mechanisms.DriveSubsystem;
import first.robot.mechanisms.SuperState;
import frc.robot.lib.BLine.commands.FollowPath;
import frc.robot.lib.BLine.commands.FollowPath.Builder;
import frc.robot.lib.BLine.following.DriveType;
import frc.robot.lib.BLine.path.Path;

/**
 * Class for event triggers for autonomous, along with autonomous
 * initialization.
 */
public class Auton {

    private final SuperState superState;
    private final Builder pathBuilder;

    public final Path test;

    public Auton(SuperState superState, DriveSubsystem driveSubsystem) {
        this.superState = superState;
        
        // setup path builder
        pathBuilder = new Builder(DriveType.SWERVE, driveSubsystem, driveSubsystem::getPose, driveSubsystem::setPose,
                driveSubsystem::getChassisVelocities, driveSubsystem::setChassisVelocities, new PIDController(0, 0, 0),
                new PIDController(0, 0, 0), new PIDController(0, 0, 0));

        registerEventTriggers();

        // define paths
        test = new Path("test").setFlipped(BearFlip.shouldFlip());  
    }

    /**
     * Registers the EventTrigger's for for auto.
     */
    private void registerEventTriggers() {
        FollowPath.registerEventTrigger("COLLECT", superState.setStateCommand(superState.COLLECT));
        FollowPath.registerEventTrigger("SCORE", superState.setStateCommand(superState.SCORE));
        FollowPath.registerEventTrigger("HOME", superState.setStateCommand(superState.NEUTRAL));
    }

    /**
     * The B-line path builder.
     */
    public Builder getBuilder(){
        return pathBuilder;
    }
}

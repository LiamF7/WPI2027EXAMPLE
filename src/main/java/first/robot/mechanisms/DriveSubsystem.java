package first.robot.mechanisms;

import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import lombok.Getter;
import lombok.Setter;

/**
 * Example DriveSubsystem, "dummy-methods" for B-line autonmous init stuff.
 */
public class DriveSubsystem implements Mechanism {
    @Getter @Setter 
    public Pose2d pose = new Pose2d(new Translation2d(), new Rotation2d());

    @Getter @Setter
    public ChassisVelocities chassisVelocities = new ChassisVelocities(0.0, 0.0, 0.0);

}

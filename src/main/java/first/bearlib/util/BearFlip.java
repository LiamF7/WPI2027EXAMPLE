package first.bearlib.util;

import java.util.Optional;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Utility class that translates WPILib's {@link Rotation2d},
 * {@link Translation2d}, and {@link Pose2d} objects based off alliance color.
 */
public class BearFlip {

    /**
     * Signals whether or not to flip (if alliance is red).
     * 
     * @return boolean, shouldFlip.
     */
    public static boolean shouldFlip() {
        return MatchState.getAlliance().isPresent() && MatchState.getAlliance().equals(Optional.of(Alliance.RED));
    }

    /**
     * Returns a {@link Translation2d} flipped based on alliance color.
     * 
     * @param translation2d The initial Translation2d to flip.
     * @return Translation2d, flippedTranslation
     */
    public static Translation2d apply(Translation2d translation2d) {
        Translation2d flipped = shouldFlip() ? new Translation2d(-translation2d.getX(), -translation2d.getY())
                : translation2d;
        return flipped;
    }

    /**
     * Returns a {@link Translation2d} mirrored based on alliance color.
     * 
     * @param translation2d The initial Translation2d to mirror.
     * @return Translation2d, mirroredTranslation.
     */
    public static Translation2d mirrorApply(Translation2d translation2d) {
        Translation2d flipped = shouldFlip() ? new Translation2d(-translation2d.getX(), translation2d.getY())
                : translation2d;
        return flipped;
    }

    /**
     * Returns a {@link Rotation2d} flipped based on alliance color.
     * 
     * @param rotation2d The initial Translation2d to flip.
     * @return Rotation2d, flippedRotation.
     */
    public static Rotation2d apply(Rotation2d rotation2d) {
        Rotation2d flipped = shouldFlip() ? rotation2d.rotateBy(Rotation2d.k180deg) : rotation2d;
        return flipped;
    }

    /**
     * Returns a {@link Pose2d} flipped based on alliance color.
     * 
     * @param pose2d The initial Pose2d to flip.
     * @return Translation2d, flippedPose
     */
    public static Pose2d apply(Pose2d pose2d) {
        Pose2d flipped = shouldFlip() ? new Pose2d(apply(pose2d.getTranslation()), apply(pose2d.getRotation()))
                : pose2d;
        return flipped;
    }

    /**
     * Returns a {@link Pose2d} mirrored based on alliance color
     * 
     * @param pose2d The intial Pose2d to flip
     * @return Pose2d, mirroredPose
     */
    public static Pose2d mirrorApply(Pose2d pose2d) {
        Pose2d flipped = shouldFlip() ? new Pose2d(mirrorApply(pose2d.getTranslation()), apply(pose2d.getRotation()))
                : pose2d;
        return flipped;
    }
}

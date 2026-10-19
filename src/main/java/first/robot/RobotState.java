package first.robot;

import org.wpilib.math.geometry.Pose2d;

import lombok.*;

public class RobotState {

    @Getter @Setter Pose2d robotPose;
    
    private static RobotState instance;
    
    public static RobotState getInstance(){
        return instance;
    }
}

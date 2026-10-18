package first.robot;

import lombok.*;

public class RobotState {
    
    private static RobotState instance;
    
    public static RobotState getInstance(){
        return instance;
    }
}

package first.robot;

import first.robot.mechanisms.SuperState;
import frc.robot.lib.BLine.commands.FollowPath;

/**
 * Class for event triggers for autonomous, along with autonomous
 * initialization.
 */
public class Auton {

    private final SuperState superState;

    public Auton(SuperState superState){
        this.superState = superState;
        
        registerEventTriggers();
    }

    private void registerEventTriggers(){
        FollowPath.registerEventTrigger("COLLECT", superState.setStateCommand(superState.COLLECT));
        FollowPath.registerEventTrigger("SCORE", superState.setStateCommand(superState.SCORE));
        FollowPath.registerEventTrigger("HOME", superState.setStateCommand(superState.NEUTRAL));
    }
}

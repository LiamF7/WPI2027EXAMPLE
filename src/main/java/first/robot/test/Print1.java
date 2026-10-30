package first.robot.test;

import java.util.Set;

import org.wpilib.internal.PeriodicPriorityQueue.Callback;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

@Teleop(name = "print one")
public class Print1 implements OpMode{
    
    public Print1(){
        System.out.println("OP MODE PRINT 1");
    }

    @Override
    public void start(){
        OpMode.super.start();
        System.out.println("OP MODE PRINT 1 started");    
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void end() {
        OpMode.super.end();
    }

    @Override
    public Set<Callback> getCallbacks() {
        return OpMode.super.getCallbacks();
    }

    @Override
    public void periodic() {
        OpMode.super.periodic();
    }
}

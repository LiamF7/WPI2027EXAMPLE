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
        // TODO Auto-generated method stub
        OpMode.super.end();
    }

    @Override
    public Set<Callback> getCallbacks() {
        // TODO Auto-generated method stub
        return OpMode.super.getCallbacks();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        OpMode.super.periodic();
    }
}

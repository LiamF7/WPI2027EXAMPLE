package first.robot.mechanisms.manipulator.elevator;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import first.bearlib.BearMechanism;

public class Elevator extends BearMechanism{
    
    public Elevator(){
        super("Elevator", 3, new CANBus("Default Name"));

        neutralMode(NeutralModeValue.Brake);
        inverted(InvertedValue.CounterClockwise_Positive);

        forwardSoftLimit(3.0);
        reverseSoftLimit(0.0);
        kP(10);
        kI(0.0);
        kD(0.0);
        kG(9.81);
        gravityType(GravityTypeValue.Elevator_Static);
        staticFeedForwardSign(StaticFeedforwardSignValue.UseVelocitySign);

        addConfig();
    }

}

package first.robot.mechanisms.elevator;

import org.wpilib.command3.Mechanism;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.bearlib.BearMechanism;

public class Elevator implements Mechanism {

    private final BearMechanism leader;
    private final BearMechanism follower;

    private final double GEAR_RATIO = 2.76;
    
    public Elevator(){
         
    leader = new BearMechanism("ElevatorLead", 5, new CANBus("Default Name"));
    follower = new BearMechanism("ElevatorFollow", 6, new CANBus("Default Name"));

    leader.neutralMode(NeutralModeValue.Brake);
    leader.inverted(InvertedValue.CounterClockwise_Positive);
    leader.sensorToMechanismRatio(GEAR_RATIO);
    leader.forwardSoftLimit(4.1);
    leader.reverseSoftLimit(0.0);
    leader.gravityType(GravityTypeValue.Elevator_Static);
    leader.kP(10);
    leader.kI(0.7);
    leader.kD(9.8);
    leader.kG(10);
    leader.kS(6.4);

    leader.addConfig();
    follower.cloneConfig(leader.getConfig());
    }

}

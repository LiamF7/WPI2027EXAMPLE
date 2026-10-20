package first.robot.mechanisms.elevator;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.bearlib.BearMechanism;
import first.bearlib.statemachine.BearState;
import first.bearlib.statemachine.State;
import lombok.Getter;

public class Elevator implements Mechanism {

    private final BearMechanism leader;
    private final BearMechanism follower;

    private final double GEAR_RATIO = 2.76;
    private final double tolerance = Rotations.of(0.2).in(Degrees);

    /**
     * Setpoint for the elevator, in rotations.
     */
    public enum Setpoint {
        HOME(Rotations.of(0.0).in(Rotations)),
        MID(Rotations.of(2.0).in(Rotations)),
        HIGH(Rotations.of(4.0).in(Rotations));

        public double rotations;

        private Setpoint(double rotations) {
            this.rotations = rotations;
        }
    }

    @Getter
    private final BearState state = new BearState();
    public final State HOME;
    public final State MID;
    public final State HIGH;

    public Elevator() {

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

        // State setup
        HOME = new State("HOME", this.goToSetpoint(Setpoint.HOME));
        MID = new State("MID", this.goToSetpoint(Setpoint.MID));
        HIGH = new State("HIGH", this.goToSetpoint(Setpoint.HIGH));

        leader.addConfig();
        follower.cloneConfig(leader.getConfig());
        state.configure(HOME, HIGH, MID);
    }

    /**
     * Runs the elevator to a specific setpoint as a {@link Setpoint}.
     * 
     * @param setpoint The specifc setpoint
     * @return {@link Command}
     */
    public Command goToSetpoint(Setpoint setpoint) {
        return run(co -> {
            co.fork(leader.runPTCFOC(setpoint.rotations));
            co.fork(follower.runPTCFOC(setpoint.rotations));
        }).named(getName() + " SETPOINT: " + setpoint.rotations);
    }

    /**
     * Signals if the elevator is at its setpoint.
     */
    public boolean isAtSetpoint() {
        return Math.abs(leader.getSetpoint() - leader.getAngle()) < tolerance
                && Math.abs(follower.getSetpoint() - follower.getAngle()) < tolerance;
    }

}

package first.robot.mechanisms.manipulator;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import org.wpilib.command3.Command;
import org.wpilib.units.measure.Angle;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.bearlib.BearMechanism;
import first.bearlib.statemachine.BearState;
import first.bearlib.statemachine.State;
import lombok.Getter;

public class Arm extends BearMechanism {

    private final double GEAR_RATIO = 1.0;

    private final double TOLERANCE = Degrees.of(5).in(Degrees);

    @Getter private final BearState state = new BearState();

    public final State HOME;

    public final State IDLE;

    public final State FORWARD;
    
    public final State BACK;

    private enum Setpoint {
        HOME(Rotations.of(0.0)),
        FORWARD(Rotations.of(-.25)),
        BACK(Rotations.of(.25));

        public final Angle angle;

        private Setpoint(Angle angle) {
            this.angle = angle;
        }
    }

    public Arm() {
        super("Arm", 2, new CANBus("Default Name"));

        neutralMode(NeutralModeValue.Brake);
        inverted(InvertedValue.Clockwise_Positive);
        forwardSoftLimit(0.3);
        reverseSoftLimit(-0.5);
        sensorToMechanismRatio(GEAR_RATIO);
        kP(1.67);
        kI(0.003);
        kD(0.035);
        kG(9.81);
        gravityType(GravityTypeValue.Arm_Cosine);
        gravityArmPositionOffset(0.25);

        HOME = new State("HOME", goToSetpoint(Setpoint.HOME)).withEnd(this::isAtHome);

        IDLE = new State("IDLE", brake());

        FORWARD = new State("FORWARD", goToSetpoint(Setpoint.FORWARD));

        BACK = new State("BACK", goToSetpoint(Setpoint.BACK));

        HOME.to(IDLE).onEnd();

        addConfig();
        state.configure(IDLE, HOME, FORWARD, BACK);
    }

    private Command goToSetpoint(Setpoint setpoint) {
        return runPV(setpoint.angle.in(Rotations));
    }

    public boolean isAtHome() {
        return Math.abs(getAngle() - Setpoint.HOME.angle.in(Degrees)) <= TOLERANCE;
    }

    public boolean isOut() {
        return Math.abs(getAngle() - Setpoint.FORWARD.angle.in(Degrees)) <= TOLERANCE;
    }
}

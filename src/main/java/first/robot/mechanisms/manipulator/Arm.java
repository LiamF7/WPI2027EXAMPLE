package first.robot.mechanisms.manipulator;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import org.wpilib.units.measure.Angle;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.bearlib.BearMechanism;

public class Arm extends BearMechanism {

    private final double GEAR_RATIO = 1.0;

    private final double TOLERANCE = Degrees.of(5).in(Degrees);

    private WantedState wantedState = WantedState.HOME;

    private SystemState systemState = SystemState.HOME;

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

        addConfig();
    }

    @Override
    public void periodic() {
        super.periodic();

        systemState = handleStateTransitions();
        applyStates();
    }

    private enum Setpoint {
        // arm setpoint
        HOME(Rotations.of(0.0)),
        FORWARD(Rotations.of(-.25)),
        BACK(Rotations.of(.25));

        public final Angle angle;

        private Setpoint(Angle angle) {
            this.angle = angle;
        }
    }

    public enum WantedState {
        HOME,
        FORWARD,
        BACK,
    }

    private enum SystemState {
        HOME,
        HOMING,
        FORWARD,
        BACK,
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case HOME: {
                if (isAtHome()) {
                    yield SystemState.HOME;
                }
                yield SystemState.HOMING;
            }
            case FORWARD: {
                yield SystemState.FORWARD;
            }
            case BACK: {
                yield SystemState.BACK;
            }
        };
    }

    private void applyStates() {
        switch (systemState) {
            case HOME -> setBrake();
            case HOMING -> runTo(Setpoint.HOME);
            case FORWARD -> runTo(Setpoint.FORWARD);
            case BACK -> runTo(Setpoint.BACK);
        }
    }

    public void setState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    private void runTo(Setpoint setpoint) {
        setPV(setpoint.angle.in(Rotations));
    }

    public boolean isAtHome() {
        return Math.abs(getAngle() - Setpoint.HOME.angle.in(Degrees)) <= TOLERANCE;
    }

    public boolean isOut() {
        return Math.abs(getAngle() - Setpoint.FORWARD.angle.in(Degrees)) <= TOLERANCE;
    }
}

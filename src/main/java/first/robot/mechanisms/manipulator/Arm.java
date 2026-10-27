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

    @Getter
    private final BearState state = new BearState();

    public final State HOME;

    public final State IDLE;

    public final State FORWARD;

    public final State BACK;

    /**
     * Setpoint for the arm, in rotations.
     * 
     * @see {@link rotations}
     */
    private enum Setpoint {
        HOME(Rotations.of(0.0)),
        FORWARD(Rotations.of(-.25)),
        BACK(Rotations.of(.25));

        public final Angle angle;

        private Setpoint(Angle angle) {
            this.angle = angle;
        }
    }

    private final double selfTestOutput = Setpoint.FORWARD.angle.in(Degrees); // 5 degrees

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

        // setup state machine
        HOME = new State("HOME", goToSetpoint(Setpoint.HOME)).withEnd(this::isAtHome);
        IDLE = new State("IDLE", this.idle());
        FORWARD = new State("FORWARD", this.goToSetpoint(Setpoint.FORWARD));
        BACK = new State("BACK", this.goToSetpoint(Setpoint.BACK));
        HOME.to(IDLE).onEnd();

        addConfig();
        state.configure(IDLE, HOME, FORWARD, BACK);
        selfTestInit(SelfTestableType.POSITION_VOLTAGE, selfTestOutput, selfTestOutput, isOut(), 15.0);
    }

    /**
     * Runs the arm to a specifc setpoint as a {@link Setpoint}.
     * 
     * @param setpoint the specific setpoint.
     * @return {@link Command}
     */
    private Command goToSetpoint(Setpoint setpoint) {
        return run(co -> {
            this.setPV(setpoint.angle.in(Rotations));
            co.park();
        }).named("ARM GO TO" + setpoint);
    }

    /**
     * Signals whether the arm is at the home setpoint.
     */
    public boolean isAtHome() {
        return Math.abs(getAngle() - Setpoint.HOME.angle.in(Degrees)) <= TOLERANCE;
    }

    /**
     * Signals whether the arm is at the extended setpoint.
     */
    public boolean isOut() {
        return Math.abs(getAngle() - Setpoint.FORWARD.angle.in(Degrees)) <= TOLERANCE;
    }
}

package first.robot.mechanisms.manipulator;

import static org.wpilib.units.Units.Amps;

import org.wpilib.command3.Command;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.bearlib.BearMechanism;
import first.bearlib.statemachine.BearState;
import first.bearlib.statemachine.State;
import lombok.Getter;

public class Roller extends BearMechanism {

    private final double GEAR_RATIO = 1.0;

    private final double PIECE_THRESHOLD = Amps.of(3.879).in(Amps);

    @Getter private final BearState state = new BearState();

    public final State RUN;

    public final State HOLD;

    public final State REVERSE;

    public Roller() {
        super("Roller", 0, new CANBus("Default Name"));

        neutralMode(NeutralModeValue.Brake);
        inverted(InvertedValue.CounterClockwise_Positive);
        sensorToMechanismRatio(GEAR_RATIO);
        kP(10);
        kI(0);
        kD(0);

        // setup state machine
        RUN = new State("RUN", runAtSpeed(1000)).withEnd(this::hasPiece);
        HOLD = new State("HOLD", this.idle());
        REVERSE = new State("REVERSE", runAtSpeed(-1000)).withEnd(() -> !hasPiece());
        RUN.to(HOLD).onEnd();
        REVERSE.to(HOLD).onEnd();

        addConfig();
        state.configure(HOLD, RUN, REVERSE);
    }

    /**
     * Signals whether the roller has a piece.
     */
    public boolean hasPiece() {
        return getSupplyCurrent() >= PIECE_THRESHOLD;
    }

    /**
     * Signals whether the roller is stopped.
     */
    public boolean isStopped() {
        return Math.abs(getVelocity()) < 0.5;
    }

    public Command runAtSpeed(double rpm){
        return run(co->{
            this.setVTCFOC(rpm);
            co.park();
        }).named("ROLLER RUN @" + rpm);
    }
}

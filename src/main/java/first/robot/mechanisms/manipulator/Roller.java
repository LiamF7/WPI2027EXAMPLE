package first.robot.mechanisms.manipulator;

import static org.wpilib.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.bearlib.BearMechanism;
import first.bearlib.statemachine.BearState;
import first.bearlib.statemachine.State;
import first.robot.Pilot;

public class Roller extends BearMechanism {

    private final double GEAR_RATIO = 1.0;

    private final double PIECE_THRESHOLD = Amps.of(3.879).in(Amps);

    private final BearState state = new BearState();

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

        RUN = new State("RUN", runVTCFOC(1000)).withEnd(this::hasPiece);

        HOLD = new State("HOLD", stop());

        REVERSE = new State("REVERSE", runVTCFOC(-1000));

        HOLD.to(RUN).when(Pilot.intake()::getAsBoolean);

        RUN.to(HOLD).onEnd();

        HOLD.to(REVERSE).when(Pilot.score()::getAsBoolean);

        REVERSE.to(HOLD).when(() -> !hasPiece());

        addConfig();
        state.configure(HOLD, RUN, REVERSE);
    }

    public boolean hasPiece() {
        return getSupplyCurrent() >= PIECE_THRESHOLD;
    }

    public boolean isStopped() {
        return Math.abs(getVelocity()) < 0.5;
    }
}

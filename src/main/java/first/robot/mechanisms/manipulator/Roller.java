package first.robot.mechanisms.manipulator;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.RobotBase;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.bearlib.BearMechanism;
import first.bearlib.BearSim;
import first.bearlib.statemachine.BearState;
import first.bearlib.statemachine.State;
import first.robot.teleop.Pilot;
import lombok.Getter;

public class Roller extends BearMechanism {

    private final double GEAR_RATIO = 1.0;

    private final double PIECE_THRESHOLD = Amps.of(3.879).in(Amps);

    private final TelemetryTable table = Telemetry.getTable("Roller");

    @Getter
    private final BearState state = new BearState();

    public final State RUN = new State("RUN", runAtSpeed(1020));
    public final State HOLD = new State("HOLD", runAtSpeed(0.0));
    public final State SubLevel = new State("SUB", runAtSpeed(1020));
    public final State REVERSE = new State("REVERSE", runAtSpeed(-1020));

    private RollerSim rollerSim;

    class RollerSim extends BearSim{
        public RollerSim(TalonFX ... motors){
            super(motors);
            flywheel44(GEAR_RATIO, 0.0001);
        }
    }

    public Roller() {
        super("Roller", 0, new CANBus("Default Name"));

        neutralMode(NeutralModeValue.Brake);
        inverted(InvertedValue.CounterClockwise_Positive);
        sensorToMechanismRatio(GEAR_RATIO);
        kP(0.006);
        kV(0.093);
        kS(0.011);

        Scheduler.getDefault().addPeriodic(this::log);

        RUN.to(SubLevel).when(()-> getVelocity() > 500);

        addConfig();
        state.configure(HOLD, RUN, REVERSE);

        if(RobotBase.isSimulation()){
            rollerSim = new RollerSim(this.getMotor());
            Scheduler.getDefault().addPeriodic(rollerSim :: flywheelPeriodic);
        }
    }

    /**
     * Signals whether the roller has a piece.
     */
    @Logged
    public boolean hasPiece() {
        return getSupplyCurrent() >= PIECE_THRESHOLD;
    }

    /**
     * Signals whether the roller is stopped.
     */
    public boolean isStopped() {
        return Math.abs(getVelocity()) < 0.5;
    }

    /**
     * Runs the Roller at a specifc rpm.
     * 
     * @param rpm The specific rpm.
     * @return {@link Command}
     */
    public Command runAtSpeed(double rpm) {
        return run(co -> {
            this.setVV(rpm);
        }).named("ROLLER RUN @" + rpm);
    }

    /**
     * The target speed in rpm.
     * 
     * @return rpm.
     */
    public double targetSpeed(){
        return vv.getVelocityMeasure().in(RPM);
    }

    @Override
    public void log() {
        table.log("velocity", this.getVelocity());
        table.log("stopped?", this.isStopped());
        table.log("state", state.getCurrent().name());
        table.log("setpoint", this.targetSpeed());
        table.log("press", Pilot.intake().getAsBoolean());
    }
}

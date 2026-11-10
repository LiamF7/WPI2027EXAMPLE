package first.robot.mechanisms.elevator.climber;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.command3.Scheduler;
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
import lombok.Getter;

public class Climber extends BearMechanism {

    private final double GEAR_RATIO = 2.0;

    private final double TOLERANCE = Degrees.of(5).in(Degrees);

    private final TelemetryTable table = Telemetry.getTable("Climber");

    private ClimberSim climberSim;

    @Getter
    private final BearState state = new BearState();

    class ClimberSim extends BearSim {
        public ClimberSim(TalonFX... motors) {
            super(motors);
            arm60(2.0, 8, GEAR_RATIO, GEAR_RATIO, TOLERANCE, GEAR_RATIO);
        }
    }

    public Climber() {
        super("Climber", 2, new CANBus("Default Name"));

        neutralMode(NeutralModeValue.Brake);
        inverted(InvertedValue.Clockwise_Positive);
        forwardSoftLimit(0.25);
        reverseSoftLimit(-0.01);
        sensorToMechanismRatio(GEAR_RATIO);
        kP(0);
        kI(0);
        kD(0);
        kG(0);
        gravityArmPositionOffset(0.25);

        if (RobotBase.isSimulation()) {
            climberSim = new ClimberSim(this.getMotor());
            Scheduler.getDefault().addPeriodic(climberSim::elevatorPeriodic);
        }
    }

    @Override
    public void log() {
        table.log("position", this.getAngle());
        table.log("voltage", this.getMotorVoltageMeasure());
    }
}

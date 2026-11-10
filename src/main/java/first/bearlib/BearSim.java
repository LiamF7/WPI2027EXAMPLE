package first.bearlib;

import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * A simulated kraken X44 or X60 mechanism. Utilizes WPILib {@link FlywheelSim},
 * {@link SingleJointedArmSim}, {@link ElevatorSim}. 
 */
public class BearSim {

    /** Simulated flywheel mechanism. */
    private FlywheelSim flywheelSim;

    /** Simulated arm mechanism. */
    private SingleJointedArmSim armSim;

    /** Simulated elevator mechanism. */
    private ElevatorSim elevatorSim;

    /** The motors involved in the simulated mechanism. */
    private TalonFX[] motors;

    /** The drum radius of a simulated elevator mechanism. */
    private double drumRadius;

    /** The gear ratio of the simulated mechanism */
    private double gearRatio;

    /**
     * @param motors Motors involved in simulated mechanism.
     */
    public BearSim(TalonFX... motors) {
        this.motors = motors;
    }

    /**
     * Constructs a {@link FlywheelSim} for a CTRE Kraken X44 flywheel mechanism.
     * 
     * @param gearRatio The gear ratio of the flywheel.
     * @param J         The inertia of the flywheel.
     * 
     * @see FlywheelSim
     */
    protected void flywheel44(double gearRatio, double J) {
        this.gearRatio = gearRatio;
        DCMotor motor = DCMotor.getKrakenX44(motors.length);
        LinearSystem<N1, N1, N1> system = Models.flywheelFromPhysicalConstants(motor, J, gearRatio);
        this.flywheelSim = new FlywheelSim(system, motor);
    }

    /**
     * Constructs a {@link SingleJointedArmSim} for a CTRE Kraken X44 arm mechanism.
     * 
     * @param gearRatio    The gear ratio of the arm.
     * @param armLength    The length of the arm.
     * @param minAngleRads The minimum rotations of the arm (in radians).
     * @param maxAngleRads The maximum rotations of the arm (in radians).
     * @param startRads    The starting rotations of the arm (in radians).
     * @param J            The inertia of the arm.
     * 
     * @see SingleJointedArmSim
     */
    protected void arm44(double gearRatio, double armLength, double minAngleRads,
            double maxAngleRads, double startRads, double J) {
        this.gearRatio = gearRatio;
        DCMotor dcMotor = DCMotor.getKrakenX44(motors.length);
        LinearSystem<N2, N1, N2> system = Models
                .singleJointedArmFromPhysicalConstants(dcMotor, J, gearRatio);
        this.armSim = new SingleJointedArmSim(system, dcMotor, gearRatio, armLength, minAngleRads, maxAngleRads, true,
                startRads);
    }

    /**
     * Constructs a {@link ElevatorSim} for a CTRE kraken X44 elevator mechanism.
     * 
     * @param mass        The elevator carriage mass.
     * @param gearRatio   The gear ratio of the elevator.
     * @param drumRadius  The radius of the elevator drum.
     * @param minHeight   The minimum height of the elevator.
     * @param maxHeight   The maximum height of the elevator.
     * @param startHeight The starting height of the elevator.
     * 
     * @see ElevatorSim
     */
    protected void elevator44(double mass, double gearRatio, double drumRadius, double minHeight, double maxHeight,
            double startHeight) {
        this.gearRatio = gearRatio;
        this.drumRadius = drumRadius;
        DCMotor dcMotor = DCMotor.getKrakenX44(motors.length);
        LinearSystem<N2, N1, N2> system = Models.elevatorFromPhysicalConstants(dcMotor, mass, drumRadius, gearRatio);
        this.elevatorSim = new ElevatorSim(system, dcMotor, minHeight, maxHeight, true, startHeight);
    }

    /**
     * Constructs a {@link FlywheelSim} for a CTRE Kraken X44 flywheel mechanism.
     * 
     * @param gearRatio The gear ratio of the flywheel.
     * @param J         The inertia of the flywheel.
     * 
     * @see FlywheelSim
     */
    protected void flywheel60(FlywheelSim flywheelSim, double gearRatio, double J) {
        this.gearRatio = gearRatio;
        DCMotor motor = DCMotor.getKrakenX60(motors.length);
        LinearSystem<N1, N1, N1> system = Models.flywheelFromPhysicalConstants(motor, J, gearRatio);
        this.flywheelSim = new FlywheelSim(system, motor);
    }

    /**
     * Constructs a {@link SingleJointedArmSim} for a CTRE Kraken X44 arm mechanism.
     * 
     * @param gearRatio    The gear ratio of the arm.
     * @param armLength    The length of the arm.
     * @param minAngleRads The minimum rotations of the arm (in radians).
     * @param maxAngleRads The maximum rotations of the arm (in radians).
     * @param startRads    The starting rotations of the arm (in radians).
     * @param J            The inertia of the arm.
     * 
     * @see SingleJointedArmSim
     */
    protected void arm60(double gearRatio, double armLength, double minAngleRads,
            double maxAngleRads, double startRads, double J) {
        this.gearRatio = gearRatio;
        DCMotor dcMotor = DCMotor.getKrakenX60(motors.length);
        LinearSystem<N2, N1, N2> system = Models
                .singleJointedArmFromPhysicalConstants(dcMotor, J, gearRatio);
        this.armSim = new SingleJointedArmSim(system, dcMotor, gearRatio, armLength, minAngleRads, maxAngleRads, true,
                startRads);
    }

    /**
     * Constructs a {@link ElevatorSim} for a CTRE kraken X44 elevator mechanism.
     * 
     * @param mass        The elevator carriage mass.
     * @param gearRatio   The gear ratio of the elevator.
     * @param drumRadius  The radius of the elevator drum.
     * @param minHeight   The minimum height of the elevator.
     * @param maxHeight   The maximum height of the elevator.
     * @param startHeight The starting height of the elevator.
     * 
     * @see ElevatorSim
     */
    protected void elevator60(double mass, double gearRatio, double drumRadius, double minHeight, double maxHeight,
            double startHeight) {
        this.gearRatio = gearRatio;
        this.drumRadius = drumRadius;
        DCMotor dcMotor = DCMotor.getKrakenX60(motors.length);
        LinearSystem<N2, N1, N2> system = Models.elevatorFromPhysicalConstants(dcMotor, mass, drumRadius, gearRatio);
        this.elevatorSim = new ElevatorSim(system, dcMotor, minHeight, maxHeight, true, startHeight);
    }

    /**
     * Periodic method for CTRE kraken X44 and X60 flywheel mechanisms. This must be
     * called manually.
     */
    public void flywheelPeriodic() {
        flywheelSim.setInputVoltage(motors[0].getSimState().getMotorVoltage());
        flywheelSim.update(0.02);
        for (TalonFX motor : motors) {
            motor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
            motor.getSimState().setRotorVelocity(
                    (RadiansPerSecond.of(flywheelSim.getAngularVelocity()).in(RotationsPerSecond) * gearRatio));
            motor.getSimState().addRotorPosition(
                    (RadiansPerSecond.of(flywheelSim.getAngularVelocity()).in(RotationsPerSecond) * gearRatio) * 0.02);
        }
    }

    /**
     * Periodic method for CTRE kraken X44 and X60 arm mechanisms. This must be
     * called manually.
     */
    public void armPeriodic() {
        armSim.setInputVoltage(motors[0].getSimState().getMotorVoltage());
        armSim.update(0.02);
        for (TalonFX motor : motors) {
            motor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
            motor.getSimState().setRawRotorPosition((armSim.getAngle() / (2 * Math.PI)) * gearRatio);
            motor.getSimState().setRotorVelocity((armSim.getVelocity() / (2 * Math.PI)) * gearRatio);
        }
    }

    /**
     * Periodic method for CTRE kraken X44 and X60 elevator mechanisms. This must be
     * called manually.
     */
    public void elevatorPeriodic() {
        elevatorSim.setInputVoltage(motors[0].getSimState().getMotorVoltage());
        elevatorSim.update(0.02);
        for (TalonFX motor : motors) {
            motor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
            motor.getSimState()
                    .setRawRotorPosition((elevatorSim.getPosition() / (2 * Math.PI * drumRadius)) * gearRatio);
            motor.getSimState().setRotorVelocity((elevatorSim.getVelocity() / (2 * Math.PI * drumRadius)) * gearRatio);
        }
    }
}

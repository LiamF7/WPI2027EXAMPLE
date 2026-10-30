package first.bearlib;

import static org.wpilib.units.Units.Volts;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

/**
 * Simulation model of CTRE mechanism, based of mechanism and motor type.
 * Utilizes WPILib's {@link FlywheelSim} or {@link DCMotorSim}.
 */
public class BearSim {

    private final FlywheelSim flywheelSim;
    private final ElevatorSim elevatorSim;
    private final SingleJointedArmSim armSim;

    private final BearSimValue bearSimValue;
    private final TalonFX[] motors;
    private double drumRadius = 0.0;
    private double gearRatio = 0.0;

    /**
     * The motor and mechanism type referenced when constructing a {@link BearSim}.
     */
    public enum BearSimValue {
        FLYWHEEL_X44,
        ELEVATOR_X44,
        ARM_X44,
        FLYWHEEL_X60,
        ELEVATOR_X60,
        ARM_X60,
    }

    /**
     * Simulation model of CTRE mechanism, based of mechanism and motor type.
     * Utilizes WPILib's {@link FlywheelSim} or {@link DCMotorSim}. This constructor
     * will be used for a flywheel sim!
     * 
     * @param bearSimValue The mechanism and motor type.
     * @param inertia      The inertia of the system.
     * @param gearRatio    The gear ratio of the system.
     * @param numMotors    The number of motors in the system
     * @param motors       The motors involved involved in the system.
     */
    public BearSim(BearSimValue bearSimValue, double inertia, double gearRatio, int numMotors, TalonFX... motors) {
        this.bearSimValue = bearSimValue;
        this.motors = motors;

        this.flywheelSim = setupFlywheelSim(inertia, gearRatio, numMotors);
        this.elevatorSim = null;
        this.armSim = null;
        this.gearRatio = gearRatio;

    }

    /**
     * Simulation model of CTRE mechanism, based of mechanism and motor type.
     * Utilizes WPILib's {@link FlywheelSim} or {@link DCMotorSim}. This constructor
     * will be used for a flywheel sim!
     * 
     * @param bearSimValue   The mechanism and motor type.
     * @param gearRatio      The elevator gear ratio.
     * @param numMotors      The number of motors in the system.
     * @param mass           The mass of the elevator.
     * @param drumRadius     The radius of the elevator drum.
     * @param minHeight      The min height of the elevator.
     * @param maxHeight      The max height of the elevator.
     * @param startingHeight The starting height of the elevator.
     * @param motors         The motors involved involved in the system.
     */
    public BearSim(BearSimValue bearSimValue, double gearRatio, int numMotors, double mass, double drumRadius,
            double minHeight, double maxHeight, double startingHeight, TalonFX... motors) {
        this.bearSimValue = bearSimValue;
        this.motors = motors;

        this.flywheelSim = null;
        this.elevatorSim = setupElevatorSim(gearRatio, numMotors, mass, drumRadius, minHeight, maxHeight,
                startingHeight);
        this.armSim = null;
        this.gearRatio = gearRatio;
        this.drumRadius = drumRadius;
    }

    /**
     * Simulation model of CTRE mechanism, based of mechanism and motor type.
     * Utilizes WPILib's {@link FlywheelSim} or {@link DCMotorSim}. This constructor
     * will be used for a flywheel sim!
     * 
     * @param inertia   The moment of inertia of the system.
     * @param gearRatio The gear ratio of the system.
     * @param numMotors The number of motors in the system
     * @param armLength The length of the arm.
     * @param minRads   The minimum arm rotations (in rads).
     * @param maxRads   The maximum arm rotations (in rads).
     * @param startRads The start/initial rotations (in rads).
     * @param motors    The motors involved involved in the system.
     */
    public BearSim(BearSimValue bearSimValue, double inertia, double gearRatio, int numMotors, double armLength,
            double minRads, double maxRads, double startRads, TalonFX... motors) {
        this.bearSimValue = bearSimValue;
        this.motors = motors;

        this.flywheelSim = null;
        this.elevatorSim = null;
        this.armSim = setupArmSim(inertia, gearRatio, numMotors, armLength, minRads, maxRads, startRads);
        this.gearRatio = gearRatio;
    }

    /**
     * Constructor for {@link FlywheelSim} dependent on {@link BearSimValue}.
     * 
     * @param inertia   The flywheel inertia.
     * @param gearRatio The flywheel gear ratio.
     * @param numMotors The number of motors in system.
     * @return {@link FlywheelSim}
     */
    private FlywheelSim setupFlywheelSim(double inertia, double gearRatio, int numMotors) {
        FlywheelSim simModel;
        switch (bearSimValue) {
            case FLYWHEEL_X44:
                simModel = new FlywheelSim(
                        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX44(numMotors), gearRatio, inertia),
                        DCMotor.getKrakenX44(numMotors));
                break;
            case FLYWHEEL_X60:
                simModel = new FlywheelSim(
                        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(numMotors), gearRatio, inertia),
                        DCMotor.getKrakenX60(numMotors));
                break;
            default:
                simModel = null;
        }
        return simModel;
    }

    /**
     * Constructor for {@link ElevatorSim} dependent on {@link BearSimValue}.
     * 
     * @param gearRatio      The elevator gear ratio.
     * @param numMotors      The number of motors in the system.
     * @param mass           The mass of the elevator.
     * @param drumRadius     The radius of the elevator drum.
     * @param minHeight      The min height of the elevator.
     * @param maxHeight      The max height of the elevator.
     * @param startingHeight The starting height of the elevator.
     * @return {@link ElevatorSim}
     */
    private ElevatorSim setupElevatorSim(double gearRatio, int numMotors, double mass, double drumRadius,
            double minHeight, double maxHeight, double startingHeight) {
        ElevatorSim simModel;
        switch (bearSimValue) {
            case ELEVATOR_X44:
                simModel = new ElevatorSim(
                        Models.elevatorFromPhysicalConstants(DCMotor.getKrakenX44(numMotors), mass, drumRadius,
                                gearRatio),
                        DCMotor.getKrakenX44(numMotors), minHeight, maxHeight, true, startingHeight);
                break;
            case ELEVATOR_X60:
                simModel = new ElevatorSim(
                        Models.elevatorFromPhysicalConstants(DCMotor.getKrakenX60(numMotors), mass, drumRadius,
                                gearRatio),
                        DCMotor.getKrakenX60(numMotors), minHeight, maxHeight, true, startingHeight);
                break;
            default:
                simModel = null;
                break;
        }
        return simModel;
    }

    /**
     * Constructor for {@link SingleJointedArmSim} dependent on
     * {@link BearSimValue.}
     * 
     * @param inertia   The moment of inertia of the system.
     * @param gearRatio The gear ratio of the system.
     * @param numMotors The number of motors in the system
     * @param armLength The length of the arm.
     * @param minRads   The minimum arm rotations (in rads).
     * @param maxRads   The maximum arm rotations (in rads).
     * @param startRads The start/initial rotations (in rads).
     * @return {@link SingleJointedArmSim}.
     */
    private SingleJointedArmSim setupArmSim(double inertia, double gearRatio, int numMotors, double armLength,
            double minRads, double maxRads, double startRads) {
        SingleJointedArmSim simModel;
        switch (bearSimValue) {
            case ARM_X44:
                simModel = new SingleJointedArmSim(DCMotor.getKrakenX44(numMotors), gearRatio, inertia, armLength,
                        minRads, maxRads, true, startRads);
                break;
            case ARM_X60:
                simModel = new SingleJointedArmSim(DCMotor.getKrakenX60(numMotors), gearRatio, inertia, armLength,
                        minRads, maxRads, true, startRads);
                break;
            default:
                simModel = null;
                break;
        }
        return simModel;
    }

    /**
     * Periodic, only call for sim.
     */
    public void periodic() {
    double volts = motors[0].getSimState().getMotorVoltageMeasure().in(Volts);

    double mechPosRots = 0;
    double mechVelRps = 0;
    boolean hasPosition = true;

    switch (bearSimValue) {
        case FLYWHEEL_X44:
        case FLYWHEEL_X60:
            flywheelSim.setInputVoltage(volts);
            flywheelSim.update(0.02);
            mechVelRps = flywheelSim.getAngularVelocity() / (2 * Math.PI);
            hasPosition = false;
            break;

        case ELEVATOR_X44:
        case ELEVATOR_X60:
            elevatorSim.setInputVoltage(volts);
            elevatorSim.update(0.02);
            mechPosRots = elevatorSim.getPosition() / (2 * Math.PI * drumRadius);
            mechVelRps = elevatorSim.getVelocity() / (2 * Math.PI * drumRadius);
            break;

        case ARM_X44:
        case ARM_X60:
            armSim.setInputVoltage(volts);
            armSim.update(0.02);
            mechPosRots = armSim.getAngle() / (2 * Math.PI);
            mechVelRps = armSim.getVelocity() / (2 * Math.PI);
            break;
    }

    for (TalonFX motor : motors) {
        TalonFXSimState simState = motor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        if (hasPosition) {
            simState.setRawRotorPosition(mechPosRots * gearRatio);
        }
        simState.setRotorVelocity(mechVelRps * gearRatio);
    }
}
}

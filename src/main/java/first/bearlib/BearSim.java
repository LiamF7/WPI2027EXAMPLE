package first.bearlib;

import static org.wpilib.units.Units.Volts;

import org.wpilib.math.numbers.N1;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
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

    /**
     * WPILIB FlywheelSim, in case of non-flywheel {@link BearSimValue}, will be set
     * to null, and all calls will return exception. Currently TalonFXSimSupport is
     * not optimal..
     */
    private final FlywheelSim flywheelSim;
    /**
     * WPILIB ElevatorSim, in case of non-elevator {@link BearSimValue}, will be set
     * to null, and all calls will return exception.
     */
    private final ElevatorSim elevatorSim;
    /**
     * WPILIB SignleJointedArmSim, in case of non-arm {@link BearSimValue}, will be
     * set
     * to null, and all calls will return exception.
     */
    private final SingleJointedArmSim armSim;

    /**
     * The {@link BearSimValue} defined in the constructor, that determines the sim
     * model, as well as the porper parameters to be paseed in.
     */
    private final BearSimValue bearSimValue;

    /**
     * A list of all the CTRE motors used in the BearSim system.
     */
    private final TalonFX[] motors;

    /**
     * Used in elevator sim, initialized to zero.
     */
    private double drumRadius = 0.0;

    /**
     * Used in all mechanisms, initialized to zero.
     */
    private double gearRatio = 0.0;

    /**
     * The motor and mechanism type referenced when constructing a {@link BearSim}.
     */
    public enum BearSimValue {
        /**
         * Will use in constructor:
         * 
         * @param bearSimValue
         * @param inertia
         * @param gearRatio
         * @param numMotors
         */
        FLYWHEEL_X44,
        /**
         * Will use in constructor:
         * 
         * @param bearSimValue   The mechanism and motor type.
         * @param gearRatio      The elevator gear ratio.
         * @param mass           The mass of the elevator.
         * @param drumRadius     The radius of the elevator drum.
         * @param minHeight      The min height of the elevator.
         * @param maxHeight      The max height of the elevator.
         * @param startingHeight The starting height of the elevator.
         * @param motors         The motors involved involved in the system.
         */
        ELEVATOR_X44,
        /**
         * Will use in constructor:
         *
         * @param bearSimValue the {@link BearSimValue} for the mechanism.
         * @param inertia      The moment of inertia of the system.
         * @param gearRatio    The gear ratio of the system.
         * @param numMotors    The number of motors in the system
         * @param armLength    The length of the arm.
         * @param minRads      The minimum arm rotations (in rads).
         * @param maxRads      The maximum arm rotations (in rads).
         * @param startRads    The start/initial rotations (in rads).
         * @param motors       The motors involved involved in the system.
         */
        ARM_X44,
        /**
         * Will use in constructor:
         * 
         * @param bearSimValue
         * @param inertia
         * @param gearRatio
         * @param numMotors
         */
        FLYWHEEL_X60,
        /**
         * Will use in constructor:
         * 
         * @param bearSimValue   The mechanism and motor type.
         * @param gearRatio      The elevator gear ratio.
         * @param mass           The mass of the elevator.
         * @param drumRadius     The radius of the elevator drum.
         * @param minHeight      The min height of the elevator.
         * @param maxHeight      The max height of the elevator.
         * @param startingHeight The starting height of the elevator.
         * @param motors         The motors involved involved in the system.
         */
        ELEVATOR_X60,
        /**
         * Will use in constructor:
         * 
         * @param bearSimValue the {@link BearSimValue} for the mechanism.
         * @param inertia      The moment of inertia of the system.
         * @param gearRatio    The gear ratio of the system.
         * @param numMotors    The number of motors in the system
         * @param armLength    The length of the arm.
         * @param minRads      The minimum arm rotations (in rads).
         * @param maxRads      The maximum arm rotations (in rads).
         * @param startRads    The start/initial rotations (in rads).
         * @param motors       The motors involved involved in the system.
         */
        ARM_X60,
    }

    /**
     * Simulation model of CTRE mechanism, based of mechanism and motor type.
     * Utilizes WPILib's {@link FlywheelSim} or {@link DCMotorSim}. This constructor
     * will be used for a elevator sim!
     * 
     * @param bearSimValue   The mechanism and motor type.
     * @param gearRatio      The elevator gear ratio.
     * @param mass           The mass of the elevator.
     * @param drumRadius     The radius of the elevator drum.
     * @param minHeight      The min height of the elevator.
     * @param maxHeight      The max height of the elevator.
     * @param startingHeight The starting height of the elevator.
     * @param motors         The motors involved involved in the system.
     */
    public BearSim(BearSimValue bearSimValue, double gearRatio, double mass, double drumRadius,
            double minHeight, double maxHeight, double startingHeight, TalonFX... motors) {
        this.bearSimValue = bearSimValue;
        this.motors = motors;

        this.flywheelSim = null;
        this.elevatorSim = setupElevatorSim(gearRatio, motors.length, mass, drumRadius, minHeight, maxHeight,
                startingHeight);
        this.armSim = null;
        this.gearRatio = gearRatio;
        this.drumRadius = drumRadius;
    }

    /**
     * Simulation model of CTRE mechanism, based of mechanism and motor type.
     * Utilizes WPILib's {@link FlywheelSim} or {@link DCMotorSim}. This constructor
     * will be used for an arm sim!
     * 
     * @param bearSimValue the {@link BearSimValue} for the mechanism.
     * @param inertia      The moment of inertia of the system.
     * @param gearRatio    The gear ratio of the system.
     * @param numMotors    The number of motors in the system
     * @param armLength    The length of the arm.
     * @param minRads      The minimum arm rotations (in rads).
     * @param maxRads      The maximum arm rotations (in rads).
     * @param startRads    The start/initial rotations (in rads).
     * @param motors       The motors involved involved in the system.
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
     * Simulation model of CTRE mechanism, based of mechanism and motor type.
     * Utilizes WPILib's {@link FlywheelSim} or {@link DCMotorSim}. This constructor
     * will be used for a flywheel sim!
     * 
     * @param bearSimValue
     * @param inertia
     * @param gearRatio
     * @param numMotors
     */
    public BearSim(BearSimValue bearSimValue, double inertia, double gearRatio, int numMotors, TalonFX... motors) {
        this.bearSimValue = bearSimValue;
        this.gearRatio = gearRatio;
        this.motors = motors;

        this.flywheelSim = setupFlywheelSim(inertia, gearRatio, numMotors);
        this.elevatorSim = null;
        this.armSim = null;
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
     * {@link BearSimValue}.
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
     * Constructor for {@link FlywheelSim} dependent on {@link BearSimValue}.
     * 
     * @param inertia
     * @param gearRatio
     * @param numMotors
     * @return {@link FlywheelSim}
     */
    private FlywheelSim setupFlywheelSim(double inertia, double gearRatio, int numMotors) {
        FlywheelSim simModel;
        switch (bearSimValue) {
            case FLYWHEEL_X44:
                LinearSystem<N1, N1, N1> x44 = Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX44(numMotors),
                        inertia, gearRatio);
                simModel = new FlywheelSim(x44, DCMotor.getKrakenX44(numMotors));
                break;
            case FLYWHEEL_X60:
                LinearSystem<N1, N1, N1> x60 = Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX44(numMotors),
                        inertia, gearRatio);
                simModel = new FlywheelSim(x60, DCMotor.getKrakenX44(numMotors));
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
        boolean isFlywheel = false;

        switch (bearSimValue) {
            case FLYWHEEL_X44:
            case FLYWHEEL_X60:
                isFlywheel = true;
                flywheelSim.setInputVoltage(volts);
                flywheelSim.update(0.02);
                mechVelRps = flywheelSim.getAngularVelocity() / (2.0 * Math.PI) * gearRatio;
                break;
            case ELEVATOR_X44:
            case ELEVATOR_X60:
                isFlywheel = false;
                elevatorSim.setInputVoltage(volts);
                elevatorSim.update(0.02);
                mechPosRots = elevatorSim.getPosition() / (2 * Math.PI * drumRadius);
                mechVelRps = elevatorSim.getVelocity() / (2 * Math.PI * drumRadius);
                break;

            case ARM_X44:
            case ARM_X60:
                isFlywheel = false;
                armSim.setInputVoltage(volts);
                armSim.update(0.02);
                mechPosRots = armSim.getAngle() / (2 * Math.PI);
                mechVelRps = armSim.getVelocity() / (2 * Math.PI);
                break;
        }

        for (TalonFX motor : motors) {
            TalonFXSimState simState = motor.getSimState();
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());
            simState.setRotorVelocity(mechVelRps * gearRatio);
            if (isFlywheel) {
                simState.addRotorPosition(mechVelRps * 0.02);
                flywheelSim.setInput(volts);
            }
            simState.setRawRotorPosition(mechPosRots * gearRatio);
        }
    }
}

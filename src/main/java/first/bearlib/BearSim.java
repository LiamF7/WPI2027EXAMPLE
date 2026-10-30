package first.bearlib;

import static org.wpilib.units.Units.Volts;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

/**
 * Simulation model of CTRE mechanism, based of mechanism and motor type.
 * Utilizes WPILib's {@link FlywheelSim} or {@link DCMotorSim}.
 */
public class BearSim {

    private FlywheelSim flywheelSim;
    private DCMotorSim dcMotorSim;
    private final TalonFX[] motors;
    private final BearSimValue bearSimValue;
    private final double inertia;
    private final double gearRatio;
    private final int numMotors;
    private final double mass;
    private final double drumRadius;

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
     * Utilizes WPILib's {@link FlywheelSim} or {@link DCMotorSim}.
     * 
     * @param bearSimValue The mechanism and motor type.
     * @param inertia      The inertia of the system.
     * @param gearRatio    The gear ratio of the system.
     * @param numMotors    The number of motors in the system
     * @param motors       The TalonFX motor(s) involved.
     */
    public BearSim(BearSimValue bearSimValue, double inertia, double gearRatio, int numMotors, TalonFX... motors) {
        this.motors = motors;
        this.bearSimValue = bearSimValue;
        this.inertia = inertia;
        this.gearRatio = gearRatio;
        this.numMotors = numMotors;
        this.mass = 0.1;
        this.drumRadius = 0.1;
        setupMotorSim();
    }

    /**
     * Simulation model of CTRE mechanism, based of mechanism and motor type.
     * Utilizes WPILib's {@link FlywheelSim} or {@link DCMotorSim}.
     * 
     * @param bearSimValue The mechanism and motor type.
     * @param inertia      The inertia of the system.
     * @param gearRatio    The gear ratio of the system.
     * @param numMotors    The number of motors in the system.
     * @param mass         (For elevators only) the mass of the carriage.
     * @param drumRadius   (For elevators only) the drum radius.
     * @param motors       The TalonFX motor(s) involved.
     */
    public BearSim(BearSimValue bearSimValue, double inertia, double gearRatio, int numMotors,
            double mass, double drumRadius, TalonFX... motor) {
        this.motors = motor;
        this.bearSimValue = bearSimValue;
        this.inertia = inertia;
        this.gearRatio = gearRatio;
        this.numMotors = numMotors;
        this.mass = mass;
        this.drumRadius = drumRadius;
        setupMotorSim();
    }

    /**
     * Sets up motorSim models based of corresponding {@link BearSimValue}.
     */
    private void setupMotorSim() {
        switch (bearSimValue) {
            case FLYWHEEL_X44:
                flywheelSim = new FlywheelSim(
                        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX44(numMotors), inertia, gearRatio),
                        DCMotor.getKrakenX44(numMotors));
                        break;
            case ELEVATOR_X44:
                dcMotorSim = new DCMotorSim(Models.elevatorFromPhysicalConstants(DCMotor.getKrakenX44(numMotors), mass,
                        drumRadius, gearRatio), DCMotor.getKrakenX44(numMotors));
                        break;
            case ARM_X44:
                dcMotorSim = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(
                        DCMotor.getKrakenX44(numMotors), inertia, gearRatio), DCMotor.getKrakenX44(numMotors));
                        break;
            case FLYWHEEL_X60:
                flywheelSim = new FlywheelSim(
                        Models.flywheelFromPhysicalConstants(DCMotor.getKrakenX60(numMotors), inertia, gearRatio),
                        DCMotor.getKrakenX44(numMotors));
                        break;
            case ELEVATOR_X60:
                dcMotorSim = new DCMotorSim(Models.elevatorFromPhysicalConstants(DCMotor.getKrakenX60(numMotors), mass,
                        drumRadius, gearRatio), DCMotor.getKrakenX44(numMotors));
                        break;
            case ARM_X60:
                dcMotorSim = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(
                        DCMotor.getKrakenX60(numMotors), inertia, gearRatio), DCMotor.getKrakenX44(numMotors));
                        break;
        }
    }

    /**
     * Run this as periodic during simulation!
     */
    public void periodic() {
        for (TalonFX m : this.motors) {

            TalonFXSimState talonFXSim = m.getSimState();

            talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

            Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();

            // update proper sim model based on BearMechanismValue
            if (bearSimValue.equals(BearSimValue.FLYWHEEL_X44) || bearSimValue.equals(BearSimValue.FLYWHEEL_X60)) {
                flywheelSim.setInputVoltage(motorVoltage.in(Volts));
                flywheelSim.update(0.02);
                talonFXSim.setRotorVelocity(flywheelSim.getAngularVelocity() * gearRatio);
                return;
            }
            dcMotorSim.setInputVoltage(motorVoltage.in(Volts));
            dcMotorSim.update(0.02);
            talonFXSim.setRawRotorPosition(dcMotorSim.getAngularPosition() * gearRatio);
        }
    }
}

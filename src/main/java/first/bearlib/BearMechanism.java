package first.bearlib;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.DriverStationDisplay;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import first.bearlib.util.PhoenixUtil;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

/**
 * Team 4068's class for CTRE Kraken X44 or X60 motor mechanism. Offers clean
 * motor configuration, unified telemetry, fault logging, control mode factories,
 * simulstion support, self test support.
 */
public class BearMechanism implements Mechanism{

  private int periodicCount = 0; // used for periodic fault logging

  private boolean lastHasFault = false;

  protected TalonFX motor;

  protected CANBus canivore;

  protected TalonFXConfiguration motorConfig;

  private SelfTestableType selfTestableType;
  private double outputFast;
  private double outputSlow;
  private boolean selfTestStatus;
  private double selfTestTimeLimit;
  private boolean selfTestPassed;

  protected final StatusSignal<Current> motorSupplyCurrent;
  protected final StatusSignal<Current> motorStatorCurrent;
  protected final StatusSignal<AngularVelocity> motorVelocity;
  protected final StatusSignal<Temperature> motorTemperature;
  protected final StatusSignal<Double> motorClosedLoopError;
  protected final StatusSignal<Voltage> motorVoltage;
  protected final StatusSignal<Angle> motorPosition;
  protected final StatusSignal<Double> setpoint;
  protected final StatusSignal<Double> motorProfileVelocity;

  // Open loop
  private final DutyCycleOut dc = new DutyCycleOut(0.0);
  private final VoltageOut vo = new VoltageOut(0.0);
  private final TorqueCurrentFOC tcFOC = new TorqueCurrentFOC(0.0);

  // Velocity
  private final VelocityVoltage vv = new VelocityVoltage(0.0);
  private VelocityTorqueCurrentFOC vtcFOC = new VelocityTorqueCurrentFOC(0.0);

  // Position
  private final PositionVoltage pv = new PositionVoltage(0.0);
  private final PositionTorqueCurrentFOC ptcFOC = new PositionTorqueCurrentFOC(0.0);

  // Motion Magic
  private final MotionMagicVoltage mmv = new MotionMagicVoltage(0.0);
  private final MotionMagicTorqueCurrentFOC mmtcFOC = new MotionMagicTorqueCurrentFOC(0.0);
  private final MotionMagicVelocityVoltage mmvv = new MotionMagicVelocityVoltage(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC mmvtcFOC = new MotionMagicVelocityTorqueCurrentFOC(0.0);

  // Neutral
  private final NeutralOut neutral = new NeutralOut();
  private final StaticBrake brake = new StaticBrake();
  private final CoastOut coast = new CoastOut();

  /**
   * super constructor.
   *
   * @param name     name of the mechanism.
   * @param ID       The CAN ID.
   * @param canivore The CANBus.
   */
  public BearMechanism(String name, int ID, CANBus canivore) {

    this.canivore = canivore;

    motor = new TalonFX(ID, canivore);

    motorConfig = new TalonFXConfiguration();

    Telemetry.getTable().log(name + " READY", this.isReady());

    motorSupplyCurrent = motor.getSupplyCurrent(false);
    motorStatorCurrent = motor.getStatorCurrent(false);
    motorVelocity = motor.getVelocity(false);
    motorTemperature = motor.getDeviceTemp(false);
    motorClosedLoopError = motor.getClosedLoopError(false);
    motorPosition = motor.getPosition(false);
    motorProfileVelocity = motor.getClosedLoopReferenceSlope(false);
    motorVoltage = motor.getMotorVoltage(false);
    setpoint = motor.getClosedLoopReference(false);

    DriverStationDisplay.addLine(name + " INITIALIZED!");

    // 2027 periodic call
    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  /** Refresh status signals, log faults. */
  public void periodic() {
    /* refresh all status signals */
    BaseStatusSignal.refreshAll(
        motorPosition,
        motorVelocity,
        motorStatorCurrent,
        motorSupplyCurrent,
        motorVoltage,
        motorTemperature,
        setpoint);
    logFaults(motor);
  }

  /**
   * Updates the simulation for the mechanism.
   *
   * @param motor         The TalonFX motor controller being simulated.
   * @param gearRatio     The gear ratio of the mechanism.
   * @param motorSimModel The physics simulation model associated with this
   *                      mechanism.
   */
  public void simulationPeriodic(TalonFX motor, double gearRatio, DCMotorSim motorSimModel) {
    var talonFXSim = motor.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition() * gearRatio);
    talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity() * gearRatio);
  }

  /**
   * Logs sticky and hardware faults,
   *
   * @param motor The TalonFX motor controller instance to check for faults.
   */
  public void logFaults(TalonFX motor) {
    if (periodicCount++ % 12 != 0) { // every 240ms at 50Hz loop
      return;
    }

    boolean liveUndervoltage = motor.getFault_Undervoltage().getValue();
    boolean liveBootDuring = motor.getFault_BootDuringEnable().getValue();
    boolean liveDeviceTemp = motor.getFault_DeviceTemp().getValue();
    boolean liveHardware = motor.getFault_Hardware().getValue();
    boolean liveBridgeBrownout = motor.getFault_BridgeBrownout().getValue();

    boolean hasFault = liveUndervoltage || liveHardware || liveDeviceTemp || liveBootDuring || liveBridgeBrownout;

    // Report once on transition to faulted
    if (hasFault && !lastHasFault) {
      DriverStationErrors.reportError(getName() + "<<< HAS ERROR, CHECK LOGS >>> CAN " + canivore.getName(), false);
    }
    lastHasFault = hasFault;
  }

  /**
   * Sets and enables the stator current limit for the mechanism.
   *
   * @param value The stator current limit value.
   */
  public void statorCurrentLimit(double value) {
    motorConfig.CurrentLimits.StatorCurrentLimit = Amps.of(value).in(Amps);
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  }

  /**
   * Sets and enables the supply current for the mechanism.
   *
   * @param value The supply current limit value.
   */
  public void supplyCurrentLimit(double value) {
    motorConfig.CurrentLimits.SupplyCurrentLimit = Amps.of(value).in(Amps);
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  /**
   * Sets the horizontal position of an arm/pivot.
   * 
   * @param position the horizontal position.
   */
  public void gravityArmPositionOffset(double position) {
    motorConfig.Slot0.GravityArmPositionOffset = position;
  }

  /**
   * Sets the sign of the static feedforward.
   * 
   * @param staticFeedForwardSignValue the feed forward sign
   */
  public void staticFeedForwardSign(StaticFeedforwardSignValue staticFeedForwardSignValue) {
    motorConfig.Slot0.StaticFeedforwardSign = staticFeedForwardSignValue;
  }

  /**
   * Sets the proportional gain of motor controller.
   *
   * @param p The propotional gain.
   */
  public void kP(double p) {
    motorConfig.Slot0.kP = p;
  }

  /**
   * Sets the proportional gain of motor controller for slot 1.
   *
   * @param p The proportional gain.
   */
  public void kP1(double p) {
    motorConfig.Slot1.kP = p;
  }

  /**
   * Sets the derivative gain of motor controller for slot 1.
   *
   * @param d The derivative gain.
   */
  public void kD1(double d) {
    motorConfig.Slot1.kD = d;
  }

  /**
   * Sets the acceleration feedforward gain of motor controller for slot 1.
   *
   * @param a The acceleration feedforward gain.
   */
  public void kA1(double a) {
    motorConfig.Slot1.kA = a;
  }

  /**
   * Sets the static feedforward gain of motor controller for slot 1.
   *
   * @param s The static feedforward gain.
   */
  public void kS1(double s) {
    motorConfig.Slot1.kS = s;
  }

  /**
   * Sets the velocity feedforward gain of motor controller for slot 1.
   *
   * @param v The velocity feedforward gain.
   */
  public void kV1(double v) {
    motorConfig.Slot1.kV = v;
  }

  /**
   * Sets the integral gain of motor controller.
   *
   * @param i The integral gain.
   */
  public void kI(double i) {
    motorConfig.Slot0.kI = i;
  }

  /**
   * Sets the derivative gain of motor controller.
   *
   * @param d The derivative gain.
   */
  public void kD(double d) {
    motorConfig.Slot0.kD = d;
  }

  /**
   * Sets the static feedforward gain of motor controller.
   *
   * @param s The static feedforward gain.
   */
  public void kS(double s) {
    motorConfig.Slot0.kS = s;
  }

  /**
   * Sets the gravity feedback/forward gain of motor controller.
   *
   * @param g The gravity feedback/forward gain.
   */
  public void kG(double g) {
    motorConfig.Slot0.kG = g;
  }

  /**
   * Sets the acceleration feedforward gain of motor controller.
   *
   * @param a The acceleration feedforward gain.
   */
  public void kA(double a) {
    motorConfig.Slot0.kA = a;
  }

  /**
   * Sets the velocity feedforward gain of motor controller.
   *
   * @param v The velocity feedforward gain.
   */
  public void kV(double v) {
    motorConfig.Slot0.kV = v;
  }

  /**
   * Sets the sensor to rotor ratio.
   * 
   * @param rotorToSensorRatio the sensor to rotot ratio.
   */
  public void rotorToSensorRatio(double rotorToSensorRatio) {
    motorConfig.Feedback.RotorToSensorRatio = rotorToSensorRatio;
  }

  /**
   * Sets the neutral mode output for mechanism.
   *
   * @param neutralModeValue The neutral mode.
   */
  public void neutralMode(NeutralModeValue neutralModeValue) {
    motorConfig.MotorOutput.NeutralMode = neutralModeValue;
  }

  /**
   * Sets the inverted output for mechanism.
   *
   * @param invertedValue The inverted value.
   */
  public void inverted(InvertedValue invertedValue) {
    motorConfig.MotorOutput.Inverted = invertedValue;
  }

  /**
   * Sets the forward soft limit.
   *
   * @param softLimit The value of the soft limit.
   */
  public void forwardSoftLimit(double softLimit) {
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softLimit;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  }

  /**
   * Sets the reverse soft limit.
   *
   * @param softLimit The value of the soft limit.
   */
  public void reverseSoftLimit(double softLimit) {
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softLimit;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  }

  /**
   * Sets the motion magic acceleration.
   *
   * @param acceleration MotionMagic acceleration value.
   */
  public void motionMagicAcceleration(double acceleration) {
    motorConfig.MotionMagic.MotionMagicAcceleration = acceleration;
  }

  /**
   * Sets the motion magic velocity.
   *
   * @param velocity MotionMagic velocity value.
   */
  public void motionMagicCruiseVelocity(double velocity) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = velocity;
  }

  /**
   * Sets the gravity type.
   *
   * @param gravityTypeValue The gravity value for the gravity type.
   */
  public void gravityType(GravityTypeValue gravityTypeValue) {
    motorConfig.Slot0.GravityType = gravityTypeValue;
  }

  /**
   * Sets the sensor to mechanism ratio.
   *
   * @param sensorToMechanismRatio The sensor to mechanism ratio.
   */
  public void sensorToMechanismRatio(double sensorToMechanismRatio) {
    motorConfig.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
  }

  /**
   * Sets the motion magic jerk.
   *
   * @param motionMagicJerk The motion magic jerk value.
   */
  public void motionMagicJerk(double motionMagicJerk) {
    motorConfig.MotionMagic.MotionMagicJerk = motionMagicJerk;
  }

  /**
   * Sets the maximum forward output for duty cycle.
   *
   * @param peakForwardDutyCycle The maximum forward output value.
   */
  public void peakForwardDutyCycle(double peakForwardDutyCycle) {
    motorConfig.MotorOutput.PeakForwardDutyCycle = peakForwardDutyCycle;
  }

  /**
   * Sets the maximum reverse output for duty cycle.
   *
   * @param peakReverseDutyCycle The maximum revese output value.
   */
  public void peakReverseDutyCycle(double peakReverseDutyCycle) {
    motorConfig.MotorOutput.PeakReverseDutyCycle = peakReverseDutyCycle;
  }

  /**
   * Sets the maximum forward output for torque control
   *
   * @param peakForwardTorqueCurrent The maximum forward output value.
   */
  public void peakForwardTorqueCurrent(double peakForwardTorqueCurrent) {
    motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = peakForwardTorqueCurrent;
  }

  /**
   * Sets the maximum reverse output for torque control.
   *
   * @param peakReverseTorqueCurrent The maxium reverse forward output value.
   */
  public void peakReverseTorqueCurrent(double peakReverseTorqueCurrent) {
    motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = peakReverseTorqueCurrent;
  }

  /** Applies config to motor. */
  public void addConfig() {
    PhoenixUtil.applyConfig(() -> motor.getConfigurator().apply(motorConfig), getName());
  }

  /**
   * Applies a prexisting config to the motor.
   * 
   * @param config The prexisting config.
   */
  public void cloneConfig(TalonFXConfiguration config) {
    PhoenixUtil.applyConfig(() -> motor.getConfigurator().apply(config), getName());
  }

  /**
   * The {@link TalonFXConfiguration} of the motor.
   */
  public TalonFXConfiguration getConfig() {
    return motorConfig;
  }

  /**
   * Logs closed loop error for mechanism.
   *
   * @return double, closedLoopError
   */
  public double getClosedLoopError() {
    return motorClosedLoopError.getValue();
  }

  /**
   * Logs motor velocity for mechansim in RPM.
   *
   * @return AngularVelocity, motorVelocity
   */
  public double getVelocity() {
    return motorVelocity.getValue().in(RPM);
  }

  /**
   * Logs motor supply current for mechanism in Amps.
   *
   * @return Current, motorSupplyCurrent
   */
  public double getSupplyCurrent() {
    return motorSupplyCurrent.getValue().in(Amps);
  }

  /**
   * Logs motor stator current for mechanism in Amps.
   *
   * @return Current, motorStatorCurrent
   */
  public double getStatorCurrent() {
    return motorStatorCurrent.getValue().in(Amps);
  }

  /**
   * Logs motor temperature for mechanism in Celsius.
   *
   * @return double, motorTemperature
   */
  public double getTemperature() {
    return motorTemperature.getValue().in(Celsius);
  }

  /**
   * Logs motor voltage for mechanism in Volts.
   *
   * @return Voltage, motorVoltage
   */
  public double getMotorVoltageMeasure() {
    return motorVoltage.getValue().in(Volts);
  }

  /**
   * Logs motor position as an angle for mechanism in Degrees.
   *
   * @return Angle, motorPosition
   */
  public double getAngle() {
    return motorPosition.getValue().in(Degrees);
  }

  /**
   * Logs setpoint in degrees for mechanism.
   *
   * @return double, setpoint
   */
  public double getSetpoint() {
    return setpoint.getValueAsDouble();
  }

  protected void optimizeCAN() {
    motorPosition.setUpdateFrequency(250);
    motorVelocity.setUpdateFrequency(250);
    motorSupplyCurrent.setUpdateFrequency(50);
    motorStatorCurrent.setUpdateFrequency(50);
    motorClosedLoopError.setUpdateFrequency(50);
    motorTemperature.setUpdateFrequency(4);
    motorProfileVelocity.setUpdateFrequency(50);

    motor.optimizeBusUtilization();
  }

  /* SIMULATION */

  /**
   * Initializes simulation for Kraken X44 motor.
   *
   * @param motor       The TalonFX motor controller being simulated.
   * @param gearRatio   The gear ratio of the mechanism (motor rotations per
   *                    mechanism rotation).
   * @param inertia     The moment of inertia of the mechanism load (kg*m^2).
   * @param orientation The mechanical orientation of the motor relative to the
   *                    chassis.
   * @return A {@link DCMotorSim} model representing te physical system.
   */
  public DCMotorSim simulationInitKrakenX44(
      TalonFX motor, double gearRatio, double inertia, ChassisReference orientation) {
    TalonFXSimState talonFXSim = motor.getSimState();

    talonFXSim.Orientation = orientation;
    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

    // TODO: setup motor sim model.

    DCMotorSim motorSimModel = new DCMotorSim(null, null, null);

    var simConfig = new TalonFXConfiguration();
    motor.getConfigurator().refresh(simConfig);
    simConfig.Slot0.kS = 0.0;
    motor.getConfigurator().apply(simConfig);

    return motorSimModel;
  }

  /**
   * Initializes simulation model for a Kraken X60.
   *
   * @param motor       The TalonFX motor controller being simulated.
   * @param gearRatio   The gear ratio of the mechanism (motor rotations per
   *                    mechanism rotation).
   * @param inertia     The moment of inertia of the mechanism load (kg*m^2).
   * @param orientation The mechanical orientation of the motor relative to the
   *                    chassis.
   * @return A {@link DCMotorSim} model representing the physical system.
   */
  public DCMotorSim simulationInitKrakenX60(
      TalonFX motor, double gearRatio, double inertia, ChassisReference orientation) {
    TalonFXSimState talonFXSim = motor.getSimState();

    talonFXSim.Orientation = orientation;
    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    // TODO: setup motor sim model.
    DCMotorSim motorSimModel = new DCMotorSim(null, null, null);

    var simConfig = new TalonFXConfiguration();
    motor.getConfigurator().refresh(simConfig);
    simConfig.Slot0.kS = 0.0;
    motor.getConfigurator().apply(simConfig);

    return motorSimModel;
  }

  /**
   * Signals that the mechanism is ready.
   */
  private boolean isReady() {
    return motor.isConnected();
  }

  /**
   * Sets the motor to run at a specified velocity with torque current FOC.

   *
   * @param output The velocity in rpm
   */
  public void setVTCFOC(double output) {
    motor.setControl(vtcFOC.withVelocity(RPM.of(output).in(RotationsPerSecond)));
  }

  /**
   * Sets the motor to run at a specified velocity.
   *
   * @param velocity The velocity in rpm
   */
  public void setVV(double velocity) {
    motor.setControl(vv.withVelocity(RPM.of(velocity).in(RotationsPerSecond)));
  }

  /**
   * Sets the motor to run at a specified velocity with Motion Magic.
   *
   * @param velocity The velocity in rpm
   */
  public void setMMVV(double velocity) {
    motor.setControl(mmvv.withVelocity(RPM.of(velocity).in(RotationsPerSecond)));
  }

  /**
   * Sets the motor to run at a specified velocity with Motion Magic and torque
   * current FOC.
   *
   * @param velocity The velocity in rpm
   */
  public void setMMVTCFOC(double velocity) {
    motor.setControl(mmvtcFOC.withVelocity(RPM.of(velocity).in(RotationsPerSecond)));
  }

  /**
   * Sets the motor to run at a specified duty cycle.
   *
   * @param output The duty cycle from -1.0 to 1.0
   */
  public void setDC(double output) {
    motor.setControl(dc.withOutput(output));
  }

  /**
   * Sets the motor to run at a specified voltage.
   *
   * @param volts The voltage in volts
   */
  public void setVO(double volts) {
    motor.setControl(vo.withOutput(volts));
  }

  /**
   * Sets the motor to run at a specified torque current.
   *
   * @param amps The current in amps
   */
  public void setTCFOC(double amps) {
    motor.setControl(tcFOC.withOutput(amps));
  }

  /**
   * Sets the motor to move to a specified position.
   *
   * @param position The position in rotations
   */
  public void setPV(double position) {
    motor.setControl(pv.withPosition(position));
  }

  /**
   * Sets the motor to move to a specified position with torque current FOC.
   *
   * @param position The position in rotations
   */
  public void setPTCFOC(double position) {
    motor.setControl(ptcFOC.withPosition(position));
  }

  /**
   * Sets the motor to move to a specified position with Motion Magic.
   *
   * @param position The position in rotations
   */
  public void setMMV(double position) {
    motor.setControl(mmv.withPosition(position));
  }

  /**
   * Sets the motor to move to a specified position with Motion Magic and torque
   * current FOC.
   *
   * @param position The position in rotations
   */
  public void setMMTCFOC(double position) {
    motor.setControl(mmtcFOC.withPosition(position));
  }

  /**
   * Sets the motor to neutral output.
   */
  public void setStop() {
    motor.setControl(neutral);
  }

  /**
   * Sets the motor to brake.
   */
  public void setBrake() {
    motor.setControl(brake);
  }

  /**
   * Sets the motor to coast.
   */
  public void setCoast() {
    motor.setControl(coast);
  }

  /**
   * The control type used for the self test, typically the same as the one used
   * for mechanism commands.
   */
  public enum SelfTestableType {
    VELOCITY_TORQUE,
    POSITION_TORQUE,
    DUTY_CYCLE,
    VOLTAGE,
    TORQUE,
    VELOCITY_VOLTAGE,
    POSITION_VOLTAGE,
    MOTION_MAGIC_VOLTAGE,
    MOTION_MAGIC_TORQUE,
    MOTION_MAGIC_VELOCITY_VOLTAGE,
    MOTION_MAGIC_VELOCITY_TORQUE,
  }

  /**
   * Returns the runnable corresponding to the {@link SelfTestableType}, and sets
   * an output for it.
   * 
   * @param output The output value.
   * @return {@link Runnablr}
   */
  private void getSelfTest(double output) {
    switch (selfTestableType) {
      case VELOCITY_TORQUE -> setVTCFOC(output);
      case POSITION_TORQUE -> setPTCFOC(output);
      case DUTY_CYCLE -> setDC(output);
      case VOLTAGE -> setVO(output);
      case TORQUE -> setTCFOC(output);
      case VELOCITY_VOLTAGE -> setVV(output);
      case POSITION_VOLTAGE -> setPV(output);
      case MOTION_MAGIC_VOLTAGE -> setMMV(output);
      case MOTION_MAGIC_TORQUE -> setMMTCFOC(output);
      case MOTION_MAGIC_VELOCITY_VOLTAGE -> setMMVV(output);
      case MOTION_MAGIC_VELOCITY_TORQUE -> setMMVTCFOC(output);
    };
  }

  /**
   * Initializes {@link SelfTestableType}, aswell as the fast and slow outputs for
   * the self test. Assume same units as selfTestableType command for outputs.
   * 
   * @param selfTestableType  The {@link SelfTestableType}.
   * @param outputFast        The output fast.
   * @param outputSlow        The output slow.
   * @param selfTestStatus    The boolean that will bve truw ehen the self test is
   *                          passed.
   * @param selfTestTimeLimit the timeout for the self test command.
   */
  public void selfTestInit(SelfTestableType selfTestableType, double outputFast, double outputSlow,
      boolean selfTestStatus, double selfTestTimeLimit) {
    this.selfTestableType = selfTestableType;
    this.outputFast = outputFast;
    this.outputSlow = outputSlow;
    this.selfTestStatus = selfTestStatus;
    this.selfTestTimeLimit = selfTestTimeLimit;
  }

  /**
   * Generic command for self test at an output. Wraps completion and results of
   * self test into NT entry.
   * 
   * @param output The output of the self test
   * @param ntKey  The NT key to report status.
   * @return
   */
  public Command selfTestAt(double output, String ntKey) {
    return run(co -> {
      var nt = NetworkTableInstance.getDefault();
      nt.getEntry(ntKey + "/message").setString("Running...");
      nt.getEntry(ntKey + "/passed").unpublish();

     getSelfTest(output);
     co.wait(Seconds.of(selfTestTimeLimit));

      selfTestPassed = selfTestStatus;
      // TODO: add specifc results stuff to result
      String result = (selfTestPassed ? "PASS" : "FAIL");
      nt.getEntry(ntKey + "/passed").setBoolean(selfTestPassed);
      nt.getEntry(ntKey + "/message").setString(result);

    }).named(this.getName() + " Self test @" + output);
  }

  /**
   * Self Tests at normal speed.
   */
  public Command selfTestFast() {
    return selfTestAt(outputFast, "Robot/Tests/" + getName() + "/slow");
  }

  /**
   * Self Tests slowly.
   */
  public Command selfTestSlow() {
    return selfTestAt(outputSlow, "Robot/Tests/" + getName() + "/fast");
  }
}

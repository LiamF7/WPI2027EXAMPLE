package first.robot.mechanisms.manipulator;

import static org.wpilib.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.bearlib.BearMechanism;

public class Roller extends BearMechanism {

    private final double GEAR_RATIO = 1.0;

    private final double PIECE_THRESHOLD = Amps.of(3.879).in(Amps);

    private WantedState wantedState = WantedState.STOP;

    private SystemState systemState = SystemState.IDLE;

    public Roller() {
        super("EXAMPLE", 0, new CANBus("Default Name"));

        neutralMode(NeutralModeValue.Brake);
        inverted(InvertedValue.CounterClockwise_Positive);
        sensorToMechanismRatio(GEAR_RATIO);
        kP(10);
        kI(0);
        kD(0);

        addConfig();
    }

    @Override
    public void periodic(){
        super.periodic();

        systemState = handleStateTransitions();
        applyStates();
    }

    public enum WantedState{
        RUN,
        STOP,
        REVERSE,
    }

    private enum SystemState{
        FORWARD,
        REVERSE,
        BRAKE,
        IDLE,
    }

    private SystemState handleStateTransitions(){
        return switch(wantedState){
            case STOP: {
                if (hasPiece()){
                    yield SystemState.BRAKE;
                }
                yield SystemState.IDLE;
            }
            case RUN : {
                yield SystemState.FORWARD;
            }
            case REVERSE: {
                yield SystemState.REVERSE;
            }
        };
    }

    private void applyStates(){
        double rpm;
        boolean brake;

        switch (systemState) {
            case BRAKE:
                brake = true;
                rpm = 0.0;                
                break;
            case IDLE:
                brake = false;
                rpm = 0.0;
                break;
            case FORWARD:
                brake = false;
                rpm = 1000;
                break;
            case REVERSE:
                brake = false;
                rpm = -1000;
                break;
            default:
                brake = false;
                rpm = 0.0;
                break;
        }
        if(brake){
            setBrake();
            return;
        }
        setVTCFOC(rpm);    
    }

    public void setState(WantedState wantedState){
        this.wantedState = wantedState;
    }

    public boolean hasPiece(){
        return getSupplyCurrent() >= PIECE_THRESHOLD;
    }

    public boolean isStopped(){
        return Math.abs(getVelocity()) < 0.5;
    }
}

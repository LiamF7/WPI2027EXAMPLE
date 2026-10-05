package first.robot.mechanisms.exampleMechanism;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.bearlib.BearMechanism;

public class Example extends BearMechanism {

    private final double gearRatio = 1.0;

    public Example() {
        super("EXAMPLE", 0, null);

        // config values
        neutralMode(NeutralModeValue.Brake);
        inverted(InvertedValue.CounterClockwise_Positive);
        sensorToMechanismRatio(gearRatio);
        kP(10);
        kI(0);
        kD(0);

        // apply config to motor
        addConfig();
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}

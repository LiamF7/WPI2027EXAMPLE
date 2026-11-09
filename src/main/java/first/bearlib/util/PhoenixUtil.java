package first.bearlib.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.Supplier;

import org.wpilib.driverstation.DriverStationErrors;


/**
 * Utility class for applying a {@link TalonFXConfiguration} to a {@link TalonFX} CTRE motor.
 */
public class PhoenixUtil {

  /**
   * Maximum attempts in which a the code can initialize the config.
   */
  private static final int DEFAULT_MAX_ATTEMPTS = 5;

  /**
   * Applys a a {@link TalonFXConfiguration} to a {@link TalonFX} CTRE motor.
   */
  public static void applyConfig(Supplier<StatusCode> command, String name) {
    applyConfig(DEFAULT_MAX_ATTEMPTS, command, name);
  }

  
  /**
   * Applys a {@link TalonFXConfiguration} to a {@link TalonFX} CTRE motor.
   */
  public static void applyConfig(int maxAttempts, Supplier<StatusCode> command, String name) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < maxAttempts; i++) {
      status = command.get();
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      // report
      DriverStationErrors.reportError("<<< ERROR INSTANTIATING: " + name + " >>>", false);
    }
  }
}

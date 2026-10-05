package first.bearlib.fms;

import org.wpilib.driverstation.Alliance;

/**
 * Interface for listening to updates about the alliance color during a match. Classes implementing
 * this interface can react to changes in the alliance color, as notified by the system.
 */
public interface AllianceReadyListener {

  /**
   * Called whenever the alliance color is updated.
   *
   * @param alliance The updated {@link Alliance} value.
   */
  void updateAlliance(Alliance alliance);
}

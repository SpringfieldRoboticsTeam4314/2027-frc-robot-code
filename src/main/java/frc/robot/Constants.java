package frc.robot;
import static edu.wpi.first.units.Units.*;
import frc.robot.generated.TunerConstants;

public final class Constants {

  /**
   * Container for all robot constants.
   */
  public static final class Robot {
    public static final double MaxSpeed =  .5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }
}
package frc.robot.subsystems.outakeLift;

import org.littletonrobotics.junction.AutoLog;

public interface OutakeLiftIO {
  @AutoLog
  public static class OutakeLiftIOInputs {
    public double positionRad = 0.0;
    public double outputVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(OutakeLiftIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setPosition(double velocityRadPerSec, double ffVolts) {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}

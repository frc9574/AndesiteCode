package frc.robot.subsystems.intakeGuard;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeGuardIO {
  @AutoLog
  public static class IntakeGuardIOInputs {
    public double positionRads = 0.0;
    public double outputVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeGuardIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setPosition(double positionRads) {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}

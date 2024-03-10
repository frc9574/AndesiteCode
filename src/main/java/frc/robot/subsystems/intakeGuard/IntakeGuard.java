package frc.robot.subsystems.intakeGuard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeGuard extends SubsystemBase {
  private final IntakeGuardIO io;
  private final IntakeGuardIOInputsAutoLogged inputs = new IntakeGuardIOInputsAutoLogged();
  private final GenericEntry liftSetpoint;

  public static final double maxPos = 0.53;

  /** Creates a new Lift. */
  public IntakeGuard(IntakeGuardIO io) {
    this.io = io;

    ShuffleboardTab tab = Shuffleboard.getTab("IntakeGuard");
    liftSetpoint = tab.add("Setpoint", 0.0).getEntry();

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case SIM:
        io.configurePID(0.25, 0.0, 0.0);
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Take the overide input and apply it to the lift
    for (NetworkTableValue entry : liftSetpoint.readQueue()) {
      if (entry.getType() == NetworkTableType.kDouble) {
        runPosition(entry.getDouble());
      }
    }

    Logger.processInputs("IntakeGuard", inputs);
  }

  /** Run closed loop at the specified position. */
  public void runPosition(double positionRads) {
    // Clamp setpoint to be within the range of the lift
    positionRads = Math.max(0, Math.min(maxPos, positionRads));

    // Set the setpoint
    io.setPosition(positionRads);

    // Log flywheel setpoint
    Logger.recordOutput("IntakeGuard/SetpointRad", positionRads);
    liftSetpoint.setDouble(positionRads);
    liftSetpoint.readQueue();
  }

  @AutoLogOutput
  public double getPositionRads() {
    return inputs.positionRads;
  }

  public void goOut() {
    runPosition(maxPos);
  }

  public void goIn() {
    runPosition(0);
  }
}

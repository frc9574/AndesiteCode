package frc.robot.subsystems.lift;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Lift extends SubsystemBase {
  private final LiftIO io;
  private final LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();
  private final GenericEntry liftSetpoint;

  public static final double maxPos = 10000;
  public static final double gearRatio = 1.0 / 12.0;

  public double target_position = 0;

  /** Creates a new Lift. */
  public Lift(LiftIO io) {
    this.io = io;

    ShuffleboardTab tab = Shuffleboard.getTab("Lift");
    liftSetpoint = tab.add("Setpoint", 0.0).getEntry();

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        break;
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
        Logger.recordOutput("Lift/OverideRad", entry.getDouble());
        runPosition(entry.getDouble());
      }
    }

    Logger.processInputs("Lift", inputs);
  }

  /** Run closed loop at the specified position. */
  public void runPosition(double positionM) {
    // Clamp setpoint to be within the range of the lift
    positionM = Math.max(-1000, Math.min(maxPos, positionM));

    // Set the setpoint
    io.setPosition(positionM);
    target_position = positionM;

    // Log flywheel setpoint
    Logger.recordOutput("Lift/SetpointRad", positionM);
    liftSetpoint.setDouble(positionM);
    liftSetpoint.readQueue();
  }

  public void moveBy(double deltaM) {
    runPosition(target_position + deltaM);
  }

  @AutoLogOutput
  public double getPositionM() {
    return inputs.positionM;
  }
}

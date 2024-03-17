package frc.robot.subsystems.outakeLift;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class OutakeLift extends SubsystemBase {
  private final OutakeLiftIO io;
  private final OutakeLiftIOInputsAutoLogged inputs = new OutakeLiftIOInputsAutoLogged();
  private final ArmFeedforward ffModel;
  private final GenericEntry liftSetpoint;

  public static final double baseAngle = 0.0;
  public static final double maxAngle = 1.2;
  public static final double gearRatio = 0.0464;

  /** Creates a new Lift. */
  public OutakeLift(OutakeLiftIO io) {
    this.io = io;

    ShuffleboardTab tab = Shuffleboard.getTab("OutakeLift");
    liftSetpoint = tab.add("Setpoint", 0.0).getEntry();

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new ArmFeedforward(0, 1, 0);
        io.configurePID(0.1, 0.00001, 0.0);
        break;
      case SIM:
        ffModel = new ArmFeedforward(0, 0.5, 0);
        io.configurePID(0.25, 0.0, 0.0);
        break;
      default:
        ffModel = new ArmFeedforward(0.0, 0.0, 0.0);
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

    Logger.processInputs("OutakeLift", inputs);
  }

  /** Run closed loop at the specified position. */
  public void runPosition(double angleRad) {
    // Clamp setpoint to be within the range of the lift
    angleRad = Math.max(baseAngle, Math.min(maxAngle, angleRad));

    // Set the setpoint
    io.setPosition(angleRad, ffModel.calculate(angleRad, 0, 0));

    // Log flywheel setpoint
    Logger.recordOutput("OutakeLift/SetpointRad", angleRad);
    liftSetpoint.setDouble(angleRad);
    liftSetpoint.readQueue();
  }

  @AutoLogOutput
  public double getPositionRad() {
    return inputs.positionRad;
  }
}

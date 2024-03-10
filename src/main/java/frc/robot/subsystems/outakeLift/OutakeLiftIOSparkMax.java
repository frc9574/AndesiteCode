package frc.robot.subsystems.outakeLift;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class OutakeLiftIOSparkMax implements OutakeLiftIO {
  private final CANSparkMax leader = new CANSparkMax(14, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkPIDController pid = leader.getPIDController();

  public OutakeLiftIOSparkMax() {
    leader.restoreFactoryDefaults();
    leader.setIdleMode(IdleMode.kBrake);

    leader.setCANTimeout(250);

    leader.setInverted(false);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(80);

    configurePID(0.1, 0.0, 0.0);

    leader.burnFlash();
  }

  @Override
  public void updateInputs(OutakeLiftIOInputs inputs) {
    if (encoder.getPosition() < 0) {
      encoder.setPosition(0);
    }

    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() * OutakeLift.gearRatio);
    Logger.recordOutput("OutakeLift/SparkRawEncoder", encoder.getPosition());
    inputs.outputVolts = leader.getAppliedOutput();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
  }

  private boolean hasReset = false;

  @Override
  public void setPosition(double positionRads, double ffVolts) {
    if (!hasReset) {
      encoder.setPosition(0);
      hasReset = true;
    }

    Logger.recordOutput("OutakeLift/FFVolts", ffVolts);

    if (positionRads == 0) {
      leader.set(0);
      return;
    }

    pid.setReference(
        Units.radiansToRotations(positionRads / OutakeLift.gearRatio),
        ControlType.kPosition,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}

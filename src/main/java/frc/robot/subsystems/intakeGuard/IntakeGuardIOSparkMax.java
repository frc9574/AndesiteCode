package frc.robot.subsystems.intakeGuard;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeGuardIOSparkMax implements IntakeGuardIO {
  private final CANSparkMax leader = new CANSparkMax(16, MotorType.kBrushed);
  private final RelativeEncoder encoder;
  private final SparkPIDController pid = leader.getPIDController();

  public IntakeGuardIOSparkMax() {
    leader.restoreFactoryDefaults();
    leader.setIdleMode(IdleMode.kBrake);

    leader.setCANTimeout(250);

    leader.setInverted(false);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(20);

    configurePID(3, 0.0, 0.0);

    encoder = leader.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 3500);

    leader.burnFlash();
  }

  @Override
  public void updateInputs(IntakeGuardIOInputs inputs) {
    inputs.positionRads = encoder.getPosition();
    inputs.outputVolts = leader.getAppliedOutput();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
  }

  private boolean hasReset = false;

  @Override
  public void setPosition(double positionRads) {
    if (!hasReset) {
      encoder.setPosition(0);
      hasReset = true;
    }

    pid.setReference(-positionRads, ControlType.kPosition);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}

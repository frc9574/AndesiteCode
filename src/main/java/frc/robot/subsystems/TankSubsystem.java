package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TankSubsystem extends SubsystemBase {
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

    private static final double kTrackWidth = 0.381 * 2; // meters
    private static final double kWheelRadius = 0.0508; // meters
    private static final int kEncoderResolution = 4096;

    private final CANSparkMax m_l1 = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax m_l2 = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax m_r1 = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax m_r2 = new CANSparkMax(4, MotorType.kBrushless);

    private final SparkAbsoluteEncoder m_leftEncoder = m_l1.getAbsoluteEncoder(Type.kDutyCycle);
    private final SparkAbsoluteEncoder m_rightEncoder = m_r1.getAbsoluteEncoder(Type.kDutyCycle);
    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
    private final DifferentialDriveOdometry m_odometry;

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

    // Stuff for teleop
    private final CommandXboxController m_driverController;
    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

    // Simulation classes help us simulate our robot
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    private final Field2d m_fieldSim = new Field2d();
    private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
            0.3);
    private final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(
            m_drivetrainSystem, DCMotor.getNEO(2), 8, kTrackWidth, kWheelRadius, null);

    public TankSubsystem(IdleMode idleMode) {
        this(null, idleMode);
    }

    public TankSubsystem(CommandXboxController driverController, IdleMode idleMode) {
        CommandScheduler.getInstance().registerSubsystem(this);

        m_gyro.reset();

        if (!DriverStation.isAutonomous()) {
            m_driverController = driverController;
        } else {
            m_driverController = null;
        }

        // Set the follower motors to follow the leader motors
        m_l2.follow(m_l1);
        m_r2.follow(m_r1);
        m_r1.setInverted(true);

        setIdleMode(idleMode);

        // Set the distance per pulse for the encoders
        m_leftEncoder.setPositionConversionFactor(kWheelRadius * 2 * Math.PI / kEncoderResolution);
        m_rightEncoder.setPositionConversionFactor(kWheelRadius * 2 * Math.PI / kEncoderResolution);

        m_odometry = new DifferentialDriveOdometry(
                m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

        SmartDashboard.putData("Field", m_fieldSim);
    }

    public void initSimulation() {
        REVPhysicsSim.getInstance().addSparkMax(m_l1, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(m_l2, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(m_r1, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(m_r2, DCMotor.getNEO(1));
    }

    public void setIdleMode(IdleMode idleMode) {
        m_l1.setIdleMode(idleMode);
        m_l2.setIdleMode(idleMode);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getVelocity(),
                speeds.leftMetersPerSecond);
        final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getVelocity(),
                speeds.rightMetersPerSecond);
        m_l1.setVoltage(leftOutput + leftFeedforward);
        m_r1.setVoltage(rightOutput + rightFeedforward);
    }

    public void stop() {
        m_l1.stopMotor();
        m_l2.stopMotor();
    }

    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    public void updateOdometry() {
        m_odometry.update(
                m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
                m_fieldSim.setRobotPose(getPose());
    }

    @Override
    public void periodic() {
        updateOdometry();
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());

        // If the driver controller is not null and the opmode is teleop or test
        if (m_driverController != null) {
            final var xSpeed = -m_speedLimiter.calculate(m_driverController.getLeftY()) * kMaxSpeed;

            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            final var rot = -m_rotLimiter.calculate(m_driverController.getRightX()) * kMaxAngularSpeed;

            drive(xSpeed, rot);
        }
    }

    @Override
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        m_drivetrainSimulator.setInputs(
            m_l1.get() * RobotController.getInputVoltage(),
            m_r1.get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(0.02);

        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
}

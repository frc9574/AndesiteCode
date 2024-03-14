// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavx2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.indexing.Indexing;
import frc.robot.subsystems.indexing.IndexingIO;
import frc.robot.subsystems.indexing.IndexingIOSim;
import frc.robot.subsystems.indexing.IndexingIOSparkMax;
import frc.robot.subsystems.intakeGuard.IntakeGuard;
import frc.robot.subsystems.intakeGuard.IntakeGuardIOSparkMax;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.lift.LiftIO;
import frc.robot.subsystems.lift.LiftIOSim;
import frc.robot.subsystems.lift.LiftIOSparkMax;
import frc.robot.subsystems.outakeLift.OutakeLift;
import frc.robot.subsystems.outakeLift.OutakeLiftIO;
import frc.robot.subsystems.outakeLift.OutakeLiftIOSim;
import frc.robot.subsystems.outakeLift.OutakeLiftIOSparkMax;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final OutakeLift outtakeLift;
  private final Indexing indexing;
  private final Lift robotLift;
  private final IntakeGuard intakeGuard;

  private DoubleArrayEntry tagEntry;

  // Controller
  private final CommandPS4Controller driveController = new CommandPS4Controller(0);
  private final CommandXboxController controller = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);
  private final LoggedDashboardNumber outakeAngleFalloff =
      new LoggedDashboardNumber("Outake angle falloff", 0.0025);
  private final LoggedDashboardNumber minOutakeFalloffDist =
      new LoggedDashboardNumber("Minimum outake distance for falloff", 1.50);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavx2(),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(0));
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        outtakeLift = new OutakeLift(new OutakeLiftIOSparkMax());
        indexing = new Indexing(new IndexingIOSparkMax());
        robotLift = new Lift(new LiftIOSparkMax());
        intakeGuard = new IntakeGuard(new IntakeGuardIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        outtakeLift = new OutakeLift(new OutakeLiftIOSim());
        indexing = new Indexing(new IndexingIOSim());
        robotLift = new Lift(new LiftIOSim());
        intakeGuard = new IntakeGuard(new IntakeGuardIOSparkMax());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        outtakeLift = new OutakeLift(new OutakeLiftIO() {});
        indexing = new Indexing(new IndexingIO() {});
        robotLift = new Lift(new LiftIO() {});
        intakeGuard = new IntakeGuard(new IntakeGuardIOSparkMax());
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Turn Modules continously", drive.turnTuning());
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive Wear in", drive.sysIdWearIn());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // drive.setDefaultCommand(
    //     DriveCommands.testDrive(
    //         drive,
    //         () -> driveController.getLeftY(),
    //         () -> driveController.getLeftX(),
    //         () -> driveController.getRightX()));
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> driveController.getRightX()));

    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    CameraServer.startAutomaticCapture();

    ShuffleboardTab tab = Shuffleboard.getTab("OutakeLift");
    Double defaultHeight = 1.16;
    GenericEntry intakeSetpoint = tab.add("IntakeSetpoint", 0.77).getEntry();

    Timer robotLiftTimer = new Timer();
    robotLift.setDefaultCommand(
        Commands.runOnce(robotLiftTimer::start)
            .andThen(
                Commands.run(
                    () -> {
                      robotLift.moveBy(
                          (controller.getRightTriggerAxis() * 5 - controller.getLeftTriggerAxis())
                              // Move 5 cm per second up, and 1/s down
                              * robotLiftTimer.get());
                      robotLiftTimer.reset();
                    },
                    robotLift)));

    driveController.cross().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driveController.options().onTrue(Commands.runOnce(drive::stopWithReset, drive));
    controller
        .start()
        .whileTrue(Commands.startEnd(intakeGuard::goOut, intakeGuard::goIn, intakeGuard));

    controller
        .a()
        .whileTrue(
            Commands.run(
                    () -> {
                      flywheel.runVelocity((this.getDistance() * 100 * 0.828) + 1793);
                      Logger.recordOutput("Vision/ObservedDistance", this.getDistance());
                    },
                    flywheel)
                .finallyDo(flywheel::stop))
        .whileTrue(
            Commands.run(
                    () -> {
                      outtakeLift.runPosition(
                          -(this.getDistance() * 100 * outakeAngleFalloff.get())
                              + minOutakeFalloffDist.get());
                      Logger.recordOutput("Vision/ObservedDistance", this.getDistance());
                    },
                    outtakeLift)
                .finallyDo(() -> outtakeLift.runPosition(0.0)));
    controller
        .b()
        .whileTrue(Commands.startEnd(() -> flywheel.runVelocity(-1000), flywheel::stop, flywheel))
        .whileTrue(Commands.startEnd(intakeGuard::goOut, intakeGuard::goIn, intakeGuard))
        .whileTrue(
            Commands.startEnd(
                () -> outtakeLift.runPosition(intakeSetpoint.getDouble(defaultHeight)),
                () -> outtakeLift.runPosition(0.0),
                outtakeLift))
        .whileTrue(Commands.startEnd(() -> indexing.runVolts(-12.0), indexing::stop, indexing));
    controller
        .rightBumper()
        .whileTrue(Commands.startEnd(() -> indexing.runVolts(12.0), indexing::stop, indexing));
    controller
        .povUp()
        .whileTrue(Commands.startEnd(() -> indexing.runVolts(12.0), indexing::stop, indexing));
    controller
        .povDown()
        .whileTrue(Commands.startEnd(() -> indexing.runVolts(-12.0), indexing::stop, indexing));
  }

  public void updateAlliance() {
    tagEntry =
        NetworkTableInstance.getDefault()
            .getTable("Top_Right")
            .getDoubleArrayTopic(DriverStation.getAlliance().equals(Optional.of(Alliance.Blue)) ? "tag_7" : "tag_4")
            .getEntry(new double[] {0, 0});
  }

  public double getDistance() {
    // Pythag
    return Math.sqrt(Math.pow(tagEntry.get()[0], 2) + Math.pow(tagEntry.get()[1], 2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

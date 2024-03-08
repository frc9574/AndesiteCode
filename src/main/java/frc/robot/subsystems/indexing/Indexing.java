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

package frc.robot.subsystems.indexing;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexing extends SubsystemBase {
  private final IndexingIO io;
  private final IndexingIOInputsAutoLogged inputs = new IndexingIOInputsAutoLogged();

  /** Creates a new Flywheel. */
  public Indexing(IndexingIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexing", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }
}

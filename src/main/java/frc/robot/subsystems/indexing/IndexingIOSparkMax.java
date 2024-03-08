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

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IndexingIOSparkMax implements IndexingIO {
  private final CANSparkMax leader = new CANSparkMax(12, MotorType.kBrushed);
  private final CANSparkMax follower = new CANSparkMax(13, MotorType.kBrushed);

  public IndexingIOSparkMax() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    leader.setCANTimeout(250);
    follower.setCANTimeout(250);

    leader.setInverted(true);
    follower.follow(leader, true);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(40);

    leader.burnFlash();
    follower.burnFlash();
  }

  @Override
  public void updateInputs(IndexingIOInputs inputs) {
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}

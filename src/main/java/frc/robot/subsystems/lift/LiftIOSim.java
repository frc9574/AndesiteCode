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

package frc.robot.subsystems.lift;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class LiftIOSim implements LiftIO {
  private ElevatorSim sim = new ElevatorSim(DCMotor.getNEO(1), 12, 50, 0.03, 0, 0.5, true, 0);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = MathUtil.clamp(pid.calculate(sim.getPositionMeters()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.positionM = sim.getPositionMeters();
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setPosition(double positionRads) {
    closedLoop = true;
    pid.setSetpoint(positionRads);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}

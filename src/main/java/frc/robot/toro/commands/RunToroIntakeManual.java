// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.toro.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.toro.ToroIntakeConfig;
import frc.robot.toro.ToroRoller;

public class RunToroIntakeManual extends CommandBase {
  private final DoubleSupplier supplier;
    private final ToroRoller roller;

    public RunToroIntakeManual(ToroRoller roller, DoubleSupplier supplier) {
        this.supplier = supplier;
        this.roller = roller;

        addRequirements(roller);
    }

    @Override
    public String getName() {
        return "ToroIntake.RunToroIntakeManual";
    }

    @Override
    public void initialize() {
        roller.setRollerProfile(ToroIntakeConfig.kDefaultProfile);
    }

    @Override
    public void execute() {
        roller.setRollerOutput(supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        roller.setRollerOutput(0);
    }
}

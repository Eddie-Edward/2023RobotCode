// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.toro.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.toro.ToroIntakeConfig;
import frc.robot.toro.ToroRoller;

public class RunToroIntake extends CommandBase {
  public enum Mode {
    kCube(ToroIntakeConfig.kCubeProfile), kCone(ToroIntakeConfig.kConeProfile), kOuttake(ToroIntakeConfig.kOuttake);

    Mode(ToroIntakeConfig.ToroIntakeProfile profile) {
        kProfile = profile;
    }

    private final ToroIntakeConfig.ToroIntakeProfile kProfile;
}

private final ToroRoller roller;
private final Mode mode;

public RunToroIntake(ToroRoller roller, Mode mode) {
    this.roller = roller;
    this.mode = mode;

    addRequirements(roller);
}

@Override
public String getName() {
    return "RunToroIntake[" + mode.name() + "]";
}

@Override
public void initialize() {
    roller.setRollerProfile(mode.kProfile);
    roller.setRollerOutput(1);
}

@Override
public void end(boolean interrupted) {
    roller.stop();
}
}

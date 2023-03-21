// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.toro.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.toro.Toro;
import frc.robot.toro.ToroIntakeConfig;
import frc.robot.toro.ToroIntakeConfig.ToroState;

public class ToggleToro extends CommandBase {
  private final ToroState s1;
  private final Toro t1;

    public ToggleToro(Toro temp, ToroState s1) {
        this.s1 = s1;
        t1 = temp;
        addRequirements(t1);
    }

    @Override
    public String getName() {
        return "ToggleToro";
    }

    @Override
    public void initialize() {
        final ToroIntakeConfig.ToroState newState;
        if (t1.getState() == ToroIntakeConfig.ToroState.kOpen) {
            newState = ToroIntakeConfig.ToroState.kClosed;
        } else {
            newState = ToroIntakeConfig.ToroState.kOpen;
        }

        t1.setState(newState);

        System.out.println(getName() + ": set state " + newState.name());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

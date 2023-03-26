// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.intake.IntakePivot;
import frc.robot.intake.IntakeConfig.PivotState;

public class ClearIntakeDown extends CommandBase {
  /** Creates a new ClearIntakeDown. */
  // private IntakePivot pivot;
  // private double currAngle;
  // private boolean isFinished;
  // public ClearIntakeDown(IntakePivot pivot) {
  //   this.pivot = pivot;

  // }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {
  //   currAngle = pivot.getAngleRads();
  //   isFinished = false;
  // }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   if(!(currAngle > 90 && currAngle < 180)) {
  //     isFinished = true;
  //   }
  //   else {
  //     isFinished = false;
  //   }
  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return isFinished;
  // }
}

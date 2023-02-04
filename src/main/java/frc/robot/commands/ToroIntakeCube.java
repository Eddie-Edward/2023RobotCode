// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ToroIntake;

public class ToroIntakeCube extends CommandBase {
  /** Creates a new ToroIntakeCone. */

  private ToroIntake IntakeCube;
  XboxController joy1; 

  public ToroIntakeCube(ToroIntake m_intake, XboxController joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    IntakeCube = m_intake;
    joy1 = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeCube.setMotorCurrentLimit(Constants.ToroIntakeConstants.CubeIntakeCurrentLimit, Constants.ToroIntakeConstants.CubeIntakeTargetVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeCube.StartCubeIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeCube.StopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

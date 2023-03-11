// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCone extends CommandBase {
  private Intake IntakeCone;
  XboxController joy1;
  

  public IntakeCone(Intake intake, XboxController joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    IntakeCone = intake;
    joy1 = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeCone.setWheelMotorCurrentLimit(Constants.IntakeConstants.ConeIntakeCurrentLimit, Constants.IntakeConstants.ConeIntakeTargetVoltage);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeCone.startIntake(1);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeCone.stopIntake();
  }
    

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

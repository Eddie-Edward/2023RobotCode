// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCoCu extends CommandBase {
  /** Creates a new IntakeCoCu. */

  private Intake IntakeCube, IntakeCone;
  Joystick joy1;

  public IntakeCoCu(Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    joy1 = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeCube = new Intake(joy1);
    IntakeCone = new Intake(joy1);

    IntakeCube.setMotorCurrentLimit(Constants.IntakeConstants.CubeIntakeCurrentLimit, Constants.IntakeConstants.CubeIntakeTargetVoltage);
    IntakeCone.setMotorCurrentLimit(Constants.IntakeConstants.ConeIntakeCurrentLimit, Constants.IntakeConstants.ConeIntakeTargetVoltage);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeCone.startIntake();
    IntakeCube.startIntake();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeCone.stopIntake();
    IntakeCube.stopIntake();
  }
    

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

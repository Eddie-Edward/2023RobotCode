// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCube extends CommandBase {
  /** Creates a new IntakeCoCu. */

  private Intake IntakeCube;
  XboxController joy1;
  

  public IntakeCube(Intake intake, XboxController joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    IntakeCube = intake;
    joy1 = joy;
    addRequirements(IntakeCube);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeCube.setWheelMotorCurrentLimit(Constants.IntakeConstants.CubeIntakeCurrentLimit, Constants.IntakeConstants.CubeIntakeTargetVoltage);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeCube.startIntake(1);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeCube.stopIntake();
  }
    

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

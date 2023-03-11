// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ExtendIntake extends CommandBase {
  /** Creates a new ExtendIntake. */
  private Intake extendIntake;
  XboxController joy1;
  private final PIDController intakeController;

  private double output, encoderValue;
  private final double INTAKE_kP = 0.0;
  private final double INTAKE_kI = 0.0;
  private final double INTAKE_kD = 0.0;

  private final int TARGET_INTAKE_TICKS;
  private final DutyCycleEncoder INTAKE_ENCODER;
  
  public ExtendIntake(Intake intake, XboxController joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    extendIntake = intake;
    joy1 = joy;
    intakeController = new PIDController(INTAKE_kP, INTAKE_kI, INTAKE_kD);
    TARGET_INTAKE_TICKS = 0;
    INTAKE_ENCODER = new DutyCycleEncoder(3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extendIntake.setIntakeMotorCurrentLimit(Constants.IntakeConstants.ToiletIntakeCurrentLimit, Constants.IntakeConstants.ToiletIntakeTargetVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encoderValue = INTAKE_ENCODER.get();
    output = MathUtil.clamp(intakeController.calculate(encoderValue, TARGET_INTAKE_TICKS), 0, 1);
    extendIntake.setIntakeSpeed(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extendIntake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

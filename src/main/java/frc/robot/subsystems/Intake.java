// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.ToIntFunction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.*;




public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private CANSparkMax wheelMotor, intakeMotor;
  private DoubleSolenoid hoodBar;
  


  public Intake() {
    wheelMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_WHEEL_NEO_ID_1, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE__NEO_ID_2, MotorType.kBrushless);
    hoodBar = new DoubleSolenoid(null, 0, 0);    
  }
  
  public void stopIntake() {
    wheelMotor.set(0);
  }

  public void startIntake(double speed) {
    wheelMotor.set(speed);
  }

  public void retractHood() {
    hoodBar.set(Value.kReverse);
  }

  public void extendHood() {
    hoodBar.set(Value.kForward);
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  public void getMotorCurrent(){
  //   wheelMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_NEO_ID_1, MotorType.kBrushless);
  //   toiletMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_NEO_ID_2, MotorType.kBrushless);
  // SmartDashboard.putNumber("Current from Motor 1:", wheelMotor.getOutputCurrent());
  // SmartDashboard.putNumber("Current from Motor 2:", toiletMotor.getOutputCurrent());
  }

  public void setWheelMotorCurrentLimit(int currentLimit, double targetVoltage){
    wheelMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_WHEEL_NEO_ID_1, MotorType.kBrushless);
    
    wheelMotor.setSmartCurrentLimit(currentLimit);

    wheelMotor.setVoltage(targetVoltage);

  }

  public void setIntakeMotorCurrentLimit(int currentLimit, double targetVoltage){
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE__NEO_ID_2, MotorType.kBrushless);
    
    intakeMotor.setSmartCurrentLimit(currentLimit);

    intakeMotor.setVoltage(targetVoltage);

  }
}


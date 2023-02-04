// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.*;




public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private CANSparkMax m_neoMotor, m_neoMotor2;
  private DoubleSolenoid twoBar;


  public Intake() {
    m_neoMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_NEO_ID_1, MotorType.kBrushless);
    m_neoMotor2 = new CANSparkMax(Constants.IntakeConstants.INTAKE_NEO_ID_2, MotorType.kBrushless);
    twoBar = new DoubleSolenoid(null, 0, 0);
    
  }
  
  public void stopIntake() {
    m_neoMotor.set(0);
    m_neoMotor2.set(0);
  }

  public void startIntake() {
    m_neoMotor.set(1);
    m_neoMotor2.set(1);
  }

  public void extendIntake(){
    if(twoBar.get() != Value.kReverse) {
      twoBar.set(Value.kReverse);
    }
  }

  public void retractIntake(){
    if(twoBar.get() != Value.kForward){
      twoBar.set(Value.kForward);
    }
  }

  public void getMotorCurrent(){
  m_neoMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_NEO_ID_1, MotorType.kBrushless);
  m_neoMotor2 = new CANSparkMax(Constants.IntakeConstants.INTAKE_NEO_ID_2, MotorType.kBrushless);
  SmartDashboard.putNumber("Current from Motor 1:", m_neoMotor.getOutputCurrent());
  SmartDashboard.putNumber("Current from Motor 2:", m_neoMotor2.getOutputCurrent());
  }

  public void setMotorCurrentLimit(int currentLimit, double targetVoltage){
    m_neoMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_NEO_ID_1, MotorType.kBrushless);
    m_neoMotor2 = new CANSparkMax(Constants.IntakeConstants.INTAKE_NEO_ID_2, MotorType.kBrushless);
    
    m_neoMotor.setSmartCurrentLimit(currentLimit);
    m_neoMotor2.setSmartCurrentLimit(currentLimit);

    m_neoMotor.setVoltage(targetVoltage);
    m_neoMotor2.setVoltage(targetVoltage);

    }
}


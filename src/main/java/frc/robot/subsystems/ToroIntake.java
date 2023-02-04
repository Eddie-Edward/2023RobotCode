// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

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

public class ToroIntake extends SubsystemBase {
  /** Creates a new ToroIntake. */
  private CANSparkMax m_NeoMotorL1, m_NeoMotorL2, m_NeoMotorR1, m_NeoMotorR2;
  private DoubleSolenoid m_BarL, m_BarR;
  
  public ToroIntake() {
    m_NeoMotorL1 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_L1, MotorType.kBrushless);
    m_NeoMotorL2 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_L2, MotorType.kBrushless);
    m_NeoMotorR1 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_R1, MotorType.kBrushless);
    m_NeoMotorR2 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_R2, MotorType.kBrushless);
    m_BarL = new DoubleSolenoid(null, 0, 0);
    m_BarR = new DoubleSolenoid(null, 0, 0);
  }

  public void StartConeIntake() {
    m_NeoMotorL1.set(1);
    m_NeoMotorL2.set(1);
    m_NeoMotorR1.set(1);
    m_NeoMotorR2.set(1);
  }

  public void StopIntake(){
    m_NeoMotorL1.set(0);
    m_NeoMotorL2.set(0);
    m_NeoMotorR1.set(0);
    m_NeoMotorR2.set(0);
  }

  public void StartCubeIntake() {
    m_NeoMotorL1.set(1);
    m_NeoMotorR1.set(1);
    m_NeoMotorL2.set(0);
    m_NeoMotorR2.set(0);
  }

  public void openIntake() {
    if(m_BarL.get() != Value.kReverse && m_BarR.get() != Value.kReverse) {
      m_BarL.set(Value.kReverse);
      m_BarR.set(Value.kReverse);
    }
  }

  public void closeIntake() {
    if(m_BarL.get() != Value.kForward && m_BarR.get() != Value.kForward) {
      m_BarL.set(Value.kForward);
      m_BarR.set(Value.kForward);
    }
  }

  public void getMotorCurrent(){
    m_NeoMotorL1 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_L1, MotorType.kBrushless);
    m_NeoMotorL2 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_L2, MotorType.kBrushless);
    m_NeoMotorR1 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_R1, MotorType.kBrushless);
    m_NeoMotorR2 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_R2, MotorType.kBrushless);
    SmartDashboard.putNumber("Current from Front Left Motor:", m_NeoMotorL1.getOutputCurrent());
    SmartDashboard.putNumber("Current from Front Right Motor:", m_NeoMotorR1.getOutputCurrent());
    SmartDashboard.putNumber("Current from Back Left Motor:", m_NeoMotorL2.getOutputCurrent());
    SmartDashboard.putNumber("Current from Back Right Motor:", m_NeoMotorR2.getOutputCurrent());
    }

  public void setMotorCurrentLimit(int currentLimit, double targetVoltage){
    m_NeoMotorL1 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_L1, MotorType.kBrushless);
    m_NeoMotorL2 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_L2, MotorType.kBrushless);
    m_NeoMotorR1 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_R1, MotorType.kBrushless);
    m_NeoMotorR2 = new CANSparkMax(Constants.ToroIntakeConstants.INTAKE_NEO_ID_R2, MotorType.kBrushless);
    
    m_NeoMotorR1.setSmartCurrentLimit(currentLimit);
    m_NeoMotorR2.setSmartCurrentLimit(currentLimit);
    m_NeoMotorL1.setSmartCurrentLimit(currentLimit);
    m_NeoMotorL2.setSmartCurrentLimit(currentLimit);

    m_NeoMotorR1.setVoltage(targetVoltage);
    m_NeoMotorR2.setVoltage(targetVoltage);
    m_NeoMotorL1.setVoltage(targetVoltage);
    m_NeoMotorL2.setVoltage(targetVoltage);
  }
}

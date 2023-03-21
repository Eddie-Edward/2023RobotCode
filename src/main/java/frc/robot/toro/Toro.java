// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.toro;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Toro extends SubsystemBase {
  private DoubleSolenoid solenoidR, solenoidL;
  private ToroIntakeConfig.ToroState state;

  public Toro() {
      solenoidR = new DoubleSolenoid(
              PneumaticsModuleType.CTREPCM,
              ToroIntakeConfig.kRightHoodForwardChannel,
              ToroIntakeConfig.kRightHoodReverseChannel
      );

      solenoidL = new DoubleSolenoid(
              PneumaticsModuleType.CTREPCM,
              ToroIntakeConfig.kLeftHoodForwardChannel,
              ToroIntakeConfig.kLeftHoodReverseChannel
      );

      state = ToroIntakeConfig.ToroState.kClosed;
  }

  public ToroIntakeConfig.ToroState getState() {
      return state;
  }

  public void setState(ToroIntakeConfig.ToroState state) {
      this.state = state;
      solenoidR.set(state.state);
      solenoidL.set(state.state);
  }
}

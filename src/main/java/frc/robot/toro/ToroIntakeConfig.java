// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.toro;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ToroIntakeConfig {
  /** Creates a new ToroIntakeConfig. */

  public enum ToroState {
    kOpen(DoubleSolenoid.Value.kReverse), kClosed(DoubleSolenoid.Value.kForward);

    ToroState(DoubleSolenoid.Value state) {
        this.state = state;
    }

    public final DoubleSolenoid.Value state;
  }
  public static class ToroIntakeProfile {
    private ToroIntakeProfile(int stallLimit, int freeLimit, double nominalSpeed, String name) {
        kStallCurrentLimit = stallLimit;
        kFreeCurrentLimit = freeLimit;
        kNominalOutput = nominalSpeed;
        kName = name;
    }

    @Override
    public String toString() {
        return kName;
    }

    final int kStallCurrentLimit;
    final int kFreeCurrentLimit;
    final double kNominalOutput;
    private final String kName;
}

    //Toro Intake Id
    public static final int INTAKE_NEO_ID_R1 = 24;
    public static final int INTAKE_NEO_ID_L1 = 23;
    public static final int INTAKE_NEO_ID_F = 0;


    //Current limit and target voltage limit
    public static final int CubeIntakeCurrentLimit = 0;
    public static final double CubeIntakeTargetVoltage = 0.0;
    public static final int ConeIntakeCurrentLimit = 0;

    // Intake Profiles
    public static ToroIntakeProfile kConeProfile = new ToroIntakeProfile(20, 40, 1, "Cone");
    public static ToroIntakeProfile kCubeProfile = new ToroIntakeProfile(10, 40, 1, "Cube");
    public static ToroIntakeProfile kOuttake = new ToroIntakeProfile(40, 40, -1, "Output");
    public static ToroIntakeProfile kDefaultProfile = new ToroIntakeProfile(30, 30, 1, "Default");

    public static final int kLeftHoodForwardChannel = 0;
    public static final int kLeftHoodReverseChannel = 1;   
    
    public static final int kRightHoodForwardChannel = 2;
    public static final int kRightHoodReverseChannel = 3;   

}

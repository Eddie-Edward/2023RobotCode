// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.toro;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ToroIntakeConfig extends SubsystemBase {
  /** Creates a new ToroIntakeConfig. */
  public ToroIntakeConfig() {
    public static final int INTAKE_NEO_ID_R1 = 24;
        public static final int INTAKE_NEO_ID_R2 = 22;
        public static final int INTAKE_NEO_ID_L1 = 23;
        public static final int INTAKE_NEO_ID_L2 = 21;
        public static final int CubeIntakeCurrentLimit = 0;
        public static final double CubeIntakeTargetVoltage = 0.0;
        public static final int ConeIntakeCurrentLimit = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

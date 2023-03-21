package frc.robot.toro;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ToroRoller extends SubsystemBase {
    private final CANSparkMax rollerSparkL, rollerSparkR, rollerSparkF;
    private double nominalOutput = 1.0;

    public ToroRoller() {
        rollerSparkL = new CANSparkMax(ToroIntakeConfig.INTAKE_NEO_ID_L1, CANSparkMaxLowLevel.MotorType.kBrushless);
        rollerSparkR = new CANSparkMax(ToroIntakeConfig.INTAKE_NEO_ID_R1, CANSparkMaxLowLevel.MotorType.kBrushless);
        rollerSparkF = new CANSparkMax(ToroIntakeConfig.INTAKE_NEO_ID_F, CANSparkMaxLowLevel.MotorType.kBrushless);
        setRollerProfile(ToroIntakeConfig.kDefaultProfile);
    }

    public void setRollerOutput(double output) {
        final var adjusted_output = output * nominalOutput;
        rollerSparkL.set(adjusted_output);
        rollerSparkR.set(adjusted_output);
        rollerSparkF.set(adjusted_output);
    }

    public void stop() {
        rollerSparkL.set(0);
        rollerSparkR.set(0);
        rollerSparkF.set(0);
    }

    public void setRollerProfile(ToroIntakeConfig.ToroIntakeProfile profile) {
        rollerSparkL.setSmartCurrentLimit(profile.kStallCurrentLimit, profile.kFreeCurrentLimit);
        rollerSparkR.setSmartCurrentLimit(profile.kStallCurrentLimit, profile.kFreeCurrentLimit);
        rollerSparkF.setSmartCurrentLimit(profile.kStallCurrentLimit, profile.kFreeCurrentLimit);
        nominalOutput = profile.kNominalOutput;
    }
}

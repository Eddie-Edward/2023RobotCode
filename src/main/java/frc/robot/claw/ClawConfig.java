package frc.robot.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ClawConfig {
    public enum ClawState {
        kOpen(DoubleSolenoid.Value.kForward), 
        kClosed(DoubleSolenoid.Value.kReverse);

        public final DoubleSolenoid.Value state;

        ClawState(DoubleSolenoid.Value state) {
            this.state = state;
        }

    }
    
    public static final int kClawForwardChannel = 2;
    public static final int kClawReverseChannel = 3;
}

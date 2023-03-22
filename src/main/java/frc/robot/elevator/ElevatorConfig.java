package frc.robot.elevator;

public class ElevatorConfig {
    public enum ElevatorPosition { 
        kMid(0), 
        kHigh(0),
        kZero(0),
        kUnspecified(0);
        
        public final int targetPos;

        ElevatorPosition(int targetPos) {
            this.targetPos = targetPos;
        }
    }

    public static final int kElevatorSparkID = 51;
    public static final int smartMotionSlot = 0;
    //Elevator PID Constants
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0; 

    //Elevator FF constants (for rio profiling) 
    public static final double elevkS = 0.0;
    public static final double elevkV = 0.0;
    public static final double elevkA = 0.0;
    public static final double elevkG = 0.0;

    //rio profiling max and min vel
    public static final double kMaxVelocity = 2000;
    public static final double kMinVelocity = 0;

    public static final double kIz = 0.0;
    public static final double kFF = 0.0;

    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;

    public static final double maxRPM = 5700;
    public static final double maxVel = 2000;
    public static final double minVel = 0;
    public static final double maxAcc = 1500;
    public static final double allowedErr = 0;

    public static final boolean kElevatorMotorInverted = false;

    public static final double kElevatorZeroOffset = 0.0;
    public static final boolean kElevatorEncoderInverted = false;
}


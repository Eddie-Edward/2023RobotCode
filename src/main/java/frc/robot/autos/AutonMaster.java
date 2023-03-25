package frc.robot.autos;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.intake.IntakePivot;
import frc.robot.intake.IntakeConfig.PivotState;
import frc.robot.intake.commands.RunIntake;
import frc.robot.intake.commands.SetPivotState;
import frc.robot.claw.ClawConfig.ClawState;
import frc.robot.claw.commands.ClawCommands;
import frc.robot.commands.ClearHood;
import frc.robot.commands.ElevatorZero;
import frc.robot.commands.HandoffCone;
import frc.robot.commands.HandoffCube;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.ScoreMid;
import frc.robot.elevator.ElevatorConfig.ElevatorState;
import frc.robot.elevator.commands.SetElevatorState;

public final class AutonMaster {

    private static final PathConstraints GENERIC_PATH_CONSTRAINTS = new PathConstraints(4, 3);
    
    public static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        Map.entry("Stop", new InstantCommand(RobotContainer.drivetrain::stopSwerve)),
        
        Map.entry("ZeroGyro", new InstantCommand(() -> RobotContainer.drivetrain.m_gyro.setYaw(RobotContainer.drivetrain.getYaw().getDegrees()))),
        
        Map.entry("IntakeDown", 
        new ParallelCommandGroup(
            new SetPivotState(RobotContainer.intakePivot, PivotState.kDeployed), 
            new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kCube))),

        
        Map.entry("IntakeUp", 
        new SetPivotState(RobotContainer.intakePivot, PivotState.kStowed)),

        Map.entry("ScoreHigh", new SequentialCommandGroup(
            ClawCommands.setState(ClawState.kClosed),   
            new ScoreHigh(),
            ClawCommands.setState(ClawState.kOpen),
            new ElevatorZero()
            )),

        Map.entry("ScoreMid", new SequentialCommandGroup(
            ClawCommands.setState(ClawState.kClosed),   
            new ScoreHigh(),
            ClawCommands.setState(ClawState.kOpen),
            new ElevatorZero()
            )),

        Map.entry("ScoreLow", new SequentialCommandGroup(
            new SetPivotState(RobotContainer.intakePivot, PivotState.kHumanPlayer),
            new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kOuttake)
        )),


        Map.entry("HandoffCone", new HandoffCone()),

        Map.entry("HandoffCube", new HandoffCube()),

        Map.entry("ClearHood", new ClearHood()),
        
        Map.entry("ElevatorDown", new SetElevatorState(RobotContainer.elevator, ElevatorState.kZero)),

        Map.entry("AutoBalance", new AutoBalancing(RobotContainer.drivetrain)),
        
        Map.entry("AntiSlip", new AntiSlip(RobotContainer.drivetrain))

    ));

    private static final SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
        RobotContainer.drivetrain::getPose, 
        RobotContainer.drivetrain::resetOdometry,
        AutonConstants.AUTO_TRANSLATION_PID,
        AutonConstants.AUTO_ROTATION_PID, 
        RobotContainer.drivetrain::setChassisSpeeds, 
        eventMap, 
        false, 
        RobotContainer.drivetrain);



    public static Command testAutoBlue() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestBlueEventMap", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command bluePreloadCommunityBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BluePreloadCommunityBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command bluePreloadPickupScoreBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BluePreloadPickUpScoreBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command blueMiddlePreloadPickupBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BlueMiddlePreloadPickupBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command blueNoElevatorPreloadPickupScoreBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("BlueNoElevatorPreloadPickupScoreBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command redPreloadCommunityBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("RedPreloadCommunityBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command redPreloadPickupScoreBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("RedPreloadPickUpScoreBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command redMiddlePreloadPickupBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("RedMiddlePreloadPickupBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command redNoElevatorPreloadPickupScoreBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("RedNoElevatorPreloadPickupScoreBalance", GENERIC_PATH_CONSTRAINTS));
    }





    public AutonMaster() {
        throw new UnsupportedOperationException("Master Auton Class!");
    }


}

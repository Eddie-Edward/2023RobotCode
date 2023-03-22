package frc.robot.autos;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.intake.IntakeConfig.PivotState;
import frc.robot.intake.commands.IntakeCommands;
import frc.robot.intake.commands.RunIntake;
import frc.robot.intake.commands.SetPivotState;
import frc.robot.autos.AutoBalancing;
import frc.robot.claw.ClawConfig.ClawState;
import frc.robot.claw.commands.ClawCommands;
import frc.robot.elevator.ElevatorConfig.ElevatorPosition;
import frc.robot.elevator.commands.SetElevatorState;

public final class AutonMaster {

    private static final PathConstraints GENERIC_PATH_CONSTRAINTS = new PathConstraints(4, 3);
    
    public static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        Map.entry("Stop", new InstantCommand(RobotContainer.drivetrain::stopSwerve)),
        
        Map.entry("ZeroGyro", new InstantCommand(() -> RobotContainer.drivetrain.m_pigeonGyro.setYaw(RobotContainer.drivetrain.getYaw().getDegrees()))),
        
        Map.entry("IntakeDown", 
        new ParallelCommandGroup(
            new SetPivotState(RobotContainer.intakePivot, PivotState.kDeployed), 
            new RunIntake(RobotContainer.intakeRoller, RunIntake.Mode.kCube))),

        
        Map.entry("IntakeUp", 
        new SetPivotState(RobotContainer.intakePivot, PivotState.kHumanPlayer)),

        Map.entry("ScoreHigh", 
        new SequentialCommandGroup(
            ClawCommands.setClawState(ClawState.kClosed),
            new SetElevatorState(RobotContainer.elevator, ElevatorPosition.kHigh)
        )),

        Map.entry("ScoreMid",
            new SequentialCommandGroup(
                ClawCommands.setClawState(ClawState.kClosed),
                new SetElevatorState(RobotContainer.elevator, ElevatorPosition.kMid)
            )
        ),
        
        Map.entry("ElevatorDown", new SetElevatorState(RobotContainer.elevator, ElevatorPosition.kZero)),

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

    public static Command preloadCommunityBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("PreloadCommunityBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command preloadScoreBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("PreloadScoreBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command preloadPickupBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("PreloadPickUpBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command middlePreloadPickupBalance() { 
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("MiddlePreloadPickupBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public static Command noElevatorPreloadPickupScoreBalance() {
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("NoElevatorPreloadPickupScoreBalance", GENERIC_PATH_CONSTRAINTS));
    }

    public AutonMaster() {
        throw new UnsupportedOperationException("Master Auton Class!");
    }


}

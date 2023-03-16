package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.drivetrain.SwerveDrivetrain;

public class TwoPieceOutsideBalance extends SequentialCommandGroup {
    SwerveDrivetrain mSwerveDrivetrain;
    PathPlannerTrajectory twoPieceOutsideBalanceTraj;
    public TwoPieceOutsideBalance(SwerveDrivetrain drivetrain) {
        mSwerveDrivetrain = drivetrain;

        twoPieceOutsideBalanceTraj = PathPlanner.loadPath("2PieceBalanceOUTSIDE", AutonConstants.kMaxSpeedMetersPerSecond, AutonConstants.kMaxAccelerationMetersPerSecondSquared);

        var translationController = new PIDController(AutonConstants.kPXController, 0, 0);
        var strafeController = new PIDController(AutonConstants.kPYController, 0, 0);
        var thetaController = new ProfiledPIDController(AutonConstants.kPThetaController, 0, AutonConstants.kDThetaController,
                        AutonConstants.kThetaControllerConstraints);
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand mAutonCommand = 
        new SwerveControllerCommand(
            twoPieceOutsideBalanceTraj, 
            mSwerveDrivetrain::getPose, 
            SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS,
            translationController, 
            strafeController,
            thetaController, 
            mSwerveDrivetrain::setModuleStates,
            mSwerveDrivetrain);

        addCommands(new InstantCommand(
            () -> mSwerveDrivetrain.resetOdometry(twoPieceOutsideBalanceTraj.getInitialHolonomicPose())),
            mAutonCommand
        );

    }
}

package frc.robot.autos;


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

public class TwoPieceInsideBalance extends SequentialCommandGroup{
    SwerveDrivetrain mSwerveDrivetrain;
    PathPlannerTrajectory twoPieceInsideBalanceTraj;
    public TwoPieceInsideBalance(SwerveDrivetrain drivetrain) {
        mSwerveDrivetrain = drivetrain;

        twoPieceInsideBalanceTraj = PathPlanner.loadPath("2PieceBalanceINSIDE", AutonConstants.kMaxSpeedMetersPerSecond, AutonConstants.kMaxAccelerationMetersPerSecondSquared);

        var translationController = new PIDController(AutonConstants.kPXController, 0, 0);
        var strafeController = new PIDController(AutonConstants.kPYController, 0, 0);
        var thetaController = new ProfiledPIDController(AutonConstants.kPThetaController, 0, AutonConstants.kDThetaController,
                        AutonConstants.kThetaControllerConstraints);
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand mAutonCommand = 
        new SwerveControllerCommand(
            twoPieceInsideBalanceTraj, 
            mSwerveDrivetrain::getPose, 
            SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS,
            translationController, 
            strafeController,
            thetaController, 
            mSwerveDrivetrain::setModuleStates,
            mSwerveDrivetrain);

        addCommands(new InstantCommand(
            () -> mSwerveDrivetrain.resetOdometry(twoPieceInsideBalanceTraj.getInitialHolonomicPose())),
            mAutonCommand
        );

    }
}

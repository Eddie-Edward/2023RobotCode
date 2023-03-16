package frc.robot.commands.autos;


import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveModes;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.drivetrain.SwerveDrivetrain;

public class OnePieceInside extends SequentialCommandGroup {
    SwerveDrivetrain mSwerveDrivetrain; 
    PathPlannerTrajectory onePieceInsideTraj;

    public OnePieceInside(SwerveDrivetrain drivetrain) {
        mSwerveDrivetrain = drivetrain;
        onePieceInsideTraj = PathPlanner.loadPath("1PieceInsidePlacePickBalance", AutonConstants.kMaxSpeedMetersPerSecond, AutonConstants.kMaxAccelerationMetersPerSecondSquared);

        var translationController = new PIDController(AutonConstants.kPXController, 0, 0);
        var strafeController = new PIDController(AutonConstants.kPYController, 0, 0);
        var thetaController = new PIDController(AutonConstants.kPThetaController, 0, AutonConstants.kDThetaController);
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand mAutonCommand = 
        new PPSwerveControllerCommand(
            onePieceInsideTraj, 
            mSwerveDrivetrain::getPose, 
            SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS,
            translationController, 
            strafeController,
            thetaController, 
            mSwerveDrivetrain::setModuleStates,
            false,
            mSwerveDrivetrain);
    
        

    
       addCommands(
        new InstantCommand(
            () -> mSwerveDrivetrain.resetOdometry(onePieceInsideTraj.getInitialHolonomicPose())),
            mAutonCommand
        );

    }
}

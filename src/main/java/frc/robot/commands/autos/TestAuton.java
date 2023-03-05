package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.commands.AntiSlip;
import frc.robot.commands.AutoBalancing;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class TestAuton extends SequentialCommandGroup {
    SwerveDrivetrain mSwerveDrivetrain;
    PathPlannerTrajectory testAutonTraj;

    AntiSlip antiSlip;
    AutoBalancing mAutoBalance;
    public TestAuton(SwerveDrivetrain drivetrain) {
        mSwerveDrivetrain = drivetrain;

        antiSlip = new AntiSlip(mSwerveDrivetrain);
        mAutoBalance = new AutoBalancing(mSwerveDrivetrain);
        testAutonTraj = PathPlanner.loadPath("Test", AutonConstants.kMaxSpeedMetersPerSecond, AutonConstants.kMaxAccelerationMetersPerSecondSquared);

        var translationController = new PIDController(AutonConstants.kPXController, 0, AutonConstants.kDXController);
        var strafeController = new PIDController(AutonConstants.kPYController, 0, AutonConstants.kDYController);
        var thetaController = new PIDController(AutonConstants.kPThetaController, 0, AutonConstants.kDThetaController);
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI); 

        PPSwerveControllerCommand mAutonCommand = 
        new PPSwerveControllerCommand(
            testAutonTraj, 
            mSwerveDrivetrain::getPose, 
            SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS,
            translationController, 
            strafeController,
            thetaController, 
            mSwerveDrivetrain::setModuleStates,
            false,
            mSwerveDrivetrain);

        addCommands(new InstantCommand(
            () -> mSwerveDrivetrain.resetOdometry(testAutonTraj.getInitialHolonomicPose())),
            mAutonCommand, 
            mAutoBalance,
            antiSlip
        );
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoBalancing;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.autos.AutonMaster;
import frc.robot.commands.autos.OnePieceInside;
import frc.robot.commands.autos.TestAuton;
import frc.robot.commands.autos.TwoPieceInsideBalance;
import frc.robot.commands.autos.TwoPieceOutsideBalance;
import frc.robot.drivetrain.SwerveDrivetrain;
import frc.robot.vision.PoseEstimator;


public class RobotContainer {

  /*Declare External Sensors -> Cameras */
  public final PhotonCamera m_OrangePI = new PhotonCamera(VisionConstants.CAMERA_NAME);
  /*Declare Joystick*/
  public final XboxController m_driverController = new XboxController(JoystickConstants.DRIVER_PORT_ID);
  
  /*Declare Subsystems*/
  public static final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();
  public final PoseEstimator m_poseEstimator = new PoseEstimator(m_OrangePI, m_swerveDrivetrain);

  /*Sendable Chooser Selector for Auton */
  private final SendableChooser<Command> mAutonChooser = new SendableChooser<>();

  
  
  /*Declare Auton Commands */
  private final TwoPieceInsideBalance mTwoPieceInsideBalance = new TwoPieceInsideBalance(m_swerveDrivetrain);
  private final TwoPieceOutsideBalance mTwoPieceOutsideBalance = new TwoPieceOutsideBalance(m_swerveDrivetrain);
  private final TestAuton mTestAuton = new TestAuton(m_swerveDrivetrain);
  private final OnePieceInside mOnePieceInside = new OnePieceInside(m_swerveDrivetrain);
  private final AutoBalancing mAutoBalancing = new AutoBalancing(m_swerveDrivetrain);
  
  /*Map Joystick Axis and Functions*/
  private final int m_translationAxis = XboxController.Axis.kLeftY.value;
  private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
  private final int m_rotationAxis = XboxController.Axis.kRightX.value;
  

  /*Map Joystick Buttons and Functions*/
  private final JoystickButton m_zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton m_resetRobotFieldPose = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private final JoystickButton m_zeroModules = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton m_autoBalance = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  
  public RobotContainer() {
    
    m_swerveDrivetrain.setDefaultCommand(new TeleopDrive(m_swerveDrivetrain, 
      m_driverController, m_translationAxis, m_strafeAxis, m_rotationAxis, 
      SwerveDrivetrainConstants.FIELD_RELATIVE, SwerveDrivetrainConstants.OPEN_LOOP));

    mAutonChooser.setDefaultOption("TwoPieceINSIDEBalance", mTwoPieceInsideBalance);
    mAutonChooser.addOption("TwoPieceOUTSIDEBalance", mTwoPieceOutsideBalance);
    mAutonChooser.addOption("TestAuton", mTestAuton);
    mAutonChooser.addOption("OnePieceInside", mOnePieceInside);
    mAutonChooser.addOption("TestNewMethod", AutonMaster.testAuto());
    SmartDashboard.putData(mAutonChooser);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@lin k edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /*Define onPressed Commands for Joystick Buttons and Triggers*/

    m_zeroGyro.onTrue(
      new InstantCommand(() -> m_swerveDrivetrain.zeroGyro()));
      
    m_resetRobotFieldPose.onTrue(
      new InstantCommand(() -> m_poseEstimator.resetFieldPosition()));

    m_zeroModules.onTrue( 
      new InstantCommand(() -> m_swerveDrivetrain.zeroModules())
    );
    
    m_autoBalance.whileTrue(mAutoBalancing);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*Returns Auton Commands (Sendable Chooser) */
    return mTestAuton;
  }
}

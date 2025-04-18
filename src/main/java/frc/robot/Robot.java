// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  private Timer disabledTimer;



  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }


  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }
    public SwerveSubsystem getDrivebase() {
    return m_robotContainer.drivebase;
}

  public VisionSim getVisionSim() {
    return m_robotContainer.visionSim;
}
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    getDrivebase().configureForAlliance();
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    getDrivebase().configureForAlliance();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
     
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {  

    // if (Robot.isSimulation()) {
    //   var visionSim = new VisionSystemSim("test");
      
    //   // Updated AprilTag loading
    //   visionSim.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
      
    //   var camera = new PhotonCamera("testCam");
      
    //   // Configure camera properties properly
    //   var cameraProp = new SimCameraProperties();
    //   cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(90));
    //   cameraProp.setFPS(30);
      
    //   var cameraSim = new PhotonCameraSim(camera, cameraProp);
    //   visionSim.addCamera(cameraSim, new Transform3d());
      
    //   // Enable streams
    //   cameraSim.enableRawStream(true);
    //   cameraSim.enableProcessedStream(true);
    //   cameraSim.enableDrawWireframe(true);
      
    //   // Force initial update
    //   visionSim.update(new Pose2d(1.5, 1.5, new Rotation2d()));
      
    //   // Add to dashboard
    //   SmartDashboard.putData("Vision Field", visionSim.getDebugField());
    

    }

  


  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  { // Safe null-checked access
   
    if (m_robotContainer != null && 
        getDrivebase() != null && 
        getVisionSim() != null) {
        
        Pose2d currentPose = getDrivebase().getPose();
        if (currentPose != null) {
            getVisionSim().simulationPeriodic(currentPose);
        }
    }
  }
}

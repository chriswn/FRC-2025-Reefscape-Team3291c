// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ChaseTag2;
import frc.robot.commands.ChaseTagCommand;
// import frc.robot.commands.RunMotorCommand;
// import frc.robot.subsystems.RunMotorSub;
import frc.robot.commands.ElevatorCMDs.GoToFloor;
import frc.robot.commands.ElevatorCMDs.GoToGround;
import frc.robot.commands.ElevatorCMDs.GoToTop;
import frc.robot.commands.ElevatorCMDs.ResetElevatorEncoder;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;
import frc.robot.commands.IntakeMotorCMDs.IntakeCMD;
import frc.robot.commands.IntakePivotCMDs.PivotToGround;
import frc.robot.commands.IntakePivotCMDs.PivotToStow;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ReefMap;
import frc.robot.subsystems.ScoringTargetManager;
import frc.robot.subsystems.VisionSubsystem;
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.ScoringTargetManager;
import frc.robot.subsystems.ScoringTarget;

import frc.robot.commands.ScoreTargetSequence;
import frc.robot.commands.ElevatorCMDs.SetElevatorLevelCommand;


import frc.robot.subsystems.ScoringTargetManager;
import frc.robot.subsystems.ScoringTarget;
import frc.robot.VisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  public CommandJoystick controller1 = new CommandJoystick(1);
  //public VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final SendableChooser<Command> autoChooser;
  // public RunMotorSub runMotorSub = new RunMotorSub();
  // public ColorChanger colorChanger = new ColorChanger();
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
  private final IntakeMotorSubsystem intakeMotorSubsystem = new IntakeMotorSubsystem();
  private final Command elevatorGoToTop = new GoToTop(elevatorSubsystem);
  private final Command elevatorGoToGround = new GoToGround(elevatorSubsystem);
  private final Command ResetElevatorEncoder = new ResetElevatorEncoder(elevatorSubsystem);
  private final Command pivotToGround = new PivotToGround(intakePivotSubsystem);
  private final Command pivotToStow = new PivotToStow(intakePivotSubsystem);
  private final Command eSpitCMD = new ESpitCMD(intakeMotorSubsystem);
  private final Command intakeCMD = new IntakeCMD(intakeMotorSubsystem);
  private final GoToFloor goToFloor = new GoToFloor(elevatorSubsystem, intakePivotSubsystem, () -> controller1.povUp().getAsBoolean(), () -> controller1.pov(180).getAsBoolean(), () -> controller1.button(Constants.ButtonList.start).getAsBoolean(), () -> controller1.button(Constants.ButtonList.a).getAsBoolean());
  private final PhotonCamera camera = new PhotonCamera("cam_in");
  public final VisionSim visionSim = new VisionSim(camera);
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
private final ScoringTargetManager scoringTargetManager = new ScoringTargetManager();
private final ReefMap reefMap = new ReefMap();

  private final Command autoAlignCommand = new AutoAlignCommand(visionSubsystem, drivebase, Constants.Vision.TARGET_TAG_ID);

  //     private final RunMotorCommand runMotorCommand = new RunMotorCommand(
//         runMotorSub,
//         () -> 2 // Example: Getting speed from joystick Y-axis
// );

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * 1,
      () -> driverXbox.getLeftX() * 1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)// Decideds robot (speed)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverXbox.getRawAxis(4),
  driverXbox::getRightY)
  .headingWhile(true);
  // SwerveInputStream driveDirectAngle =
  // driveAngularVelocity.copy().withControllerHeadingAxis(() ->
  // driverJoystick.getRawAxis(4),
  // () -> driverJoystick.getRawAxis(5))
  // .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> -driverXbox.getLeftY(),
  () -> -driverXbox.getLeftX())
  .withControllerRotationAxis(() -> driverXbox.getRawAxis(
      2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // SwerveInputStream driveAngularVelocityKeyboard =
  // SwerveInputStream.of(drivebase.getSwerveDrive(),
  // () -> driverJoystick.getRawAxis(1),
  // () -> driverJoystick.getRawAxis(0))
  // .withControllerRotationAxis(() -> driverJoystick.getRawAxis(4))
  // .deadband(OperatorConstants.DEADBAND)
  // .scaleTranslation(0.8)
  // .allianceRelativeControl(true);
  //Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true);

  // SwerveInputStream driveDirectAngleKeyboard =
  // driveAngularVelocityKeyboard.copy()
  // .withControllerHeadingAxis(() ->
  // Math.sin(
  // driverJoystick.getRawAxis(4) *
  // Math.PI) *
  // (Math.PI *
  // 2),
  // () ->
  // Math.cos(
  // driverJoystick.getRawAxis(4) *
  // Math.PI) *
  // (Math.PI *
  // 2))
  // .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // ChaseTagCommand chaseCommand = new ChaseTagCommand(visionSubsystem.getCamera(), drivebase);
    ChaseTag2 ChaseTag2 = new ChaseTag2(visionSubsystem, drivebase);
    scoringTargetManager.reset(); // Reset on init


    // In your robot container initialization
// if (Robot.isSimulation()) {
//  SmartDashboard.putData("Vision Sim Field", visionSim.getSimDebugField());
//  // In RobotContainer.java
// ChaseTagCommand chaseCommand = new ChaseTagCommand(visionSim.getCamera(), drivebase);
// driverXbox.a().whileTrue(chaseCommand);
// }


if (RobotBase.isSimulation()) {
  SmartDashboard.putData("Vision Sim Field", visionSim.getSimDebugField());
  drivebase.configureForAlliance();

 }
 

        DriverStation.silenceJoystickConnectionWarning(true);
        
    // Configure the trigger bindings
    configureBindings();
    driverXbox.y().whileTrue(new AutoAlignCommand(visionSubsystem, drivebase, Constants.Vision.TARGET_TAG_ID));
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("goToGroundFloor", new GoToFloor(elevatorSubsystem, intakePivotSubsystem, () -> controller1.povUp().getAsBoolean(), () -> controller1.povDown().getAsBoolean(), () -> controller1.button(Constants.ButtonList.start).getAsBoolean(), () -> controller1.button(Constants.ButtonList.a).getAsBoolean(), 0).until(() -> elevatorSubsystem.ifAtFloor(Elevator.groundFloor)));
    NamedCommands.registerCommand("goToSecondFloor", new GoToFloor(elevatorSubsystem, intakePivotSubsystem, () -> controller1.povUp().getAsBoolean(), () -> controller1.povDown().getAsBoolean(),() -> controller1.button(Constants.ButtonList.start).getAsBoolean(), () -> controller1.button(Constants.ButtonList.a).getAsBoolean(), 1).until(() -> elevatorSubsystem.ifAtFloor(Elevator.secondFloor)));
    NamedCommands.registerCommand("goToThirdFloor", new GoToFloor(elevatorSubsystem, intakePivotSubsystem, () -> controller1.povUp().getAsBoolean(), () -> controller1.povDown().getAsBoolean(),() -> controller1.button(Constants.ButtonList.start).getAsBoolean(), () -> controller1.button(Constants.ButtonList.a).getAsBoolean(), 2).until(() -> elevatorSubsystem.ifAtFloor(Elevator.thirdFloor)));
    NamedCommands.registerCommand("goToFourthFloor", new GoToFloor(elevatorSubsystem, intakePivotSubsystem, () -> controller1.povUp().getAsBoolean(), () -> controller1.povDown().getAsBoolean(),() -> controller1.button(Constants.ButtonList.start).getAsBoolean(), () -> controller1.button(Constants.ButtonList.a).getAsBoolean(), 3).until(() -> elevatorSubsystem.ifAtFloor(Elevator.fourthFloor)));
    NamedCommands.registerCommand("intakeCMD", intakeCMD);
    NamedCommands.registerCommand("eSpitCMD", eSpitCMD);
    // driverXbox.a().whileTrue(chaseCommand);
    driverXbox.a().whileTrue(ChaseTag2);
    driverXbox.a().whileTrue(ChaseTag2);

    driverXbox.a().onTrue(
      // Commands.runOnce(() ->
      //   scoringTargetManager.callTarget(scoringTargetManager.getNextTarget())  // getNextTarget(), not getNext()
      // ).andThen(
      //   new ScoreTargetSequence(
      //     visionSubsystem,
      //     drivebase,
      //     elevatorSubsystem,
      //     intakeMotorSubsystem,
      //     scoringTargetManager
      //   )
      // )


        // 1) pick & call the next ScoringTargetManager target
  Commands.runOnce(() -> {
    // ask ReefMap for [face,level]
    int[] next = reefMap.getNextTarget();
    if (next != null) {
      // convert to ScoringTarget and store in manager
      ScoringTarget tgt = new ScoringTarget(next[0], next[1] + 1);
      scoringTargetManager.callTarget(tgt);
    }
  })
  // 2) then run the actual scoring sequence
  .andThen(new ScoreTargetSequence(
    visionSubsystem,
    drivebase,
    elevatorSubsystem,
    intakeMotorSubsystem,
    scoringTargetManager
  ))
  // 3) then mark that face/level as scored in the ReefMap
  .andThen(Commands.runOnce(() -> {
    scoringTargetManager.getCurrentTarget().ifPresent(t -> {
      // note: t.getLevel() returns 1–3, ReefMap.Level.L1 = ordinal(0)
      ReefMap.Level levelEnum = ReefMap.Level.values()[t.getLevel() - 1];
      reefMap.markScored(t.getFace(), levelEnum);
    });
  }))
    );

    //NamedCommands.registerCommand("RunMotor", new RunMotorCommand(runMotorSub, () -> 2).withTimeout(5));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //controller0.button(2).whileTrue(runMotorCommand);
   
      
  
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    // if (RobotBase.isSimulation()) {
    //   drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    // } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // }

    // if (Robot.isSimulation()) {
    //   driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    //   driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    // }

    if (DriverStation.isTest()) {
      // drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      // driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.leftBumper().onTrue(Commands.none());
      // driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      controller1.button(Constants.ButtonList.r3).whileTrue(ResetElevatorEncoder);//should be commented out after testing
      driverXbox.x().whileTrue(pivotToGround);
      driverXbox.b().whileTrue(pivotToStow);
      controller1.povLeft().toggleOnTrue(eSpitCMD);
      controller1.povRight().toggleOnTrue(intakeCMD);
      elevatorSubsystem.setDefaultCommand(goToFloor);
      intakePivotSubsystem.setDefaultCommand(goToFloor);
      // ChaseTagCommand chaseCommand = new ChaseTagCommand(visionSim.getCamera(), drivebase);

      // driverXbox.a().whileTrue(chaseCommand);

      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      // driverXbox.start).whileTrue(Commands.none());
      // driverXbox.back).whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
     // driverXbox.rb).onTrue(Commands.none());
    }

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("Simple Command");
    //return drivebase.getAutonomousCommand("Simple Auto");
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
} 
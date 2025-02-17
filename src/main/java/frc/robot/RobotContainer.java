// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RunMotorCommand;
import frc.robot.subsystems.RunMotorSub;
import frc.robot.commands.ElevatorCMDs.GoToFloor;
import frc.robot.commands.ElevatorCMDs.GoToGround;
import frc.robot.commands.ElevatorCMDs.GoToTop;
import frc.robot.commands.ElevatorCMDs.ResetElevatorEncoder;
import frc.robot.commands.IntakeMotorCMDs.ESpit;
import frc.robot.commands.IntakeMotorCMDs.IntakeCMD;
import frc.robot.commands.IntakePivotCMDs.PivotToGround;
import frc.robot.commands.IntakePivotCMDs.PivotToStow;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

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
  //final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandJoystick controller0 = new CommandJoystick(0);
  public VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final SendableChooser<Command> autoChooser;
  public RunMotorSub runMotorSub = new RunMotorSub();
  // public ColorChanger colorChanger = new ColorChanger();
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve3291"));

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
  private final IntakeMotorSubsystem intakeMotorSubsystem = new IntakeMotorSubsystem();
  private final Command elevatorGoToTop = new GoToTop(elevatorSubsystem);
  private final Command elevatorGoToGround = new GoToGround(elevatorSubsystem);
  private final Command ResetElevatorEncoder = new ResetElevatorEncoder(elevatorSubsystem);
  private final Command pivotToGround = new PivotToGround(intakePivotSubsystem);
  private final Command pivotToStow = new PivotToStow(intakePivotSubsystem);
  private final Command ejectCMD = new ESpit(intakeMotorSubsystem);
  private final Command intakeCMD = new IntakeCMD(intakeMotorSubsystem);
  private final GoToFloor goToFloor = new GoToFloor(elevatorSubsystem, intakePivotSubsystem, () -> controller0.povUp().getAsBoolean(), () -> controller0.pov(180).getAsBoolean());

    private final RunMotorCommand runMotorCommand = new RunMotorCommand(
        runMotorSub,
        () -> 2 // Example: Getting speed from joystick Y-axis
);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> controller0.getRawAxis(1) * -1,
      () -> controller0.getRawAxis(0) * -1)
      .withControllerRotationAxis(() -> controller0.getRawAxis(4))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> controller0.getRawAxis(4),
  () -> controller0.getRawAxis(5))
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
      () -> -controller0.getRawAxis(1),
      () -> -controller0.getRawAxis(0))
      .withControllerRotationAxis(() -> controller0.getRawAxis(
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
          controller0.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              controller0.getRawAxis(
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
    // Configure the trigger bindings


    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("goToThirdFloor", new GoToFloor(elevatorSubsystem, intakePivotSubsystem, () -> controller0.povUp().getAsBoolean(), () -> controller0.povDown().getAsBoolean(), 2).until(() -> elevatorSubsystem.ifAtFloor(Elevator.thirdFloor)));
    //NamedCommands.registerCommand("RunMotor", new RunMotorCommand(runMotorSub, () -> 2).withTimeout(5));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    controller0.button(Constants.ButtonList.b).whileTrue(runMotorCommand);
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

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      controller0.button(Constants.ButtonList.start).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      controller0.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }

    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      controller0.button(Constants.ButtonList.x).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      controller0.button(Constants.ButtonList.y).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      controller0.button(Constants.ButtonList.start).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      controller0.button(Constants.ButtonList.back).whileTrue(drivebase.centerModulesCommand());
    } else {
      // controller0.pov(0).whileTrue(elevatorGoToGround);
      controller0.button(Constants.ButtonList.l3).whileTrue(new GoToFloor(elevatorSubsystem, intakePivotSubsystem, () -> controller0.povUp().getAsBoolean(), () -> controller0.povDown().getAsBoolean(), 2));
      controller0.button(Constants.ButtonList.a).whileTrue(ResetElevatorEncoder);
      controller0.button(Constants.ButtonList.x).whileTrue(pivotToGround);
      controller0.button(Constants.ButtonList.b).whileTrue(pivotToStow);
      controller0.povLeft().toggleOnTrue(ejectCMD);
      controller0.povRight().toggleOnTrue(intakeCMD);
      //elevatorSubsystem.setDefaultCommand(goToFloor);
      intakePivotSubsystem.setDefaultCommand(goToFloor);
      controller0.button(Constants.ButtonList.a).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      controller0.button(Constants.ButtonList.x).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      controller0.button(Constants.ButtonList.b).whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      controller0.button(Constants.ButtonList.start).whileTrue(Commands.none());
      controller0.button(Constants.ButtonList.back).whileTrue(Commands.none());
      controller0.button(Constants.ButtonList.lb).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      controller0.button(Constants.ButtonList.rb).onTrue(Commands.none());
    }

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("Simple Command");
    return drivebase.getAutonomousCommand("Simple Auto");
    //return autoChooser.getSelected();
  }

  // public void setMotorBrake(boolean brake) {
  //   drivebase.setMotorBrake(brake);
  // }
}
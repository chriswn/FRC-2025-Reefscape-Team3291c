// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Swerve;
import frc.robot.commands.ClimbCMD;
import frc.robot.commands.ColorChangingCMD;
import frc.robot.commands.DriveToApriltag;
import frc.robot.commands.DriveToApriltagAndShoot;
import frc.robot.commands.FeedWheelCMD;
import frc.robot.commands.LaunchNoteCMD;
import frc.robot.commands.LaunchWheelCMD;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.Auto.MN_MildAuto;
import frc.robot.commands.IntakeCMDS.IntakeMotor.EjectCMD;
import frc.robot.commands.IntakeCMDS.IntakeMotor.IntakeMotorCMD;
import frc.robot.commands.IntakeCMDS.PivotMotor.AmpCMD;
import frc.robot.commands.IntakeCMDS.PivotMotor.GroundCMD;
import frc.robot.commands.IntakeCMDS.PivotMotor.SourceCMD;
import frc.robot.commands.IntakeCMDS.PivotMotor.StowCMD;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSub;
import frc.robot.subsystems.PreferencesSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorChanger;





/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


private final SendableChooser<Command> autoChooser;
  //controllers
  public CommandJoystick controller0 = new CommandJoystick(0);
  public CommandJoystick controller1 = new CommandJoystick(1);


  //buttons
  public final JoystickButton backToggleButton = new JoystickButton(controller0.getHID(), Constants.buttonList.back);
  public final JoystickButton aToggleButton = new JoystickButton(controller1.getHID(), Constants.buttonList.a);
  public final JoystickButton colorToggleButton = new JoystickButton(controller1.getHID(), Constants.buttonList.l3);
  public final JoystickButton robotCentricButton = new JoystickButton(controller0.getHID(), Constants.buttonList.r3);


  //subsystems
  public PreferencesSubsystem preferencesSubsystem = new PreferencesSubsystem();
  public VisionSubsystem visionSubsystem = new VisionSubsystem();
  public ColorChanger colorChanger = new ColorChanger();
  public LauncherSub launcherSub = new LauncherSub();
  public ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public IntakeMotorSubsystem intakeMotorSubsystem = new IntakeMotorSubsystem();
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem(colorChanger, preferencesSubsystem);//below colorChanger


  //commands
 // public SysIdDynamic sysIdDynamic = new SysIdDynamic();
  public ClimbCMD climbCMD = new ClimbCMD(
        climberSubsystem,
        colorChanger,
        () -> controller1.getRawAxis(2),
        () -> controller1.getRawAxis(3),
        () -> aToggleButton.getAsBoolean(),
        () -> colorToggleButton.getAsBoolean()
      );

  //launcher
  public FeedWheelCMD feedWheelCMD = new FeedWheelCMD(launcherSub);
  public LaunchWheelCMD launchWheelCMD = new LaunchWheelCMD(launcherSub, preferencesSubsystem);
  //pivot motor
  public StowCMD stowCMD = new StowCMD(intakeSubsystem);
  public AmpCMD ampCMD = new AmpCMD(intakeSubsystem);
  public GroundCMD groundCMD = new GroundCMD(intakeSubsystem);
  public SourceCMD sourceCMD = new SourceCMD(intakeSubsystem);
  //intake motor
  public EjectCMD ejectCMD = new EjectCMD(intakeMotorSubsystem, preferencesSubsystem);
  public IntakeMotorCMD intakeMotorCMD = new IntakeMotorCMD(intakeMotorSubsystem, intakeSubsystem, colorChanger, preferencesSubsystem);
  //intake motor + launcher
  public LaunchNoteCMD launchNoteCMD = new LaunchNoteCMD(intakeMotorSubsystem, launcherSub, preferencesSubsystem);
  //Vision
  public DriveToApriltagAndShoot driveToApriltagAndShoot = new DriveToApriltagAndShoot(swerveSubsystem, visionSubsystem, intakeMotorSubsystem, launcherSub, 0, preferencesSubsystem);
  public DriveToApriltag driveToApriltag = new DriveToApriltag(swerveSubsystem, visionSubsystem);
  
 






  //subsystems\\


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
 // Subsystem initialization
    
        // Register Named Commands
        NamedCommands.registerCommand("EjectCMD", new EjectCMD(intakeMotorSubsystem, preferencesSubsystem).withTimeout(1));
        NamedCommands.registerCommand("IntakeMotorCMD", new IntakeMotorCMD(intakeMotorSubsystem, intakeSubsystem, colorChanger, preferencesSubsystem).withTimeout(5));
        NamedCommands.registerCommand("LaunchWheelCMD", new LaunchWheelCMD(launcherSub, preferencesSubsystem).withTimeout(1));
        NamedCommands.registerCommand("FeedWheelCMD", new FeedWheelCMD(launcherSub).withTimeout(1));
        NamedCommands.registerCommand("AmpCMD", new AmpCMD(intakeSubsystem).until(intakeSubsystem::ampAtAngle));
        NamedCommands.registerCommand("SourceCMD", new SourceCMD(intakeSubsystem).until(intakeSubsystem::sourceAtAngle));
        NamedCommands.registerCommand("GroundCMD", new GroundCMD(intakeSubsystem).until(intakeSubsystem::groundAtAngle));
        NamedCommands.registerCommand("StowCMD", new StowCMD(intakeSubsystem).until(intakeSubsystem::stowAtAngle));
        NamedCommands.registerCommand("ColorChangingCMD", new ColorChangingCMD(colorChanger));
        NamedCommands.registerCommand("DriveToApriltag", new DriveToApriltag(swerveSubsystem, visionSubsystem));
        NamedCommands.registerCommand("LaunchNoteCMD", new LaunchNoteCMD(intakeMotorSubsystem, launcherSub, preferencesSubsystem).withTimeout(2));

    configureBindings();

    //Controller 0
    //controller0.button(Constants.buttonList.b).whileTrue(launchWheelCMD);
    controller0.button(Constants.buttonList.b).whileTrue(launchWheelCMD);
    controller0.button(Constants.buttonList.x).whileTrue(feedWheelCMD);
  
    controller0.button(Constants.buttonList.rb).whileTrue(ejectCMD);
    controller0.button(Constants.buttonList.lb).whileTrue(intakeMotorCMD);

    controller0.povDown().whileTrue(groundCMD);
    controller0.povUp().whileTrue(stowCMD);
    controller0.povLeft().whileTrue(sourceCMD);
    controller0.povRight().whileTrue(ampCMD);

    controller0.button(Constants.buttonList.start).toggleOnTrue(climbCMD);
    controller0.button(Constants.buttonList.y).toggleOnTrue(launchNoteCMD);

    //Controller1
    controller1.button(Constants.buttonList.back).whileTrue(driveToApriltagAndShoot);
    controller1.button(Constants.buttonList.b).whileTrue(launchWheelCMD);
    controller1.button(Constants.buttonList.x).whileTrue(feedWheelCMD);
    
    controller1.povDown().whileTrue(groundCMD);
    controller1.povUp().whileTrue(stowCMD);
    controller1.povLeft().whileTrue(sourceCMD);
    controller1.povRight().whileTrue(ampCMD);
   
    controller1.button(Constants.buttonList.rb).whileTrue(ejectCMD);//This ejects notes
    controller1.button(Constants.buttonList.lb).whileTrue(intakeMotorCMD);//This takes notes in
    //controller1.button(Constants.buttonList.start).whileTrue(driveToApriltag);
    controller1.button(Constants.buttonList.start).toggleOnTrue(climbCMD);
    controller1.button(Constants.buttonList.y).toggleOnTrue(launchNoteCMD);
   
    //Autonomous
    autoChooser = AutoBuilder.buildAutoChooser();

    
  SmartDashboard.putData("AutoChooser", autoChooser);

  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    swerveSubsystem.setDefaultCommand(
      new SwerveDrive(
        swerveSubsystem,
        visionSubsystem,
        () -> controller0.getRawAxis(1),
        () -> -controller0.getRawAxis(0),
        () -> controller0.getRawAxis(4),
        () -> robotCentricButton.getAsBoolean(),
        () -> backToggleButton.getAsBoolean()
      )
    );

  }
                                                                                             
  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
   
    //return new PathPlannerAuto("Launch Auto");
    return autoChooser.getSelected();
    //return new MildAuto(swerveSubsystem);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}




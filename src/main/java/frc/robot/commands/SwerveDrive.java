// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SwerveDrive extends Command {
  private final VisionSubsystem visionSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier translationSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier robotCentricSupplier;
  private final BooleanSupplier backToggleSupplier;
  private final BooleanSupplier isAuto;
  private int backToggleInt;
  private double rotationVal;
  private double translationVal;
  private double visionTX;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(Swerve.kMaxTranslationAcceleration);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Swerve.kMaxStrafeAcceleration);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Swerve.kMaxRotationAcceleration);
  private double strafeVal;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(
    SwerveSubsystem swerveSubsystem,
    VisionSubsystem visionSubsystem,
    DoubleSupplier translationSupplier,
    DoubleSupplier strafeSupplier,
    DoubleSupplier rotationSupplier,
    BooleanSupplier robotCentricSupplier,
    BooleanSupplier backToggleSupplier,
    BooleanSupplier isAuto
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem, visionSubsystem);

    this.translationSupplier = translationSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    this.robotCentricSupplier = robotCentricSupplier;
    this.backToggleSupplier = backToggleSupplier;
    this.isAuto = isAuto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.resetToAbsolute();
    this.backToggleInt = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (backToggleSupplier.getAsBoolean() && this.backToggleInt == 0) {
      this.backToggleInt = 1;
  }

  if (this.backToggleInt == 1 && !backToggleSupplier.getAsBoolean()) {
      this.backToggleInt = 2;
    }

  if (this.backToggleInt == 2 && backToggleSupplier.getAsBoolean()) {
      this.backToggleInt = 3;
    }

  if (this.backToggleInt == 3 && !backToggleSupplier.getAsBoolean()) {
      this.backToggleInt = 0; 
    }

    SmartDashboard.putNumber("Translation Supplier", translationSupplier.getAsDouble());
    SmartDashboard.putNumber("Strafe Supplier", strafeSupplier.getAsDouble());
    SmartDashboard.putNumber("Rotation Supplier", rotationSupplier.getAsDouble());

    if (backToggleInt == 0) {
    this.rotationVal = rotationLimiter.calculate(
      MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Swerve.stickDeadband));
      this.translationVal = translationLimiter.calculate(
      MathUtil.applyDeadband(translationSupplier.getAsDouble(), Swerve.stickDeadband));
      this.strafeVal = strafeLimiter.calculate(
      MathUtil.applyDeadband(strafeSupplier.getAsDouble() * 1.25, Swerve.stickDeadband));
    }
    else if (backToggleInt == 2) {
      if (visionSubsystem.getTXSwerve() > Constants.Vision.XDeadband || visionSubsystem.getTXSwerve() < -Constants.Vision.XDeadband) {
        if (visionSubsystem.getTXSwerve() > Constants.Vision.XDeadband) {
        this.visionTX = Constants.Vision.XDeadband;
        }
          else if (visionSubsystem.getTXSwerve() < -Constants.Vision.XDeadband) {
            this.visionTX = -Constants.Vision.XDeadband;
          }
            else {
              this.visionTX = visionSubsystem.getTXSwerve() * Constants.Vision.XProportionalGain;
            }
      }
      else {
        this.visionTX = Constants.Vision.XOffset;
      }
      double visionOutput = (visionTX - Constants.Vision.XOffset)/Constants.Vision.XRange;
      this.rotationVal = rotationLimiter.calculate(
      MathUtil.applyDeadband(visionOutput, Swerve.stickDeadband));

      this.translationVal = translationLimiter.calculate(visionSubsystem.getDistanceToSpeaker());
      this.strafeVal = 0.0;
    }
    swerveSubsystem.drive(
      new Translation2d(-translationVal, strafeVal).times(Swerve.maxSpeed), 
      this.rotationVal * Swerve.maxAngularVelocity, 
      robotCentricSupplier.getAsBoolean(),
      true
    );

    SmartDashboard.putNumber("Translation Val", translationVal);
    SmartDashboard.putNumber("Strafe Val", strafeVal);
    SmartDashboard.putNumber("Rotation Val", this.rotationVal);
    SmartDashboard.putBoolean("robot centric", robotCentricSupplier.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isAuto.getAsBoolean() ) {
      return true;
    }
    return false;
  }
}

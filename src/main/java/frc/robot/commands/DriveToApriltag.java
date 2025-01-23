// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.Swerve;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// public class DriveToApriltag extends Command {
//   private final VisionSubsystem visionSubsystem;
//   private final SwerveSubsystem swerveSubsystem;
//   private double rotationVal;
//   private double translationVal;
//   private double strafeVal;
//   private double visionTX;

//   private SlewRateLimiter translationLimiter = new SlewRateLimiter(Swerve.kMaxTranslationAcceleration);
//   //private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Swerve.kMaxStrafeAcceleration);
//   private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Swerve.kMaxRotationAcceleration);

//   /** Creates a new SwerveDrive. */
//   public DriveToApriltag(
//     SwerveSubsystem swerveSubsystem,
//     VisionSubsystem visionSubsystem
//     //boolean targetLocation
//   ) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.visionSubsystem = visionSubsystem;
//     this.swerveSubsystem = swerveSubsystem;
//     addRequirements(swerveSubsystem, visionSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     swerveSubsystem.resetToAbsolute();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     int targetLocation;
//     if (visionSubsystem.isThereATarget() == true) {

//       if (visionSubsystem.getApriltagId() == 7 || visionSubsystem.getApriltagId() == 4 || visionSubsystem.getApriltagId() == 3 || visionSubsystem.getApriltagId() == 8){
//         targetLocation = 1;
//       } else if (visionSubsystem.getApriltagId() == 6 || visionSubsystem.getApriltagId() == 5){
//         targetLocation = 3;//change later if add amp
//       } else {
//         targetLocation = 3;
//       }

//       if (visionSubsystem.getTXSwerve() > Constants.Vision.XDeadband || visionSubsystem.getTXSwerve() < -Constants.Vision.XDeadband) {
//         if (visionSubsystem.getTXSwerve() > Constants.Vision.XRange) {
//           this.visionTX = Constants.Vision.XRange;
//         }
//         if (visionSubsystem.getTXSwerve() < -Constants.Vision.XRange) {
//           this.visionTX = -Constants.Vision.XRange;
//         }
//         else {
//          this.visionTX = visionSubsystem.getTXSwerve();
//         }
//     }
//       else {
//         this.visionTX = Constants.Vision.XOffset;
//       }

//       double visionXOutput = (visionTX - Constants.Vision.XOffset)/Constants.Vision.XRange;
//       this.rotationVal = rotationLimiter.calculate(visionXOutput);

//       if (targetLocation == 1) {
//         this.translationVal = translationLimiter.calculate(visionSubsystem.getDistanceToSpeaker());
//       } else if ( targetLocation == 2) {
//         this.translationVal = translationLimiter.calculate(visionSubsystem.getLimelightSpeed());
//       } else {
//         this.translationVal = 0;
//         this.rotationVal = rotationLimiter.calculate(Constants.Vision.findAprilTagTurnSpeed);
//       }

      
//       this.strafeVal = 0.0;

//     swerveSubsystem.drive(
//       new Translation2d(this.translationVal, this.strafeVal).times(Swerve.maxSpeed), 
//       this.rotationVal * Swerve.maxAngularVelocity, 
//       false,
//       false
//     );

//   } else {
//     swerveSubsystem.drive(
//       new Translation2d(0.0, 0.0).times(Swerve.maxSpeed), 
//       0.1 * Swerve.maxAngularVelocity, 
//       false,
//       false
//     );
//   }

//     SmartDashboard.putNumber("Translation Val", this.translationVal);
//     SmartDashboard.putNumber("Strafe Val", this.strafeVal);
//     SmartDashboard.putNumber("Rotation Val", this.rotationVal);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     double txTarget = MathUtil.applyDeadband(visionSubsystem.getTXSwerve(), Constants.Vision.XDeadband);
//     double tyTarget = MathUtil.applyDeadband(visionSubsystem.getTYSwerve(), Constants.Vision.XDeadband);

//     if (txTarget == Constants.Vision.XOffset && tyTarget == Constants.Vision.yTargetValue) {
//       return true;
//     }
//     return false;
//   }
// }

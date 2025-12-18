package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * Command to automatically align the robot to a specific AprilTag.
 * Uses ProfiledPIDControllers for smooth motion to the goal pose.
 */
public class AutoAlignCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final SwerveSubsystem drivebase;
    private final int targetTagId;
    private boolean isActive = false;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private Pose2d targetPose = new Pose2d();
    private Pose2d startingPose;
    private double startTime;
    private boolean hasValidTarget = false;
    private double tagYawRad = 0.0;  // remember tag orientation

    /**
     * Creates a new AutoAlignCommand.
     * 
     * @param visionSubsystem The vision subsystem for target detection
     * @param drivebase The swerve drive subsystem
     * @param targetTagId The AprilTag ID to align to
     */
    public AutoAlignCommand(VisionSubsystem visionSubsystem, SwerveSubsystem drivebase, int targetTagId) {
        this.visionSubsystem = visionSubsystem;
        this.drivebase = drivebase;
        this.targetTagId = targetTagId;

        // Initialize PID controllers with constants from VisionConstants
        this.xController = new ProfiledPIDController(
            VisionConstants.AUTO_ALIGN_TRANSLATION_KP, 
            0, 
            VisionConstants.AUTO_ALIGN_TRANSLATION_KD,
            new TrapezoidProfile.Constraints(
                VisionConstants.AUTO_ALIGN_MAX_VELOCITY,
                VisionConstants.AUTO_ALIGN_MAX_ACCELERATION
            )
        );
        
        this.yController = new ProfiledPIDController(
            VisionConstants.AUTO_ALIGN_TRANSLATION_KP, 
            0, 
            VisionConstants.AUTO_ALIGN_TRANSLATION_KD,
            new TrapezoidProfile.Constraints(
                VisionConstants.AUTO_ALIGN_MAX_VELOCITY,
                VisionConstants.AUTO_ALIGN_MAX_ACCELERATION
            )
        );
        
        this.thetaController = new ProfiledPIDController(
            VisionConstants.AUTO_ALIGN_ROTATION_KP, 
            0, 
            VisionConstants.AUTO_ALIGN_ROTATION_KD,
            new TrapezoidProfile.Constraints(
                VisionConstants.AUTO_ALIGN_MAX_ANGULAR_VELOCITY,
                VisionConstants.AUTO_ALIGN_MAX_ANGULAR_ACCELERATION
            )
        );

        // Set tolerances from constants
        xController.setTolerance(VisionConstants.AUTO_ALIGN_POSITION_TOLERANCE);
        yController.setTolerance(VisionConstants.AUTO_ALIGN_POSITION_TOLERANCE);
        thetaController.setTolerance(VisionConstants.AUTO_ALIGN_ROTATION_TOLERANCE);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        startTime      = Timer.getFPGATimestamp();
        startingPose   = drivebase.getPose();
        isActive       = true;
        hasValidTarget = false;

        xController.reset(startingPose.getX());
        yController.reset(startingPose.getY());
        thetaController.reset(startingPose.getRotation().getRadians());

        VisionConstants.APRILTAG_FIELD_LAYOUT
            .getTagPose(targetTagId)
            .ifPresent(tagPose3d -> {
                // Capture tag yaw for orientation
                tagYawRad = tagPose3d.getRotation().getZ();

                // Offset 2m in front of tag (adjust as needed for your robot)
                Transform3d tagToGoal = new Transform3d(
                    new Translation3d(2.0, 0.0, 0.0),
                    new Rotation3d()
                );
                Pose3d goal3d = tagPose3d.transformBy(tagToGoal);
                targetPose = goal3d.toPose2d();
                hasValidTarget = true;

                xController.setGoal(targetPose.getX());
                yController.setGoal(targetPose.getY());

                // Face opposite direction of tag
                double yawGoal = MathUtil.angleModulus(tagYawRad + Math.PI);
                thetaController.setGoal(yawGoal);
            });

        SmartDashboard.putString("AutoAlign/StartPose", startingPose.toString());
        SmartDashboard.putNumber("AutoAlign/TagYawDeg", Math.toDegrees(tagYawRad));
    }

    @Override
    public void execute() {
        if (!hasValidTarget) return;

        // always use odometry for current pose
        Pose2d currentPose = drivebase.getPose();

        double xSpeed     = xController.calculate(currentPose.getX());
        double ySpeed     = yController.calculate(currentPose.getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

        if (xController.atGoal())     xSpeed     *= 0.2;
        if (yController.atGoal())     ySpeed     *= 0.2;
        if (thetaController.atGoal()) thetaSpeed *= 0.2;

        // dynamic inversion: tags facing red side (|yaw|<90°) need inverted drives
        if (Math.abs(tagYawRad) < Math.PI/2.0) {
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
        }

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, thetaSpeed, drivebase.getHeading()
        );
        drivebase.drive(speeds);

        // telemetry
        SmartDashboard.putString("AutoAlign/CurrentPose", currentPose.toString());
        SmartDashboard.putString("AutoAlign/TargetPose",  targetPose.toString());
        SmartDashboard.putNumber("AutoAlign/XError",      xController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/YError",      yController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/ThetaError",  thetaController.getPositionError());
        SmartDashboard.putBoolean("AutoAlign/Active",     isActive);
        SmartDashboard.putBoolean("AutoAlign/HasValidTarget", hasValidTarget);
        SmartDashboard.putNumber("AutoAlign/TimeElapsed", Timer.getFPGATimestamp() - startTime);
        SmartDashboard.putNumber("AutoAlign/TargetID",    targetTagId);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new ChassisSpeeds());
        SmartDashboard.putBoolean("AutoAlign/Interrupted", interrupted);
        isActive = false;
    }

    @Override
    public boolean isFinished() {
        return hasValidTarget
            && xController.atGoal()
            && yController.atGoal()
            && thetaController.atGoal()
            && Timer.getFPGATimestamp() - startTime > 1.0;
    }
}

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.VisionSim;

public class AutoAlignCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final SwerveSubsystem drivebase;
    private final int targetTagId;
    private boolean isActive = false;

    private final ProfiledPIDController xController =
        new ProfiledPIDController(1.5, 0, 0.2, new TrapezoidProfile.Constraints(3, 2));
    private final ProfiledPIDController yController =
        new ProfiledPIDController(1.5, 0, 0.2, new TrapezoidProfile.Constraints(3, 2));
    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(2.0, 0, 0.1, new TrapezoidProfile.Constraints(8, 8));

    private Pose2d targetPose = new Pose2d();
    private Pose2d startingPose;
    private double startTime;
    private boolean hasValidTarget = false;
    private double tagYawRad = 0.0;  // remember tag orientation

    public AutoAlignCommand(VisionSubsystem visionSubsystem, SwerveSubsystem drivebase, int targetTagId) {
        this.visionSubsystem = visionSubsystem;
        this.drivebase   = drivebase;
        this.targetTagId = targetTagId;

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        thetaController.setTolerance(Units.degreesToRadians(1.5));
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

        Constants.Vision.APRILTAG_FIELD_LAYOUT
            .getTagPose(targetTagId)
            .ifPresent(tagPose3d -> {
                // capture tag yaw
                tagYawRad = tagPose3d.getRotation().getZ();

                // offset 1m in front of tag
                Transform3d tagToGoal = new Transform3d(
                    new Translation3d(2.0, 0.0, 0.0),
                    new Rotation3d()
                );
                Pose3d goal3d  = tagPose3d.transformBy(tagToGoal);
                targetPose     = goal3d.toPose2d();
                hasValidTarget = true;

                xController.setGoal(targetPose.getX());
                yController.setGoal(targetPose.getY());

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

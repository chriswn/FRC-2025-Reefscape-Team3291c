package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.Optional;

public class ChaseTag2 extends Command {
    
    // Motion constraints for profiling
    private static final TrapezoidProfile.Constraints XY_CONSTRAINTS = 
        new TrapezoidProfile.Constraints(3.0, 2.0);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = 
        new TrapezoidProfile.Constraints(8.0, 8.0);
    
    private final VisionSubsystem visionSubsystem;
    private final SwerveSubsystem drivebase;
    
    private final ProfiledPIDController xController = 
        new ProfiledPIDController(3.0, 0.0, 0.0, XY_CONSTRAINTS);
    private final ProfiledPIDController yController = 
        new ProfiledPIDController(3.0, 0.0, 0.0, XY_CONSTRAINTS);
    private final ProfiledPIDController omegaController = 
        new ProfiledPIDController(2.0, 0.0, 0.0, OMEGA_CONSTRAINTS);

    // This transform offsets the tag to a goal pose (adjust as needed)
    private final Transform3d TAG_TO_GOAL = new Transform3d(
        new Translation3d(1.5, 0.0, 0.0), 
        new Rotation3d(0.0, 0.0, Math.PI)
    );

    private Pose2d targetPose = new Pose2d();
    private Pose2d startingPose;
    private double startTime;
    private boolean hasValidTarget = false;
    private double tagYawRad = 0.0;  // remember tag orientation

    public ChaseTag2(VisionSubsystem visionSubsystem, SwerveSubsystem drivebase) {
        this.visionSubsystem = visionSubsystem;
        this.drivebase = drivebase;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        startingPose = drivebase.getPose();
        hasValidTarget = false;

        xController.reset(startingPose.getX());
        yController.reset(startingPose.getY());
        omegaController.reset(startingPose.getRotation().getRadians());

        // Retrieve the tag pose from your field layout (using your constants)
        Constants.Vision.APRILTAG_FIELD_LAYOUT
            .getTagPose(Constants.Vision.TARGET_TAG_ID)
            .ifPresent(tagPose3d -> {
                // Capture tag yaw (assuming Z-axis holds the yaw value)
                tagYawRad = tagPose3d.getRotation().getZ();

                // Offset from tag to goal pose (adjust offset as needed)
                Pose3d goal3d = tagPose3d.transformBy(TAG_TO_GOAL);
                targetPose = goal3d.toPose2d();
                hasValidTarget = true;

                xController.setGoal(targetPose.getX());
                yController.setGoal(targetPose.getY());

                double yawGoal = Math.abs(tagYawRad) < Math.PI/2.0 
                    ? Math.toRadians(180) : 0.0; // Example logic: flip goal based on tag orientation
                omegaController.setGoal(yawGoal);
            });

        SmartDashboard.putString("ChaseTag/StartPose", startingPose.toString());
        SmartDashboard.putNumber("ChaseTag/TagYawDeg", Math.toDegrees(tagYawRad));
    }

    @Override
    public void execute() {
        // Get the latest vision result from the VisionSubsystem's camera
        PhotonPipelineResult result = visionSubsystem.getCamera().getLatestResult();
        if (!result.hasTargets()) {
            drivebase.drive(new ChassisSpeeds());
            return;
        }

        Optional<PhotonTrackedTarget> targetOpt = result.getTargets().stream()
            .filter(t -> t.getFiducialId() == Constants.Vision.TARGET_TAG_ID)
            .filter(t -> t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1)
            .findFirst();

        if (targetOpt.isEmpty()) {
            drivebase.drive(new ChassisSpeeds());
            return;
        }

        PhotonTrackedTarget target = targetOpt.get();
        updateGoalPosition(target);
        driveToTarget();
    }

    private void updateGoalPosition(PhotonTrackedTarget target) {
        Pose2d currentPose = drivebase.getPose();
        // Using your constants, get the transformation from robot to camera.
        Transform3d cameraToRobot = Constants.Vision.ROBOT_TO_CAMERA.inverse();
        
        // Get camera-to-target transform from the vision target data
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        
        // Calculate target pose relative to the field
        Pose3d targetPose = new Pose3d(currentPose).transformBy(cameraToRobot).transformBy(cameraToTarget);
        
        // Calculate desired goal pose relative to the tag
        Pose3d goalPose3d = targetPose.transformBy(TAG_TO_GOAL);
        Pose2d goalPose = goalPose3d.toPose2d();

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
    }

    private void driveToTarget() {
        Pose2d currentPose = drivebase.getPose();
        
        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double omegaSpeed = omegaController.calculate(currentPose.getRotation().getRadians());

        if (xController.atGoal()) xSpeed = 0;
        if (yController.atGoal()) ySpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, omegaSpeed, currentPose.getRotation()
        );

        drivebase.drive(speeds);
        updateTelemetry();
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("ChaseTag/X Error", xController.getPositionError());
        SmartDashboard.putNumber("ChaseTag/Y Error", yController.getPositionError());
        SmartDashboard.putNumber("ChaseTag/Omega Error", omegaController.getPositionError());
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }
}

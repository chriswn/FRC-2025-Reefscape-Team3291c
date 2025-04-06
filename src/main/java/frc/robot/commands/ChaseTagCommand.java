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
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class ChaseTagCommand extends Command {
    
    // Motion constraints for profiling
    private static final TrapezoidProfile.Constraints XY_CONSTRAINTS = 
        new TrapezoidProfile.Constraints(3.0, 2.0);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = 
        new TrapezoidProfile.Constraints(8.0, 8.0);
    
    private final PhotonCamera camera;
    private final SwerveSubsystem drivebase;
    
    private final ProfiledPIDController xController = 
        new ProfiledPIDController(3.0, 0.0, 0.0, XY_CONSTRAINTS);
    private final ProfiledPIDController yController = 
        new ProfiledPIDController(3.0, 0.0, 0.0, XY_CONSTRAINTS);
    private final ProfiledPIDController omegaController = 
        new ProfiledPIDController(2.0, 0.0, 0.0, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;
    private final Transform3d TAG_TO_GOAL = new Transform3d(
        new Translation3d(1.5, 0.0, 0.0), 
        new Rotation3d(0.0, 0.0, Math.PI));

    public ChaseTagCommand(PhotonCamera camera, SwerveSubsystem drivebase) {
        this.camera = camera;
        this.drivebase = drivebase;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        Pose2d currentPose = drivebase.getPose();
        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
        omegaController.reset(currentPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
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
        Pose3d robotPose3d = new Pose3d(drivebase.getPose());
        Transform3d cameraToRobot = Constants.Vision.ROBOT_TO_CAMERA.inverse();
        
        // Get camera to target transform
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        
        // Calculate target pose relative to field
        Pose3d targetPose = robotPose3d
            .transformBy(cameraToRobot)
            .transformBy(cameraToTarget);
        
        // Calculate desired goal pose
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
            xSpeed, 
            ySpeed, 
            omegaSpeed, 
            currentPose.getRotation()
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
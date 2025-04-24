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
    private double tagYawRad = 0.0;  // Remember tag orientation

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

        // Initialize telemetry
        SmartDashboard.putString("ChaseTag2/StartPose", startingPose.toString());
    }

    @Override
    public void execute() {

        PhotonPipelineResult result = visionSubsystem.getCamera().getLatestResult();
        if (!result.hasTargets()) {
        SmartDashboard.putString("ChaseTag2/TargetStatus", "No Targets");
        drivebase.drive(new ChassisSpeeds());
        return;
}
SmartDashboard.putString("ChaseTag2/TargetStatus", "Target Found");

       

        // Find the best target (smallest pose ambiguity)
        Optional<PhotonTrackedTarget> targetOpt = result.getTargets().stream()
        .filter(t -> t.getPoseAmbiguity() <= 0.9 && t.getPoseAmbiguity() != -1)
        .min((t1, t2) -> Double.compare(t1.getPoseAmbiguity(), t2.getPoseAmbiguity()));
    
    if (targetOpt.isEmpty()) {
        SmartDashboard.putString("ChaseTag2/TargetStatus", "No Valid Target");
        drivebase.drive(new ChassisSpeeds());
        return;
    }
    
        PhotonTrackedTarget target = targetOpt.get();
        SmartDashboard.putString("ChaseTag2/TargetPose", target.getBestCameraToTarget().toString());
         SmartDashboard.putNumber("ChaseTag2/PoseAmbiguity", target.getPoseAmbiguity());
        updateGoalPosition(target);
        driveToTarget();
    }

    private void updateGoalPosition(PhotonTrackedTarget target) {
        Pose2d currentPose = drivebase.getPose();
        
        // Using your constants, get the transformation from robot to camera.
        Transform3d cameraToRobot = Constants.Vision.ROBOT_TO_CAMERA.inverse();
        
        // Get camera-to-target transform from the vision target data
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        SmartDashboard.putString("ChaseTag2/CameraToTarget", cameraToTarget.toString());

        // Calculate target pose relative to the field
        Pose3d targetPose3d = new Pose3d(currentPose).transformBy(cameraToRobot).transformBy(cameraToTarget);
        SmartDashboard.putString("ChaseTag2/TargetPose3d", targetPose3d.toString());
        
        // Calculate desired goal pose relative to the tag
        Pose3d goalPose3d = targetPose3d.transformBy(TAG_TO_GOAL);
        Pose2d goalPose = goalPose3d.toPose2d();

        // Update the PID controllers with the new goal pose
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
    }

    private void driveToTarget() {
        Pose2d currentPose = drivebase.getPose();
        
        // Calculate PID outputs
        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double omegaSpeed = omegaController.calculate(currentPose.getRotation().getRadians());

        // If at goal, stop moving
        if (xController.atGoal()) xSpeed = 0;
        if (yController.atGoal()) ySpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        // Drive the robot with field-relative control
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, omegaSpeed, currentPose.getRotation()
        );
        
        SmartDashboard.putNumber("ChaseTag2/X Speed", xSpeed);
        SmartDashboard.putNumber("ChaseTag2/Y Speed", ySpeed);
        SmartDashboard.putNumber("ChaseTag2/Omega Speed", omegaSpeed);
        
        drivebase.drive(speeds);
        updateTelemetry();
    }

    private void updateTelemetry() {
        SmartDashboard.putString("ChaseTag2/GoalPose", targetPose.toString());
        SmartDashboard.putNumber("ChaseTag2/X Error", xController.getPositionError());
        SmartDashboard.putNumber("ChaseTag2/Y Error", yController.getPositionError());
        SmartDashboard.putNumber("ChaseTag2/Omega Error", omegaController.getPositionError());
        
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

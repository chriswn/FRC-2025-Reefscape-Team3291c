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
import frc.robot.VisionSim;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

public class AutoAlignCommand extends Command {

    private final VisionSim visionSim;
    private final SwerveSubsystem drivebase;
    private final int targetTagId;
    private boolean isActive = false;

    // Profiled PID Controllers
    private final ProfiledPIDController xController = 
        new ProfiledPIDController(1.5, 0, 0.2, new TrapezoidProfile.Constraints(3, 2));
    private final ProfiledPIDController yController = 
        new ProfiledPIDController(1.5, 0, 0.2, new TrapezoidProfile.Constraints(3, 2));
    private final ProfiledPIDController thetaController = 
        new ProfiledPIDController(2.0, 0, 0.1, new TrapezoidProfile.Constraints(8, 8));

    // State
    private Pose2d targetPose = new Pose2d();
    private double startTime;
    private boolean hasValidTarget;

    public AutoAlignCommand(VisionSim visionSim, SwerveSubsystem drivebase, 
                          int targetTagId) {
        this.visionSim = visionSim;
        this.drivebase = drivebase;
        this.targetTagId = targetTagId;
        

        configureControllers();
        addRequirements(drivebase);
    }

    private void configureControllers() {
        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        thetaController.setTolerance(Units.degreesToRadians(1.5));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        hasValidTarget = false;
        isActive = true;

        // Reset controllers with current state
        Pose2d currentPose = drivebase.getPose();
        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
        thetaController.reset(currentPose.getRotation().getRadians());

        
Constants.Vision.APRILTAG_FIELD_LAYOUT.getTagPose(targetTagId).ifPresent(tagPose3d -> {
    Rotation3d tagRotation = tagPose3d.getRotation();

    // Offset robot 1.5 meters in front of the tag (opposite its facing direction)
    Transform3d tagToGoal = new Transform3d(
        new Translation3d(1, 0.0, 0.0),  // 
        new Rotation3d(0.0, 0.0, 0.0)       // Robot faces the tag
    );

    Pose3d goalPose3d = tagPose3d.transformBy(tagToGoal);
    targetPose = goalPose3d.toPose2d();
    hasValidTarget = true;

    // Set controller goals: x, y = target position, theta = face the tag
    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    thetaController.setGoal(
        MathUtil.angleModulus(tagRotation.toRotation2d().plus(Rotation2d.fromDegrees(180)).getRadians())
    );
});
    }
        // Get target pose from field layout
    //     Constants.Vision.APRILTAG_FIELD_LAYOUT.getTagPose(targetTagId).ifPresent(tagPose3d -> {
    //         Pose3d goalPose3d = tagPose3d.transformBy(TAG_TO_GOAL);
    //         targetPose = goalPose3d.toPose2d();
    //         hasValidTarget = true;
            
    //         // Set controller goals
    //         xController.setGoal(targetPose.getX());
    //         yController.setGoal(targetPose.getY());
    //         thetaController.setGoal(targetPose.getRotation().getRadians());
    //     });
    // }

    @Override
    public void execute() {
        if (!hasValidTarget) return;

        // Get vision-corrected pose
        Pose2d currentPose = getCorrectedPose();
        
        // Calculate speeds using motion profiling
        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

        // Apply smoothing at goal
        if (xController.atGoal()) xSpeed *= 0.2;
        if (yController.atGoal()) ySpeed *= 0.2;
        if (thetaController.atGoal()) thetaSpeed *= 0.2;

        // Drive with field-relative speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, thetaSpeed, drivebase.getHeading()
        );
        
        drivebase.drive(speeds);
        updateTelemetry(currentPose);
    }

    private Pose2d getCorrectedPose() {
        // Use vision pose when available with low ambiguity
        Optional<EstimatedRobotPose> visionEst = visionSim.getEstimatedGlobalPose();
        if (visionEst.isPresent() && visionEst.get().estimatedPose.getTranslation().getNorm() < 3.0) {
            return visionEst.get().estimatedPose.toPose2d();
        }
        return drivebase.getPose();
    }

    private void updateTelemetry(Pose2d currentPose) {
        SmartDashboard.putNumberArray("AutoAlign/Current Pose", 
            new double[]{currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()});
        
        SmartDashboard.putNumberArray("AutoAlign/Target Pose", 
            new double[]{targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});
        
        SmartDashboard.putNumber("AutoAlign/X Error", xController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/Y Error", yController.getPositionError());
        SmartDashboard.putNumber("AutoAlign/Theta Error", thetaController.getPositionError());
        SmartDashboard.putBoolean("AutoAlign/Active", isActive);
        SmartDashboard.putBoolean("AutoAlign/Valid Target", hasValidTarget);
        SmartDashboard.putNumber("AutoAlign/Time Elapsed", Timer.getFPGATimestamp() - startTime);   
        SmartDashboard.putNumber("AutoAlign/Target ID", targetTagId);
        SmartDashboard.putNumber("AutoAlign/Target X", targetPose.getX());
        SmartDashboard.putNumber("AutoAlign/Target Y", targetPose.getY());
        SmartDashboard.putNumber("AutoAlign/Target Rotation", targetPose.getRotation().getDegrees());
        SmartDashboard.putNumber("AutoAlign/Current X", currentPose.getX());
        SmartDashboard.putNumber("AutoAlign/Current Y", currentPose.getY());
        SmartDashboard.putNumber("AutoAlign/Current Rotation", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("AutoAlign/Current Heading", drivebase.getHeading().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new ChassisSpeeds());
    }

    public boolean isActive() {
        return isActive;
    }


    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime > 1.0) && 
               xController.atGoal() && 
               yController.atGoal() && 
               thetaController.atGoal();
    }
    
}
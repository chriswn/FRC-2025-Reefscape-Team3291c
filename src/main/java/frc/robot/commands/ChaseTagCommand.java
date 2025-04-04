package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import frc.robot.VisionSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class ChaseTagCommand extends Command {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private final XboxController controller;
    private final SwerveSubsystem drivebase;
    private boolean chasingTag = false;
    
    // PID constants
    private static final double TURN_kP = 0.03;
    private static final double DRIVE_kP = 0.5;
    private static final double STRAFE_kP = 0.4;
    private static final double MAX_DRIVE_SPEED = 3.0;
    private static final double MAX_TURN_SPEED = Math.PI;
    private static final double MAX_STRAFE_SPEED = 2.0;
    private static final double TARGET_DISTANCE = 1.0;
    private static final double TARGET_CENTER_TOLERANCE = 2.5;
    
    private final SlewRateLimiter driveLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2.0);
    
    public ChaseTagCommand(PhotonCamera camera, PhotonPoseEstimator photonEstimator, 
                         XboxController controller, SwerveSubsystem drivebase) {
        this.camera = camera;
        this.photonEstimator = photonEstimator;
        this.controller = controller;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        chasingTag = false;
    }

    @Override
    public void execute() {
        if (controller.getAButtonPressed()) {
            chasingTag = !chasingTag;
        }

        if (chasingTag) {
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                double yaw = target.getYaw();
                double pitch = target.getPitch();
                double distance = calculateDistance(pitch);
                
                double driveSpeed = driveLimiter.calculate(calculateDriveSpeed(distance));
                double turnSpeed = turnLimiter.calculate(calculateTurnSpeed(yaw));
                double strafeSpeed = strafeLimiter.calculate(calculateStrafeSpeed(yaw));
                
                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveSpeed, 
                    strafeSpeed,  
                    turnSpeed,
                    drivebase.getHeading()
                );
                
                drivebase.drive(speeds);
                
                SmartDashboard.putNumber("Tag Distance", distance);
                SmartDashboard.putNumber("Tag Yaw", yaw);
                SmartDashboard.putNumber("Drive Speed", driveSpeed);
                SmartDashboard.putNumber("Turn Speed", turnSpeed);
                SmartDashboard.putNumber("Strafe Speed", strafeSpeed);
            } else {
                drivebase.drive(new ChassisSpeeds());
            }
        } else {
            drivebase.drive(new ChassisSpeeds());
        }
    }

    private double calculateDistance(double pitch) {
        final double CAMERA_HEIGHT = 0.5;
        final double TAG_HEIGHT = 1.0;
        return (TAG_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(pitch));
    }

    private double calculateDriveSpeed(double distance) {
        if (Math.abs(distance - TARGET_DISTANCE) < 0.1) {
            return 0;
        }
        double speed = (distance - TARGET_DISTANCE) * DRIVE_kP;
        return Math.copySign(Math.min(Math.abs(speed), MAX_DRIVE_SPEED), speed);
    }

    private double calculateTurnSpeed(double yaw) {
        if (Math.abs(yaw) < TARGET_CENTER_TOLERANCE) {
            return 0;
        }
        double speed = -Units.degreesToRadians(yaw) * TURN_kP;
        return Math.copySign(Math.min(Math.abs(speed), MAX_TURN_SPEED), speed);
    }

    private double calculateStrafeSpeed(double yaw) {
        if (Math.abs(yaw) < TARGET_CENTER_TOLERANCE) {
            return 0;
        }
        double speed = yaw * STRAFE_kP; 
        return Math.copySign(Math.min(Math.abs(speed), MAX_STRAFE_SPEED), speed);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

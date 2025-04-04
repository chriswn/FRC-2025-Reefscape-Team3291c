package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ChaseTagCommand extends Command {
    private final PhotonCamera camera;
    private final SwerveSubsystem drivebase;
    
    // PID constants
    private static final double TURN_kP = 0.035;
    private static final double DRIVE_kP = 0.4;
    private static final double STRAFE_kP = 0.35;
    private static final double MAX_DRIVE_SPEED = 3.0;
    private static final double MAX_TURN_SPEED = Math.PI;
    private static final double MAX_STRAFE_SPEED = 2.0;
    private static final double TARGET_DISTANCE = 1.0;
    private static final double TARGET_YAW_TOLERANCE = 1.5;

    private final SlewRateLimiter driveLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(3.0);

    public ChaseTagCommand(PhotonCamera camera, SwerveSubsystem drivebase) {
        this.camera = camera;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            drivebase.drive(new ChassisSpeeds());
            return;
        }

        PhotonTrackedTarget target = getDesiredTarget(result);
        if (target == null) {
            drivebase.drive(new ChassisSpeeds());
            return;
        }

        processTarget(target);
    }

    private PhotonTrackedTarget getDesiredTarget(PhotonPipelineResult result) {
        return result.getTargets().stream()
                .filter(t -> t.getFiducialId() == Constants.Vision.TARGET_TAG_ID)
                .findFirst()
                .orElse(null);
    }

    private void processTarget(PhotonTrackedTarget target) {
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
        updateTelemetry(distance, yaw, driveSpeed, turnSpeed, strafeSpeed);
    }

    private double calculateDistance(double pitch) {
        return Math.abs(
            (Constants.Vision.TAG_HEIGHT - Constants.Vision.CAMERA_HEIGHT) 
            / Math.tan(Units.degreesToRadians(pitch))
        );
    }

    private double calculateDriveSpeed(double distance) {
        double error = distance - TARGET_DISTANCE;
        if (Math.abs(error) < 0.1) return 0;
        return edu.wpi.first.math.MathUtil.clamp(error * DRIVE_kP, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    }

    private double calculateTurnSpeed(double yaw) {
        if (Math.abs(yaw) < TARGET_YAW_TOLERANCE) return 0;
        return edu.wpi.first.math.MathUtil.clamp(
            -Units.degreesToRadians(yaw) * TURN_kP, 
            -MAX_TURN_SPEED, 
            MAX_TURN_SPEED
        );
    }

    private double calculateStrafeSpeed(double yaw) {
        if (Math.abs(yaw) < TARGET_YAW_TOLERANCE) return 0;
        return edu.wpi.first.math.MathUtil.clamp(
            -yaw * STRAFE_kP,
            -MAX_STRAFE_SPEED,
            MAX_STRAFE_SPEED
        );
    }

    private void updateTelemetry(double distance, double yaw, double drive, double turn, double strafe) {
        SmartDashboard.putNumber("ChaseTag/Distance", distance);
        SmartDashboard.putNumber("ChaseTag/Yaw", yaw);
        SmartDashboard.putNumber("ChaseTag/Drive", drive);
        SmartDashboard.putNumber("ChaseTag/Turn", turn);
        SmartDashboard.putNumber("ChaseTag/Strafe", strafe);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new ChassisSpeeds());
        driveLimiter.reset(0);
        strafeLimiter.reset(0);
        turnLimiter.reset(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
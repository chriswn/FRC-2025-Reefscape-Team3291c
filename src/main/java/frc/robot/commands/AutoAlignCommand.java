package frc.robot.commands;

import java.util.*;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ScoringTarget;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignCommand extends Command {
  private final VisionSubsystem visionSubsystem;
  private final SwerveSubsystem drivebase;
  private final Supplier<Optional<ScoringTarget>> scoringTargetSupplier;
  private final Field2d field;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(1.5, 0, 0.2, new TrapezoidProfile.Constraints(3, 2));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(1.5, 0, 0.2, new TrapezoidProfile.Constraints(3, 2));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(2.0, 0, 0.1, new TrapezoidProfile.Constraints(8, 8));

  private Pose2d targetPose = new Pose2d();
  private Pose2d startingPose;
  private List<Pose2d> bezierPath;
  private int pathIndex = 0;
  private double startTime;
  private boolean hasValidTarget = false;
  private double tagYawRad = 0.0;

  public AutoAlignCommand(
      VisionSubsystem visionSubsystem,
      SwerveSubsystem drivebase,
      Supplier<Optional<ScoringTarget>> scoringTargetSupplier,
      Field2d field
  ) {
    this.visionSubsystem = visionSubsystem;
    this.drivebase = drivebase;
    this.scoringTargetSupplier = scoringTargetSupplier;
    this.field = field;

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    thetaController.setTolerance(Units.degreesToRadians(1.5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    startingPose = drivebase.getPose();
    hasValidTarget = false;
    System.out.println("[AutoAlign] Initializing...");

    scoringTargetSupplier.get().ifPresent(target -> {
      int tagId = target.getTagId();
      Constants.Vision.APRILTAG_FIELD_LAYOUT.getTagPose(tagId).ifPresent(tagPose3d -> {
        tagYawRad = tagPose3d.getRotation().getZ();

        Transform3d tagToGoal = new Transform3d(new Translation3d(1.5, 0.0, 0.0), new Rotation3d());
        Pose3d goal3d = tagPose3d.transformBy(tagToGoal);
        targetPose = goal3d.toPose2d();

        hasValidTarget = true;
        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());
        double yawGoal = MathUtil.angleModulus(tagYawRad + Math.PI);
        thetaController.setGoal(yawGoal);

        loadPIDFromSmartDashboard();

        bezierPath = generateBezierPath(startingPose, targetPose, 50);
        field.getObject("Bezier Path").setPoses(bezierPath);
        pathIndex = 0;
        startTime = Timer.getFPGATimestamp();
      });
    });
  }

  @Override
  public void execute() {
    loadPIDFromSmartDashboard();
    if (!hasValidTarget || bezierPath == null) return;

    Pose2d currentPose = drivebase.getPose();
    if (pathIndex < bezierPath.size() - 1) pathIndex++;
    Pose2d intermediate = bezierPath.get(pathIndex);

    double xSpeed = xController.calculate(currentPose.getX(), intermediate.getX());
    double ySpeed = yController.calculate(currentPose.getY(), intermediate.getY());
    double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians());

    if (xController.atGoal()) xSpeed *= 0.2;
    if (yController.atGoal()) ySpeed *= 0.2;
    if (thetaController.atGoal()) thetaSpeed *= 0.2;

    if (Math.abs(tagYawRad) < Math.PI / 2.0) {
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, thetaSpeed, drivebase.getHeading()
    );
    drivebase.drive(speeds);

    SmartDashboard.putString("AutoAlign/CurrentPose", currentPose.toString());
    SmartDashboard.putString("AutoAlign/TargetPose", targetPose.toString());
    SmartDashboard.putNumber("AutoAlign/XError", xController.getPositionError());
    SmartDashboard.putNumber("AutoAlign/YError", yController.getPositionError());
    SmartDashboard.putNumber("AutoAlign/ThetaError", thetaController.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new ChassisSpeeds());
    SmartDashboard.putBoolean("AutoAlign/Interrupted", interrupted);
  }

  @Override
  public boolean isFinished() {
    return hasValidTarget
        && xController.atGoal()
        && yController.atGoal()
        && thetaController.atGoal()
        && (Timer.getFPGATimestamp() - startTime) > 1.0;
  }

  private void loadPIDFromSmartDashboard() {
    xController.setP(SmartDashboard.getNumber("AutoAlign/X_P", 1.5));
    xController.setI(SmartDashboard.getNumber("AutoAlign/X_I", 0.0));
    xController.setD(SmartDashboard.getNumber("AutoAlign/X_D", 0.2));

    yController.setP(SmartDashboard.getNumber("AutoAlign/Y_P", 1.5));
    yController.setI(SmartDashboard.getNumber("AutoAlign/Y_I", 0.0));
    yController.setD(SmartDashboard.getNumber("AutoAlign/Y_D", 0.2));

    thetaController.setP(SmartDashboard.getNumber("AutoAlign/Theta_P", 2.0));
    thetaController.setI(SmartDashboard.getNumber("AutoAlign/Theta_I", 0.0));
    thetaController.setD(SmartDashboard.getNumber("AutoAlign/Theta_D", 0.1));
  }

  private List<Pose2d> generateBezierPath(Pose2d start, Pose2d end, int samples) {
    List<Pose2d> path = new ArrayList<>();

    Translation2d p0 = start.getTranslation();
    Translation2d p3 = end.getTranslation();

    Translation2d p1 = p0.plus(new Translation2d(start.getRotation().getCos(), start.getRotation().getSin()).times(0.5));
    Translation2d p2 = p3.minus(new Translation2d(end.getRotation().getCos(), end.getRotation().getSin()).times(0.5));

    for (int i = 0; i <= samples; i++) {
      double t = i / (double) samples;
      Translation2d pos = bezierPoint(p0, p1, p2, p3, t);
      path.add(new Pose2d(pos, end.getRotation()));
    }

    return path;
  }

  private Translation2d bezierPoint(Translation2d p0, Translation2d p1, Translation2d p2, Translation2d p3, double t) {
    double umt = 1 - t;
    double x = Math.pow(umt, 3) * p0.getX()
        + 3 * Math.pow(umt, 2) * t * p1.getX()
        + 3 * umt * Math.pow(t, 2) * p2.getX()
        + Math.pow(t, 3) * p3.getX();
    double y = Math.pow(umt, 3) * p0.getY()
        + 3 * Math.pow(umt, 2) * t * p1.getY()
        + 3 * umt * Math.pow(t, 2) * p2.getY()
        + Math.pow(t, 3) * p3.getY();
    return new Translation2d(x, y);
  }
}

package frc.robot.commands;

import java.util.Optional;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.ElevatorCMDs.SetElevatorLevelCommand;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;

public class MultiScoreCommand extends SequentialCommandGroup {
  public MultiScoreCommand(
      VisionSubsystem vision,
      SwerveSubsystem drivebase,
      ElevatorSubsystem elevator,
      IntakeMotorSubsystem intake,
      ScoringTargetManager manager
  ) {
    ProxyCommand loopBody = new ProxyCommand(() -> {
      ScoringTarget target = manager.getNextTarget();
      if (target == null) {
        return Commands.none();  // break the loop
      }

      manager.callTarget(target);
      SmartDashboard.putString("Picked target", target.toString());
      System.out.println("[MultiScore] Aligning to TagID: " + target.getTagId());

      return Commands.sequence(
        new AutoAlignCommand(vision, drivebase, () -> Optional.of(target)),
        new SetElevatorLevelCommand(elevator, target.getLevel())
          .beforeStarting(() ->
            System.out.println("[MultiScore] Elevator to level " + target.getLevel())
          ),
        new ESpitCMD(intake).withTimeout(1.0),
        Commands.runOnce(() -> {
          manager.markScored(target);
          System.out.println("[MultiScore] Scored: " + target);
        })
      );
    });

    // Use RepeatCommand directly
    addCommands(
      new RepeatCommand(loopBody)
        .until(() -> !manager.hasUnscoredTargets())
    );
  }
}

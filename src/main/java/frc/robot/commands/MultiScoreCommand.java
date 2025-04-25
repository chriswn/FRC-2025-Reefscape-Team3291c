package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.ElevatorCMDs.SetElevatorLevelCommand;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;
import frc.robot.commands.AutoAlignCommand;
public class MultiScoreCommand extends SequentialCommandGroup {
  public MultiScoreCommand(
      VisionSubsystem vision,
      SwerveSubsystem drivebase,
      ElevatorSubsystem elevator,
      IntakeMotorSubsystem intake,
      ScoringTargetManager manager
  ) {
    // Build one proxy that handles fetching + building the inner sequence
    ProxyCommand loopBody = new ProxyCommand(() -> {
      ScoringTarget target = manager.getNextTarget();
      if (target == null) {
        // No targets left → no-op instantly, will break the repeat
        return Commands.none();
      }

      // Announce & reserve
      manager.callTarget(target);
      SmartDashboard.putString("Picked target", target.toString());
      System.out.println("[MultiScore] Aligning to TagID: " + target.getTagId());

      // Per-target action sequence
      return Commands.sequence(
        new AutoAlignCommand(vision, drivebase, () -> Optional.of(target)),
        new SetElevatorLevelCommand(elevator, target.getLevel())
          .beforeStarting(()
            -> System.out.println("[MultiScore] Elevator to level " + target.getLevel())
          ),
        new ESpitCMD(intake).withTimeout(1.0),
        Commands.runOnce(() -> {
          manager.markScored(target);
          System.out.println("[MultiScore] Scored: " + target);
        })
      );
    });

    // Repeat until manager reports no more unscored targets
    addCommands(
      Commands.repeat(loopBody)
              .until(() -> !manager.hasUnscoredTargets())
    );
  }
}

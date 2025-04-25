package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.ElevatorCMDs.*; // Adjust the package path if necessary
import frc.robot.commands.IntakeMotorCMDs.*;

public class MultiScoreCommand extends SequentialCommandGroup {
  public MultiScoreCommand(
      VisionSubsystem vision,
      SwerveSubsystem drivebase,
      ElevatorSubsystem elevator,
      IntakeMotorSubsystem intake,
      ScoringTargetManager manager
  ) {
<<<<<<< HEAD
    List<Command> steps = new ArrayList<>();  

    // Pull off every unscored target up front
    ScoringTarget target;
    while ((target = manager.getNextTarget()) != null) {
      final ScoringTarget t = target; // capture for lambdas

      // Mark the target as scored immediately so getNextTarget() will move to the next one
      manager.markScored(t);
      
      // 1) Auto-align to this tag
      steps.add(
        new AutoAlignCommand(vision, drivebase, () -> Optional.of(t))
          .beforeStarting(() -> 
            System.out.println("[MultiScore] Aligning to TagID: " + t.getTagId())
          )
      );

      // 2) Move elevator to the level
      steps.add(
        new SetElevatorLevelCommand(elevator, t.getLevel())
          .beforeStarting(() -> {
            System.out.println("[MultiScore] Moving elevator to level " + t.getLevel());
            // Log the current elevator position before moving
            System.out.println("[Elevator] Current level: " + elevator.getCurrentLevel());
          })
          .andThen(() -> {
            // Log after elevator move
            System.out.println("[Elevator] Elevator move complete");
            System.out.println("[Elevator] New level: " + elevator.getCurrentLevel());
          })
      );

      // 3) Spit with a 1s timeout
      steps.add(
        new ESpitCMD(intake)
          .withTimeout(1.0)
          .beforeStarting(() -> System.out.println("[MultiScore] Spitting"))
          .andThen(() -> System.out.println("[MultiScore] Spit complete"))
      );

      // 4) Mark that target as done (it was already marked as scored above)
      steps.add(
        Commands.runOnce(() -> {
          System.out.println("[MultiScore] Marked scored: " + t);
        })
      );
    }

    // If no targets were found, this group will immediately finish.
    addCommands(steps.toArray(new Command[0]));
=======
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
>>>>>>> 85c5ae3c73aa630ae5af5d3411259d32da4df784
  }
}

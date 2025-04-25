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
  }
}

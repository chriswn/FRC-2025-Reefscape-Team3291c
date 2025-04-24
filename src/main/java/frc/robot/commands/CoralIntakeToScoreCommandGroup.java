package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ChaseTag2;
import frc.robot.commands.IntakeMotorCMDs.IntakeCMD;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;
import frc.robot.commands.IntakePivotCMDs.PivotToGround;
import frc.robot.commands.IntakePivotCMDs.PivotToStow;

/**
 * Command group to intake a coral piece, align to the reef tag, and score the piece.
 * Uses only subsystems declared in RobotContainer.
 */
public class CoralIntakeToScoreCommandGroup extends SequentialCommandGroup {
    /**
     * @param visionVision subsystem handling AprilTag vision
     * @param drivebase   swerve drive subsystem
     * @param intakeMotor intake motor subsystem for spinning rollers
     * @param intakePivot intake pivot subsystem for lowering/raising intake
     * @param elevator    elevator subsystem (if needed for tie-in)
     */
    public CoralIntakeToScoreCommandGroup(
        VisionSubsystem vision,
        SwerveSubsystem drivebase,
        IntakeMotorSubsystem intakeMotor,
        IntakePivotSubsystem intakePivot,
        ElevatorSubsystem elevator
    ) {
        super(
            // 1. Lower intake to ground
            new PivotToGround(intakePivot),

            // 2. Run intake rollers to pick up coral
            new IntakeCMD(intakeMotor).withTimeout(2.0),

            // 3. Raise intake to stow position
            new PivotToStow(intakePivot),

            // 4. Brief pause for any motion to settle
            new WaitCommand(0.2),

            // 5. Align to AprilTag on the reef
            new ChaseTag2(vision, drivebase),

            // 6. Release the coral to score
            new ESpitCMD(intakeMotor).withTimeout(1.0)
        );
    }
}

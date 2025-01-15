package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int canCoderId;

    public final Rotation2d angleOffset;

    public final double angleMultiplier;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * 
     * @param driveMotorId  Drive motor CAN device ID or PWM port number
     * @param angleMotorId  Angle motor CAN device ID or PWM port number
     * @param canCoderId    CANCoder encoder CAN device ID
     * @param angleOffset   Wheel starting angle
     */
    public SwerveModuleConstants(
        int driveMotorId,
        int angleMotorId,
        int canCoderId,
        Rotation2d angleOffset,
        double angleMultiplier
    ) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.canCoderId   = canCoderId;
        this.angleOffset  = angleOffset;
        this.angleMultiplier = angleMultiplier;
    }
}

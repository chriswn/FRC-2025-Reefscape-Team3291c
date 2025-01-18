package frc.lib.util;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CANSparkMaxUtil {
    public enum Usage {
        kAll,
        kPositionOnly,
        kVelocityOnly,
        kMinimal
    };

    /**
     * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
     * frame period of nonessential frames from 20ms to 500ms.
     *
     * <p>See
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * for a description of the status frames.
     *
     * @param motor The motor to adjust the status frame periods on.
     * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
     *     constructed.
     * @param enableFollowing Whether to enable motor following.
     */
    public static void setCANSparkMaxBusUsage(
        SparkMax motor,          // SparkMAX motor to configure
        Usage usage,                // Enum above, used to determine which config
        boolean enableFollowing     // Used to determine following
    ) {
        SparkMaxConfig config = new SparkMaxConfig();

        // Set rate of transmission for Status0
        if (enableFollowing) {
            config.signals.appliedOutputPeriodMs(10);
            //motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 10);
        } else {
            config.signals.appliedOutputPeriodMs(500);
           // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 500);
        }

        if (usage == Usage.kAll) {
            config.signals
                .primaryEncoderVelocityPeriodMs(20)  // Previously status 1
                .primaryEncoderPositionPeriodMs(20)  // Previously status 2
                .analogVoltagePeriodMs(50);                  // Previously status 3
            // Increase rate for all Status channels
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 20);
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 20);
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 50);
        } else if (usage == Usage.kPositionOnly) {
            // Only increase Position (Status2) channel
            config.signals
                .primaryEncoderVelocityPeriodMs(500)  // Previously status 1
                .primaryEncoderPositionPeriodMs(20)  // Previously status 2
                .analogVoltagePeriodMs(500); 
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 500);
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 20);
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 500);
        } else if (usage == Usage.kVelocityOnly) {
            // Only increase Velocity (Status1) channel
            config.signals
                .primaryEncoderVelocityPeriodMs(20)  // Previously status 1
                .primaryEncoderPositionPeriodMs(500)  // Previously status 2
                .analogVoltagePeriodMs(500); 
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 20);
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 500);
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 500);
        } else if (usage == Usage.kMinimal) {
            // Setting minimal rate for all channels.
            config.signals
                .primaryEncoderVelocityPeriodMs(500)  // Previously status 1
                .primaryEncoderPositionPeriodMs(500)  // Previously status 2
                .analogVoltagePeriodMs(500); 
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 500);
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 500);
            // motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 500);
        }
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
     * frame period of nonessential frames from 20ms to 500ms.
     *
     * <p>See
     * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     * for a description of the status frames.
     *
     * @param motor The motor to adjust the status frame periods on.
     * @param usage The status frame feedack to enable. kAll is the default when a SparkMax is
     *     constructed.
     */
    public static void setSparkMaxBusUsage(SparkMax motor, Usage usage) {
        setCANSparkMaxBusUsage(motor, usage, false);
    }
}

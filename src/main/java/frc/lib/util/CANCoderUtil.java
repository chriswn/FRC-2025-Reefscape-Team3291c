// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

/** Sets status frames for the CTRE CANCoder. */
public class CANCoderUtil {
  public enum CCUsage {
    kAll,
    kSensorDataOnly,
    kFaultsOnly,
    kMinimal
  }

  /**
   * This function allows reducing a CANCoder's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 10ms to 255ms.
   *
   * <p>See https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html#cancoder for a description
   * of the status frames.
   *
   * @param angleEncoder The CANCoder to adjust the status frames on.
   * @param usage The status frame feedback to enable. kAll is the default when a CANCoder
   *     isconstructed.
   */
  public static void setCANCoderBusUsage(CANCoder angleEncoder, CCUsage usage) {
    if (usage == CCUsage.kAll) {
      angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
      angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10);
    } else if (usage == CCUsage.kSensorDataOnly) {
      angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
      angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100);
    } else if (usage == CCUsage.kFaultsOnly) {
      angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
      angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 10);
    } else if (usage == CCUsage.kMinimal) {
      angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
      angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100);
    }
  }

}
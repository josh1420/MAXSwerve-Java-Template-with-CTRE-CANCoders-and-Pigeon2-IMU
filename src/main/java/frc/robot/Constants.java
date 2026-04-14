// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // CANCoder offsets of each module in rotations, make sure the bevel gears are all pointed to the right side of the robot
    public static final double kFrontLeftCANCoderOffset = 0.959473;
    public static final double kFrontRightCANCoderOffset = 0.190918;
    public static final double kBackLeftCANCoderOffset = 0.743652;
    public static final double kBackRightCANCoderOffset = 0.375488;

    // SPARK MAX and CANCoder CAN IDs
    public static final int kFrontLeftDrivingCanId = 18;
    public static final int kRearLeftDrivingCanId = 19;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 10;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 8;

    public static final int kFrontLeftCANCoderId = 22;
    public static final int kFrontRightCANCoderId = 21;
    public static final int kBackLeftCANCoderId = 23;
    public static final int kBackRightCANCoderId = 24;


    public static final int kGyroCanId = 20;
    public static final boolean kGyroReversed = false;
    
  }

public static final class ModuleConstants {

  // SDS MK4i L3 has fixed gearing (no pinion options)
  public static final double kDrivingMotorFreeSpeedRps =
      NeoMotorConstants.kFreeSpeedRpm / 60.0;

  // 4 inch wheel
  public static final double kWheelDiameterMeters = 0.1016;
  public static final double kWheelCircumferenceMeters =
      kWheelDiameterMeters * Math.PI;

  // SDS MK4i L3 drive reduction
  public static final double kDrivingMotorReduction = 6.12;

  public static final double kDriveWheelFreeSpeedRps =
      (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
          / kDrivingMotorReduction;
}

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIdentification {
    public static final int SLAP_DOWN_MOTOR_ID = 9;
    public static final int INTAKE_ROLLER_LEADER_MOTOR_ID = 10;
    public static final int INTAKE_ROLLER_FOLLOWER_MOTOR_ID = 11;
    public static final int FEEDING_ROLLER_MOTOR_ID = 12;
    public static final int SHOOTER_FEED_LEADER_MOTOR_ID = 13;
    public static final int SHOOTER_FEED_FOLLOWER_MOTOR_ID = 14;
    public static final int TURRET_MOTOR_ID = 15;
    public static final int SHOOTER_SPEED_LEADER_MOTOR_ID = 16;
    public static final int SHOOTER_SPEED_FOLLOWER_MOTOR_ID = 17;
    public static final int HOOD_ANGLE_MOTOR_ID = 18;
  }

  public static class DriveConstants {
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static final LinearAcceleration MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(3); // TODO measure
    public static final AngularVelocity MAX_ANGULAR_RATE =
        RotationsPerSecond.of(0.75); // TODO measure
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(1); // TODO
    // measure

    public static final double STOW_DISTANCE_REQUIREMENT = 6.75;
    public static final double CENTER_XPOS = 8.25;

    public static final double DEADBAND_SCALAR = 0.1;
    public static final double SLOW_SPEED_SCALAR = 0.1;
    public static final double MID_SPEED_SCALAR = 0.5;
    public static final double FAST_SPEED_SCALAR = 0.75;
  }

  public static class IntakeConstants {
    public static final int SLAPDOWN_STOWED_POS = 0;
    public static final int SLAPDOWN_DOWN_POS = 0;
    public static final int INTAKE_ROLLER_MOTOR_SPEED = 0;
  }

  public static class SerializationConstants {
    public static final int FEEDING_ROLLER_MOTOR_SPEED = 0;
    public static final int SHOOTER_FEED_MOTOR_SPEED = 0;
  }

  public static class ShooterConstants {}

  public static class HubConstants {
    public static final Translation2d BLUE_HUB_POS = new Translation2d(4.61, 4.021);
    public static final Translation2d RED_HUB_POS = new Translation2d(11.9, 4.021);
  }
}

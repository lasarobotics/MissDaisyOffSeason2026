// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
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

  public static class DriveConstants {
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static final LinearAcceleration MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(3); // TODO measure
    public static final AngularVelocity MAX_ANGULAR_RATE =
        RotationsPerSecond.of(0.75); // TODO measure
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(1); // TODO}
    public static final double SLOWDOWN_SPEED = 0.3;
    public static final double DEADBAND_SCALAR = 0.1;
    public static final double TURN_P = 0;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;
  }

  public static class IntakeConstants {
    public static final int INTAKE_ROLLER_LEADER_ID = 31;
    public static final int INTAKE_ROLLER_FOLLOWER_ID = 32;
    public static final int INTAKE_SLAPDOWN_ID = 30;
    public static final double SLAPDOWN_POS = 0;
    public static final double INTAKE_ROLLER_DIAMETER = 0.035; // meters
    public static final int INTAKE_BOTTOM_ROLLER_GEAR_RATIO = 1; // motor : bottom roller
    public static final double INTAKE_DRIVETRAIN_SPEED_RATIO = 2;
    public static final double INTAKE_ROLLER_SPEED =
        (DriveConstants.MAX_SPEED.in(MetersPerSecond)
                * INTAKE_BOTTOM_ROLLER_GEAR_RATIO
                * INTAKE_DRIVETRAIN_SPEED_RATIO)
            / (Math.PI * INTAKE_ROLLER_DIAMETER);
  }

  public static class SerializationConstants {
    public static final int SERIALIZATION_FEEDER_LEADER_ID = 41;
    public static final int SERIALIZATION_FEEDER_FOLLOWER_ID = 42;
    public static final int SERIALIZATION_OMNI_ID = 40;
    public static final double SERIALIZATION_OMNI_SPEED = 0;
    public static final double SERIALIZATION_FEEDER_SPEED = 0;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_LEADER_ID = 50;
    public static final int SHOOTER_FOLLOWER_ID = 51;
    public static final int HOOD_MOTOR_ID = 52;
    public static final int TURRET_MOTOR_ID = 53;
    public static final double HOOD_MIN_POS = 0;
    public static final double HOOD_MAX_POS = 0;
    public static final Translation2d BLUE_HUB_POS = new Translation2d(4.61, 4.021);
    public static final Translation2d RED_HUB_POS = new Translation2d(11.9, 4.021);
    public static final Translation2d BLUE_LEFT_BUMP = new Translation2d(4.61, 6.03);
    public static final Translation2d BLUE_RIGHT_BUMP = new Translation2d(4.61, 2.01);
    public static final Translation2d RED_LEFT_BUMP = new Translation2d(11.9, 2.01);
    public static final Translation2d RED_RIGHT_BUMP = new Translation2d(11.9, 6.03);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
    public static final double SINGLE_TAG_AMBIGUITY_CUTOFF = 0.5;
    public static final double SINGLE_TAG_DISTANCE_CUTOFF = 5;
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
    public static final int ENCODER_ONE_TEETH = 17;
    public static final int ENCODER_TWO_TEETH = 18;
    public static final int TURRET_GEAR_TEETH = 92;
    public static final int MOTOR_TURRET_GEAR_RATIO = 46;
    public static final int ENCODER_TWO_ID = 54;
    public static final int ENCODER_ONE_ID = 55;
    public static final double CRT_EPSILON = 0.01;
    public static final double SHOOTER_OFFSET_X = 0;
    public static final double SHOOTER_OFFSET_Y = 0;
    public static final double HANG_TIME = 1.5;
    public static final double MOTOR_HOOD_GEAR_RATIO = 0;
    public static final double HOOD_COLLISION_TIME = 0;
  }

  public static class FieldConstants {
    public static final Distance FIELD_X = Inches.of(650.12);
    public static final Distance FIELD_Y = Inches.of(316.64);
    public static final double NZ_MID_LINE_X = FIELD_X.in(Meters) / 2;
    public static final double NZ_MID_LINE_Y = FIELD_Y.in(Meters) / 2;

    public static final Translation2d BLUE_HUB_POS = new Translation2d(4.61, NZ_MID_LINE_Y);
    public static final Translation2d RED_HUB_POS = new Translation2d(11.9, NZ_MID_LINE_Y);
    public static final Translation2d BLUE_LEFT_BUMP = new Translation2d(BLUE_HUB_POS.getX(), 6.03);
    public static final Translation2d BLUE_RIGHT_BUMP = new Translation2d(BLUE_HUB_POS.getX(), 1);
    public static final Translation2d RED_LEFT_BUMP = new Translation2d(RED_HUB_POS.getX(), 2.01);
    public static final Translation2d RED_RIGHT_BUMP = new Translation2d(RED_HUB_POS.getX(), 6.03);

    public static final double NZ_RED_X = RED_HUB_POS.getX();
    public static final double NZ_BLUE_X = BLUE_HUB_POS.getX();
    public static final double TRENCH_WIDTH = Inches.of(50.34).in(Meters);
    public static final Translation2d RED_LEFT_TRENCH_P1 =
        new Translation2d(RED_HUB_POS.getX(), FIELD_Y.in(Meters));
    public static final Translation2d RED_LEFT_TRENCH_P2 =
        new Translation2d(RED_HUB_POS.getX(), FIELD_Y.in(Meters) - TRENCH_WIDTH);
    public static final Translation2d RED_RIGHT_TRENCH_P1 =
        new Translation2d(RED_HUB_POS.getX(), 0);
    public static final Translation2d RED_RIGHT_TRENCH_P2 =
        new Translation2d(RED_HUB_POS.getX(), TRENCH_WIDTH);
    public static final Translation2d BLUE_RIGHT_TRENCH_P1 =
        new Translation2d(BLUE_HUB_POS.getX(), 0);
    public static final Translation2d BLUE_RIGHT_TRENCH_P2 =
        new Translation2d(BLUE_HUB_POS.getX(), TRENCH_WIDTH);
    public static final Translation2d BLUE_LEFT_TRENCH_P1 =
        new Translation2d(BLUE_HUB_POS.getX(), FIELD_Y.in(Meters));
    public static final Translation2d BLUE_LEFT_TRENCH_P2 =
        new Translation2d(BLUE_HUB_POS.getX(), FIELD_Y.in(Meters) - TRENCH_WIDTH);
  }
}

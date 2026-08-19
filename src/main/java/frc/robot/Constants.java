// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

  public static class ClimbConstants {
    public static final Translation2d CLIMB_POS_LEFT = new Translation2d(4.7, 0);
    public static final Translation2d CLIMB_POS_RIGHT = new Translation2d(4.7, 0);
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

    public static final double TURN_P = 2.65;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0.15;
    public static final double SINGLE_TAG_AMBIGUITY_CUTOFF = 0.5;
    public static final double SINGLE_TAG_DISTANCE_CUTOFF = 5;

    public static final double ROBOT_LATENCY = 0;

    public static final double MOVEMENT_THRESHOLD = 0.01;
  }

  public static class IntakeConstants {
    public static final int ARM_CAN_ID = 100;
    public static final int LEADER_CAN_ID = 100;
    public static final int FOLLOWER_CAN_ID = 100;

    public static final PositionVoltage ARM_STOW_SETPOINT = new PositionVoltage(0);
    public static final PositionVoltage ARM_DEPLOY_SETPOINT = new PositionVoltage(0);
    public static final double INTAKE_STOW_SPEED = 0;
    public static final double INTAKE_ROLLER_CIRCUMFIRENCE = 0;
    public static final double INTAKE_MOTOR_GEAR_RATIO = 0;
  }

  public static class SerializationConstants {
    public static final int OMNI_CAN_ID = 100;
    public static final int MECANUM_LEADER_CAN_ID = 100;
    public static final int MECANUM_FOLLOWER_CAN_ID = 100;

    public static final double OMNI_SPEED = 0;
    public static final double MECANUM_SPEED = 0;
    public static final double OMNI_REST_SPEED = 0;
    public static final double MECANUM_REST_SPEED = 0;
  }

  public static class ShooterConstants {

    public static final int HOOD_CAN_ID = 100;
    public static final int FLYWHEEL_LEADER_CAN_ID = 100;
    public static final int FLYWHEEL_FOLLOWER_CAN_ID = 100;
    public static final int TURRET_CAN_ID = 100;

    public static final double TURRET_THRESHOLD = 0.01;
    public static final double HOOD_THRESHOLD = 0.01;
    public static final double SHOOTER_THRESHOLD = 0.01;

    public static final AngularVelocity FLYWHEEL_REST_SPEED = RotationsPerSecond.of(0);

    public static final Angle HOOD_MAX_ANGLE = Degrees.of(0);
    public static final Angle HOOD_MINIMUM_ANGLE = Degrees.of(0);

    public static final Angle TURRET_MAX_ANGLE = Degrees.of(0);
    public static final Angle TURRET_MINIMUM_ANGLE = Degrees.of(0);

    public static final Distance SHOOTER_OFFSET_X = Meters.of(0);
    public static final Distance SHOOTER_OFFSET_Y = Meters.of(0);
    public static final Distance SHOOTER_OFFSET_Z = Meters.of(0);

    public static final Distance SHOOTER_CENTER_OFFSET =
        Meters.of(
            Math.sqrt(
                Math.pow(SHOOTER_OFFSET_X.in(Meters), 2)
                    + Math.pow(SHOOTER_OFFSET_Y.in(Meters), 2)));

    public static final Distance BIG_ROLLER_RADIUS = Inches.of(2);

    public static final double EXIT_VELOCITY_TO_MECHANISM_VELOCITY_SCALAR =
        (4.0 / 3) / (2 * Math.PI * BIG_ROLLER_RADIUS.in(Meters));

    public static final LoggedNetworkNumber AIMUTIL_SHOOTER_SPEED_ADDEND =
        new LoggedNetworkNumber("/Tuning/shooterSpeedFudger", 0);
    public static final LoggedNetworkNumber AIMUTIL_SHOOTER_SPEED_SCALAR =
        new LoggedNetworkNumber("/Tuning/shooterSpeedScalar", 0);
    public static final LoggedNetworkNumber AIMUTIL_HOOD_ANGLE_ADDEND =
        new LoggedNetworkNumber("/Tuning/hoodAngleFudger", 0);
    public static final LoggedNetworkNumber AIMUTIL_HOOD_ANGLE_SCALAR =
        new LoggedNetworkNumber("/Tuning/hoodAngleScalar", 0);
  }

  public static class FieldConstants {
    public static final double BLUE_ZONE_X = 3.964;
    public static final double RED_ZONE_X = 12.549;
    public static final LoggedNetworkNumber MAX_BALL_Y_POS =
        new LoggedNetworkNumber("Tuning/maxBallYPos", 3.0);
    // TODO for comp
    // = new LoggedNetworkNumber("Tuning/maxBallYPos", 2.7);
    public static final double HUB_Y_POS = 1.83;
    public static final double GRAVITY_VALUE = 9.80665;
    public static final DoubleSupplier HUB_HANG_TIME =
        () ->
            (Math.sqrt(
                        (MAX_BALL_Y_POS.getAsDouble()
                                - ShooterConstants.SHOOTER_OFFSET_Z.in(Meters))
                            * 2)
                    + Math.sqrt(
                        2
                            * ((MAX_BALL_Y_POS.getAsDouble()
                                    - ShooterConstants.SHOOTER_OFFSET_Z.in(Meters))
                                - (HUB_Y_POS - ShooterConstants.SHOOTER_OFFSET_Z.in(Meters)))))
                / Math.sqrt(GRAVITY_VALUE);
    // we're only playing on andymark field so we can just use these
    public static final Distance FIELD_X = Inches.of(650.12);
    public static final Distance FIELD_Y = Inches.of(316.64);
    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_X.div(2), FIELD_Y.div(2));
    public static final Translation2d BLUE_HUB_COORDINATES = new Translation2d(4.619, 4.049);
    public static final Translation2d RED_HUB_COORDINATES = new Translation2d(11.925, 4.049);
    // Depot
    public static final Pose2d BLUE_DEPOT_CENTER =
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    public static final Pose2d RED_DEPOT_CENTER =
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));

    // Blue Tower
    public static final Pose2d BLUE_TOWER_LEFT =
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_TOWER_RIGHT =
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_TOWER_CLIMB_LEFT =
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_TOWER_CLIMB_RIGHT =
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));

    // Red Tower
    public static final Pose2d RED_TOWER_LEFT =
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    public static final Pose2d RED_TOWER_RIGHT =
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    public static final Pose2d RED_TOWER_CLIMB_LEFT =
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    public static final Pose2d RED_TOWER_CLIMB_RIGHT =
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));

    // Passing Locations
    public static final Translation2d BLUE_AZ_PASS_LEFT = new Translation2d(0, 0);
    public static final Translation2d BLUE_AZ_PASS_RIGHT = new Translation2d(0, 0);
    public static final Translation2d RED_AZ_PASS_LEFT = new Translation2d(0, 0);
    public static final Translation2d RED_AZ_PASS_RIGHT = new Translation2d(0, 0);
    public static final Translation2d BLUE_NZ_PASS_LEFT = new Translation2d(0, 0);
    public static final Translation2d BLUE_NZ_PASS_RIGHT = new Translation2d(0, 0);
    public static final Translation2d RED_NZ_PASS_LEFT = new Translation2d(0, 0);
    public static final Translation2d RED_NZ_PASS_RIGHT = new Translation2d(0, 0);

    // meters
    public static final double HALF_FIELD_Y_POS = 4.022;
  }
}

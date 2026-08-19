// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Intake {
    public static final int LEADER_ROLLER_ID = 0;
    public static final int FOLLOWER_ROLLER_ID = 0;
    public static final int ARM_MOTOR_ID = 0;

    public static final double ARM_STOW_POSITION = 0;
    public static final double ARM_DEPLOY_POSITION = 0;

    public static final double ROLLER_SPEED = 1;
    // there are multiple roller sizes
    // this is the radius of the one that is farthest out
    public static final Distance OUTER_ROLLER_RADIUS = Inches.of(1.375 / 2);
  }

  public static class Serialization {
    public static final int OMNI_WHEEL_MOTOR_ID = 0;
    public static final int LEADER_MECANUM_ROLLER_ID = 0;
    public static final int FOLLOWER_MECANUM_ROLLER_ID = 0;

    public static final Distance MECANUM_ROLLER_RADIUS = Inches.of(2.5);
    public static final Distance OMNI_WHEEL_RADIUS = Inches.of(6);

    // ball entry speed is multiplied by this
    public static final double MECANUM_ROLLER_SPEED_SCALAR = 1.1;
    public static final double OMNI_WHEEL_SPEED_SCALAR = 1.2;
  }

  public static class Shooter {
    public static final int SHOOTER_MOTOR_LEADER_ID = 0;
    public static final int SHOOTER_MOTOR_FOLLOWER_ID = 0;
    public static final int HOOD_MOTOR_ID = 0;
    public static final int TURRET_MOTOR_ID = 0;

    // TODO
    // 3 inches left, 4 inches back, approx 20 inches up
    public static final Distance SHOOTER_OFFSET_X = Meters.of(-4);
    public static final Distance SHOOTER_OFFSET_Y = Meters.of(3);
    public static final Distance SHOOTER_OFFSET_Z = Meters.of(20);

    // TODO
    public static final Angle LOWERED_HOOD_POSITION = Degrees.of(0);

    public static final Distance SHOOTER_DISTANCE_FROM_CENTER = Meters.of(Math
        .sqrt(Math.pow(SHOOTER_OFFSET_X.in(Meters), 2) + Math.pow(SHOOTER_OFFSET_Y.in(Meters), 2)));

    public static final Distance BIG_ROLLER_RADIUS = Inches.of(2);

    // ball velocity multiplied by this to get rot/s of big roller
    public static final double EXIT_VELOCITY_TO_MECHANISM_VELOCITY_SCALAR =
        (4.0 / 3) / (2 * Math.PI * BIG_ROLLER_RADIUS.in(Meters));

    // in m/s exit velocity
    public static final LoggedNetworkNumber AIMUTIL_SHOOTER_SPEED_ADDEND =
        new LoggedNetworkNumber("/Tuning/shooterSpeedFudger", 0);
    public static final LoggedNetworkNumber AIMUTIL_SHOOTER_SPEED_SCALAR =
        new LoggedNetworkNumber("/Tuning/shooterSpeedScalar", 0);
    public static final LoggedNetworkNumber AIMUTIL_HOOD_ANGLE_ADDEND =
        new LoggedNetworkNumber("/Tuning/hoodAngleFudger", 0);
    public static final LoggedNetworkNumber AIMUTIL_HOOD_ANGLE_SCALAR =
        new LoggedNetworkNumber("/Tuning/hoodAngleScalar", 0);

    public static final double SHOOTER_ALLOWED_ERROR = 0.2;
    public static final double HOOD_ALLOWED_ERROR = 0.3;
    public static final double TURRET_ALLOWED_ERROR = 0.2;

    public static final int ENCODER_ONE_TEETH = 17;
    public static final int ENCODER_TOW_TEETH = 18;
    public static final int TURRET_GEAR_TEETH = 92;
  }

  public static class Climb {
    public static final int CLIMB_MOTOR_ID = 0;

    public static final double CLIMB_EXTENDED_SETPOINT = 0;
    public static final double CLIMB_RETRACTED_SETPOINT = 0;
  }

  public static class Drive {
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    // assume all modules equidistant from center
    public static final Distance MODULE_CENTER_DIST =
        Meters.of(Math.sqrt(Math.pow(TunerConstants.FrontLeft.LocationX, 2)
            + Math.pow(TunerConstants.FrontLeft.LocationY, 2)));
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond
        .of(MAX_SPEED.div(MODULE_CENTER_DIST.in(Meters) * 2 * Math.PI).in(MetersPerSecond));

    public static final double MAX_DRIVE_SPEED_SCALAR = 0.8;
    public static final double MAX_ROTATION_SPEED_SCALAR = 0.5;

    public static final double ROBOT_LATENCY = 0;
  }

  public static class Field {
    public static final double GRAVITY_VALUE = 9.81;

    public static final LoggedNetworkNumber MAX_BALL_Y_POS =
        new LoggedNetworkNumber("/Tuning/maxBallYPos", 2.7);
  }
}

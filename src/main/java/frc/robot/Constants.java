// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.controls.PositionVoltage;
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

  public static class ShooterConstants {}
}

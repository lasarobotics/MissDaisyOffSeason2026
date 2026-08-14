// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

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
  }

  public static class IntakeConstants {
    public static final int INTAKE_ROLLER_LEADER_ID = 10;
    public static final int INTAKE_ROLLER_FOLLOWER_ID = 11;
    public static final int INTAKE_SLAPDOWN_ID = 12;
    public static final double SLAPDOWN_POS = 0;
    public static final double INTAKE_ROLLER_SPEED = 0;
  }

  public static class SerializationConstants {
    public static final int SERIALIZATION_FEEDER_LEADER_ID = 20;
    public static final int SERIALIZATION_FEEDER_FOLLOWER_ID = 21;
    public static final int SERIALIZATION_OMNI_ID = 22;
    public static final double SERIALIZATION_OMNI_SPEED = 0;
    public static final double SERIALIZATION_FEEDER_SPEED = 0;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_LEADER_ID = 30;
    public static final int SHOOTER_FOLLOWER_ID = 31;
    public static final int HOOD_MOTOR_ID = 32;
    public static final double HOOD_MIN_POS = 0;
    public static final double HOOD_MAX_ANGLE = 0;
  }

  public static class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 40;
    public static final int SERIALIZATION_FEEDER_LEADER_ID = 20;
    public static final int SERIALIZATION_OMNI_ID = 22;
    public static final double SERIALIZATION_OMNI_SPEED = 0;
    public static final double SERIALIZATION_FEEDER_SPEED = 0;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeStates;
import frc.robot.subsystems.serialization.SerializationSubsystem;
import frc.robot.subsystems.serialization.SerializationSubsystem.SerializationStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterStates;
import java.util.function.BooleanSupplier;

public class HeadHoncho extends StateMachine {

  public enum HeadHonchoStates implements SystemState {
    REST {
      @Override
      public void initialize() {
        getInstance().stopBallFlow();
        DriveSubsystem.setState(DriveStates.DRIVER_CONTROL);
      }

      @Override
      public SystemState nextState() {
        if (getInstance().m_activeToggleButton.getAsBoolean()) {
          return ACTIVE;
        }

        return this;
      }
    },
    ACTIVE {
      @Override
      public void initialize() {
        DriveSubsystem.setState(DriveStates.DRIVER_CONTROL);
      }

      @Override
      public void execute() {
        if (getInstance().shouldShoot()) {
          getInstance().startBallFlow();
        } else {
          getInstance().stopBallFlow();
        }
      }

      @Override
      public SystemState nextState() {
        if (getInstance().m_activeToggleButton.getAsBoolean()) {
          return REST;
        }

        return this;
      }
    }
  }

  private static HeadHoncho s_headHoncho;

  private BooleanSupplier m_activeToggleButton;
  private BooleanSupplier m_climbButton;
  private BooleanSupplier m_climbAlignButton;

  public HeadHoncho() {
    super(HeadHonchoStates.REST);
  }

  public static HeadHoncho getInstance() {
    if (s_headHoncho == null) {
      s_headHoncho = new HeadHoncho();
    }
    return s_headHoncho;
  }

  public void configureBindings(
      BooleanSupplier activeToggleButton,
      BooleanSupplier climbButton,
      BooleanSupplier climbAlignButton) {
    m_activeToggleButton = activeToggleButton;
    m_climbButton = climbButton;
    m_climbAlignButton = climbAlignButton;
  }

  private void startBallFlow() {
    IntakeSubsystem.setState(IntakeStates.INTAKING);
    SerializationSubsystem.setState(SerializationStates.SERIALIZING);
    ShooterSubsystem.setState(ShooterStates.SHOOTING);
  }

  private void stopBallFlow() {
    IntakeSubsystem.setState(IntakeStates.INACTIVE);
    SerializationSubsystem.setState(SerializationStates.REST);
    ShooterSubsystem.setState(ShooterStates.REST);
  }

  public boolean shouldShoot() {
    // return true when in AZ and can shoot
    return false;
  }

  public static AngularVelocity getDesiredShooterSpeed() {
    return RotationsPerSecond.of(
        AimUtil.getBallVelocity().in(MetersPerSecond)
            * Constants.Shooter.EXIT_VELOCITY_TO_MECHANISM_VELOCITY_SCALAR);
  }

  public static Angle getDesiredHoodAngle() {
    return AimUtil.getExitAngle();
  }

  public static Angle getDesiredTurretAngle() {
    // TODO
    return Degrees.of(0);
  }
}

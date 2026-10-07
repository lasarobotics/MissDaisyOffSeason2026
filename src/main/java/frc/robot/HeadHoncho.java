// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
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
import org.littletonrobotics.junction.Logger;

public class HeadHoncho extends StateMachine {

  public enum HeadHonchoStates implements SystemState {
    AUTO {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().setState(DriveStates.REST);
        ShooterSubsystem.getInstance().setState(ShooterStates.ON);
        IntakeSubsystem.getInstance().setState(IntakeStates.INTAKE);
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isAutonomous()) {
          return AUTO;
        }
        return REST;
      }
    },

    ZERO {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().setState(DriveStates.REST);
        ShooterSubsystem.getInstance().setState(ShooterStates.ZERO);
        IntakeSubsystem.getInstance().setState(IntakeStates.ZERO);
        SerializationSubsystem.getInstance().setState(SerializationStates.REST);
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isAutonomous()) {
          return AUTO;
        }

        if (getInstance().m_zeroToggle.getAsBoolean() && !getInstance().finishedZeroing()
            || getInstance().isZeroing()) {
          return this;
        }

        if (getInstance().m_activeToggle.getAsBoolean()) {
          return TOGGLE_ON;
        }
        return REST;
      }
    },

    REST {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().setState(DriveStates.DRIVER_CONTROL);
        ShooterSubsystem.getInstance().setState(ShooterStates.OFF);
        IntakeSubsystem.getInstance().setState(IntakeStates.REST);
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isAutonomous()) {
          return AUTO;
        }

        if (getInstance().m_zeroToggle.getAsBoolean() && !getInstance().finishedZeroing()
            || getInstance().isZeroing()) {
          return this;
        }

        if (getInstance().m_activeToggle.getAsBoolean()) {
          return TOGGLE_ON;
        }
        return REST;
      }
    },
    TOGGLE_ON {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().setState(DriveStates.DRIVER_CONTROL);
      }

      @Override
      public void execute() {
        if (!( // ShooterSubsystem.getInstance().isReadyToShoot() &&
        !ShooterSubsystem.getInstance().robotCrossTrench()
            && !DriveSubsystem.getInstance().underTower()
            && ShooterSubsystem.getInstance().canSeeTarget()
            && !(ShooterSubsystem.getInstance().inAZ() && !GameHelpers.isHubActive()))) {
          ShooterSubsystem.getInstance().setState(ShooterStates.OFF);

          IntakeSubsystem.getInstance().setState(IntakeStates.REST);
          SerializationSubsystem.getInstance().setState(SerializationStates.REST);
        } else {
          ShooterSubsystem.getInstance().setState(ShooterStates.ON);
          IntakeSubsystem.getInstance().setState(IntakeStates.INTAKE);
          SerializationSubsystem.getInstance().setState(SerializationStates.ACTIVE);
        }
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isAutonomous()) {
          return AUTO;
        }

        if (getInstance().m_zeroToggle.getAsBoolean() && !getInstance().finishedZeroing()
            || getInstance().isZeroing()) {
          return this;
        }

        if (!getInstance().m_activeToggle.getAsBoolean()) {
          return REST;
        }
        return TOGGLE_ON;
      }
    },
    REVERSE {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().setState(DriveStates.DRIVER_CONTROL);
        ShooterSubsystem.getInstance().setState(ShooterStates.OFF);
        IntakeSubsystem.getInstance().setState(IntakeStates.REVERSE);
        SerializationSubsystem.getInstance().setState(SerializationStates.REVERSE);
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isAutonomous()) {
          return AUTO;
        }
        if (getInstance().m_zeroToggle.getAsBoolean() && !getInstance().finishedZeroing()
            || getInstance().isZeroing()) {
          return this;
        }
        if (getInstance().m_activeToggle.getAsBoolean()) {
          return TOGGLE_ON;
        }
        return REST;
      }
    }
  }

  private static HeadHoncho s_headHoncho;
  private BooleanSupplier m_activeToggle;
  private BooleanSupplier m_reverseButton;
  private BooleanSupplier m_zeroToggle;

  public HeadHoncho() {
    super(HeadHonchoStates.REST); // TODO switch to auto
  }

  public boolean finishedZeroing() {
    if (IntakeSubsystem.getInstance().finishedZero()
        && ShooterSubsystem.getInstance().finishedZero()) {
      getInstance().m_zeroToggle = () -> false;
      IntakeSubsystem.getInstance().setFinishedZero(false);
      ShooterSubsystem.getInstance().setFinishedZero(false);
      IntakeSubsystem.getInstance().setIsZeroing(false);
      ShooterSubsystem.getInstance().setIsZeroing(false);
      return true;
    }
    return false;
  }

  public boolean isZeroing() {
    if (ShooterSubsystem.getInstance().isZeroing() || IntakeSubsystem.getInstance().isZeroing()) {
      return true;
    }
    return false;
  }

  public void configureBindings(
      BooleanSupplier activeToggle, BooleanSupplier reverse, BooleanSupplier zeroIntake) {
    getInstance().m_activeToggle = activeToggle;
    getInstance().m_reverseButton = reverse;
    getInstance().m_zeroToggle = zeroIntake;
  }

  public static HeadHoncho getInstance() {
    if (s_headHoncho == null) {
      s_headHoncho = new HeadHoncho();
    }
    return s_headHoncho;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("HeadHoncho/currentState", getState().toString());
    Logger.recordOutput("HeadHoncho/activeToggle", m_activeToggle);
    Logger.recordOutput("HeadHoncho/reverse", m_reverseButton);
    Logger.recordOutput("Field/BLUE_HUB_POS", Constants.Field.BLUE_HUB_POS);
    Logger.recordOutput("Field/BLUE_LEFT_BUMP", Constants.Field.BLUE_LEFT_BUMP);
    Logger.recordOutput("Field/BLUE_RIGHT_BUMP", Constants.Field.BLUE_RIGHT_BUMP);
    Logger.recordOutput("Field/RED_HUB_POS", Constants.Field.RED_HUB_POS);
    Logger.recordOutput("Field/RED_RIGHT_BUMP", Constants.Field.RED_RIGHT_BUMP);
    Logger.recordOutput("Field/RED_RIGHT_BUMP", Constants.Field.RED_RIGHT_BUMP);
    Logger.recordOutput(
        "HeadHoncho/ballChecksPass",
        // ShooterSubsystem.getInstance().isReadyToShoot() && TODO
        !ShooterSubsystem.getInstance().robotCrossTrench()
            && !DriveSubsystem.getInstance().underTower()
            && ShooterSubsystem.getInstance().canSeeTarget()
            && !(ShooterSubsystem.getInstance().inAZ() && !GameHelpers.isHubActive()));

    Logger.recordOutput("HeadHoncho/isHubActive", GameHelpers.isHubActive());
    Logger.recordOutput(
        "HeadHoncho/crossTrench", !ShooterSubsystem.getInstance().robotCrossTrench());
    Logger.recordOutput("HeadHoncho/underTower", !DriveSubsystem.getInstance().underTower());
    Logger.recordOutput("HeadHoncho/canSeeTarget", ShooterSubsystem.getInstance().canSeeTarget());
    Logger.recordOutput(
        "HeadHoncho/shifts",
        !(ShooterSubsystem.getInstance().inAZ() && !GameHelpers.isHubActive()));
  }
}

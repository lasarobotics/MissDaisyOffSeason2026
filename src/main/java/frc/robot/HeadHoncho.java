// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return this;
      }
    },

    REST {
      @Override
      public void initialize() {
        getInstance().restRobot();
      }

      @Override
      public SystemState nextState() {
        if (getInstance().wantsToDriveUnwind()) {
          return DRIVE_UNWIND;
        }
        if (getInstance().wantToActive()) {
          return ACTIVE;
        }
        if (getInstance().wantToReverse()) {
          return REVERSE;
        }
        return this;
      }
    },

    ACTIVE {
      @Override
      public void initialize() {
        getInstance().activeRobot();
      }

      @Override
      public SystemState nextState() {
        if (getInstance().wantsToDriveUnwind()) {
          return DRIVE_UNWIND;
        }
        if (getInstance().wantToActive()) {
          return ACTIVE;
        }
        if (getInstance().wantToReverse()) {
          return REVERSE;
        }
        return REST;
      }
    },

    REVERSE {
      @Override
      public void initialize() {
        getInstance().reverseRobot();
      }

      @Override
      public SystemState nextState() {
        if (getInstance().wantsToDriveUnwind()) {
          return DRIVE_UNWIND;
        }
        if (getInstance().wantToActive()) {
          return ACTIVE;
        }
        if (getInstance().wantToReverse()) {
          return REVERSE;
        }
        return REST;
      }
    },

    DRIVE_UNWIND {
      @Override
      public void initialize() {
        getInstance().driveUnwindRobot();
      }

      @Override
      public SystemState nextState() {
        if (getInstance().wantsToDriveUnwind()) {
          return DRIVE_UNWIND;
        }
        if (getInstance().wantToActive()) {
          return ACTIVE;
        }
        if (getInstance().wantToReverse()) {
          return REVERSE;
        }
        return REST;
      }
    }
  }

  private static HeadHoncho s_headHoncho;
  private static DriveSubsystem s_driveSubsystem;
  private static IntakeSubsystem s_intakeSubsystem;
  private static SerializationSubsystem s_serializationSubsystem;
  private static ShooterSubsystem s_shooterSubsystem;

  private BooleanSupplier m_active;
  private BooleanSupplier m_reverseActive;

  private boolean m_driveUnwindRequested;

  public HeadHoncho() {
    super(HeadHonchoStates.ACTIVE);

    m_driveUnwindRequested = false;

    s_driveSubsystem = DriveSubsystem.getInstance();
    s_intakeSubsystem = IntakeSubsystem.getInstance();
    s_serializationSubsystem = SerializationSubsystem.getInstance();
    s_shooterSubsystem = ShooterSubsystem.getInstance();
  }

  public static HeadHoncho getInstance() {
    if (s_headHoncho == null) {
      s_headHoncho = new HeadHoncho();
    }
    return s_headHoncho;
  }

  public void configureBindings(BooleanSupplier activeButton, BooleanSupplier reverseActive) {
    getInstance().m_active = activeButton;
    getInstance().m_reverseActive = reverseActive;
  }

  public void requestDriveUnwind() {
    getInstance().m_driveUnwindRequested = true;
  }

  public void driveUnwindEnded() {
    getInstance().m_driveUnwindRequested = false;
  }

  public boolean wantToActive() {
    return getInstance().m_active.getAsBoolean();
  }

  public boolean wantToReverse() {
    return getInstance().m_reverseActive.getAsBoolean();
  }

  public boolean wantsToDriveUnwind() {
    return getInstance().m_driveUnwindRequested;
  }

  public boolean numberWithinThreshold(double target, double value, double threshold) {
    if (value - threshold > target || value - threshold < target) {
      return false;
    }
    return true;
  }

  public void restRobot() {
    s_driveSubsystem.setState(DriveStates.DRIVER_CONTROL);
    s_intakeSubsystem.setState(IntakeStates.STOW);
    s_serializationSubsystem.setState(SerializationStates.REST);
    s_shooterSubsystem.setState(ShooterStates.REST);
  }

  public void activeRobot() {
    s_driveSubsystem.setState(DriveStates.DRIVER_CONTROL);
    s_intakeSubsystem.setState(IntakeStates.INTAKE);
    s_serializationSubsystem.setState(SerializationStates.ACTIVE);
    s_shooterSubsystem.setState(ShooterStates.SHOOT);
  }

  public void reverseRobot() {
    s_driveSubsystem.setState(DriveStates.DRIVER_CONTROL);
    s_intakeSubsystem.setState(IntakeStates.REVERSE);
    s_serializationSubsystem.setState(SerializationStates.REVERSE);
    s_shooterSubsystem.setState(ShooterStates.SHOOT);
  }

  public void driveUnwindRobot() {
    s_driveSubsystem.setState(DriveStates.UNWIND);
    s_intakeSubsystem.setState(IntakeStates.INTAKE);
    s_serializationSubsystem.setState(SerializationStates.ACTIVE);
    s_shooterSubsystem.setState(ShooterStates.SHOOT);
  }

  @Override
  public void periodic() {
    // State Logging
    Logger.recordOutput("HeadHoncho/State", getState().toString());
    Logger.recordOutput("HeadHoncho/IsActive", wantToActive());
    Logger.recordOutput("HeadHoncho/IsReverse", wantToReverse());

    // Log all field values for verification
    Logger.recordOutput(
        "Field/BLUE_HUB_COORDINATES", Constants.FieldConstants.BLUE_HUB_COORDINATES);
    Logger.recordOutput("Field/RED_HUB_COORDINATES", Constants.FieldConstants.RED_HUB_COORDINATES);
    Logger.recordOutput("Field/BLUE_DEPOT_CENTER", Constants.FieldConstants.BLUE_DEPOT_CENTER);
    Logger.recordOutput("Field/RED_DEPOT_CENTER", Constants.FieldConstants.RED_DEPOT_CENTER);
    Logger.recordOutput("Field/BLUE_TOWER_LEFT", Constants.FieldConstants.BLUE_TOWER_LEFT);
    Logger.recordOutput("Field/BLUE_TOWER_RIGHT", Constants.FieldConstants.BLUE_TOWER_RIGHT);
    Logger.recordOutput(
        "Field/BLUE_TOWER_CLIMB_LEFT", Constants.FieldConstants.BLUE_TOWER_CLIMB_LEFT);
    Logger.recordOutput(
        "Field/BLUE_TOWER_CLIMB_RIGHT", Constants.FieldConstants.BLUE_TOWER_CLIMB_RIGHT);
    Logger.recordOutput("Field/RED_TOWER_LEFT", Constants.FieldConstants.RED_TOWER_LEFT);
    Logger.recordOutput("Field/RED_TOWER_RIGHT", Constants.FieldConstants.RED_TOWER_RIGHT);
    Logger.recordOutput(
        "Field/RED_TOWER_CLIMB_LEFT", Constants.FieldConstants.RED_TOWER_CLIMB_LEFT);
    Logger.recordOutput(
        "Field/RED_TOWER_CLIMB_RIGHT", Constants.FieldConstants.RED_TOWER_CLIMB_RIGHT);
    Logger.recordOutput("Field/BLUE_AZ_PASS_LEFT", Constants.FieldConstants.BLUE_AZ_PASS_LEFT);
    Logger.recordOutput("Field/BLUE_AZ_PASS_RIGHT", Constants.FieldConstants.BLUE_AZ_PASS_RIGHT);
    Logger.recordOutput("Field/RED_AZ_PASS_LEFT", Constants.FieldConstants.RED_AZ_PASS_LEFT);
    Logger.recordOutput("Field/RED_AZ_PASS_RIGHT", Constants.FieldConstants.RED_AZ_PASS_RIGHT);
    Logger.recordOutput("Field/BLUE_NZ_PASS_LEFT", Constants.FieldConstants.BLUE_NZ_PASS_LEFT);
    Logger.recordOutput("Field/BLUE_NZ_PASS_RIGHT", Constants.FieldConstants.BLUE_NZ_PASS_RIGHT);
    Logger.recordOutput("Field/RED_NZ_PASS_LEFT", Constants.FieldConstants.RED_NZ_PASS_LEFT);
    Logger.recordOutput("Field/RED_NZ_PASS_RIGHT", Constants.FieldConstants.RED_NZ_PASS_RIGHT);
    Logger.recordOutput("Field/HALF_FIELD_Y_POS", Constants.FieldConstants.HALF_FIELD_Y_POS);
  }
}

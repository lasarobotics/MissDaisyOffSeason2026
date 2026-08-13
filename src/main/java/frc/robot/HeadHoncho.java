// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.serialization.SerializationSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class HeadHoncho extends StateMachine {

  public enum HeadHonchoStates implements SystemState {
    REST {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_activeButton.getAsBoolean()) {
          return TOGGLE_ON;
        }
        if (getInstance().m_climbAlignButton.getAsBoolean()) {
          return CLIMB_ALIGN;
        }
        if (getInstance().m_climbButton.getAsBoolean()) {
          return CLIMB;
        }
        return REST;
      }
    },
    TOGGLE_ON {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_inactiveButton.getAsBoolean()) {
          return REST;
        }
        if (getInstance().m_climbAlignButton.getAsBoolean()) {
          return CLIMB_ALIGN;
        }
        if (getInstance().m_climbButton.getAsBoolean()) {
          return CLIMB;
        }
        return TOGGLE_ON;
      }
    },
    CLIMB_ALIGN {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return REST;
      }
    },
    CLIMB {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return REST;
      }
    },
  }

  private static HeadHoncho s_headHoncho;
  private static DriveSubsystem DRIVE_SUBSYSTEM = DriveSubsystem.getInstance();
  private static IntakeSubsystem INTAKE_SUBSYSTEM = IntakeSubsystem.getInstance();
  private static ShooterSubsystem SHOOTER_SUBSYSTEM = ShooterSubsystem.getInstance();
  private static SerializationSubsystem SERIALIZATION_SUBSYSTEM =
      SerializationSubsystem.getInstance();
  private BooleanSupplier m_climbAlignButton;
  private BooleanSupplier m_climbButton;
  private BooleanSupplier m_activeButton;
  private BooleanSupplier m_inactiveButton;

  public HeadHoncho() {
    super(HeadHonchoStates.REST);
    DRIVE_SUBSYSTEM = DriveSubsystem.getInstance();
    INTAKE_SUBSYSTEM = IntakeSubsystem.getInstance();
    SHOOTER_SUBSYSTEM = ShooterSubsystem.getInstance();
    SERIALIZATION_SUBSYSTEM = SerializationSubsystem.getInstance();
  }

  public void configureBindings(
      BooleanSupplier climbAlignButton,
      BooleanSupplier activeButton,
      BooleanSupplier inactiveButton,
      BooleanSupplier climbButton) {
    m_climbAlignButton = climbAlignButton;
    m_activeButton = activeButton;
    m_inactiveButton = inactiveButton;
    m_climbButton = climbButton;
  }

  public static HeadHoncho getInstance() {
    if (s_headHoncho == null) {
      s_headHoncho = new HeadHoncho();
    }
    return s_headHoncho;
  }

  @Override
  public void periodic() {}
}

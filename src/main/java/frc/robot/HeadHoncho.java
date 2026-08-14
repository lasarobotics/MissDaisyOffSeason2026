// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbStates;
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
        DriveSubsystem.getInstance().setState(DriveStates.DRIVER_CONTROL);
        ShooterSubsystem.getInstance().setState(ShooterStates.OFF);
        IntakeSubsystem.getInstance().setState(IntakeStates.OFF);
        ClimbSubsystem.getInstance().setState(ClimbStates.OFF);
      }

      @Override
      public SystemState nextState() {
        if (getInstance().m_activeToggle.getAsBoolean()) {
          return TOGGLE_ON;
        }
        if (getInstance().m_climbAlignButton.getAsBoolean()) {
          return CLIMB_ALIGN;
        }
        if (getInstance().m_climbToggle.getAsBoolean()) {
          return CLIMB;
        }
        return REST;
      }
    },
    TOGGLE_ON {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().setState(DriveStates.DRIVER_CONTROL);
        ShooterSubsystem.getInstance().setState(ShooterStates.ON);
        IntakeSubsystem.getInstance().setState(IntakeStates.ON);
        ClimbSubsystem.getInstance().setState(ClimbStates.OFF);
      }

      @Override
      public SystemState nextState() {
        if (!getInstance().m_activeToggle.getAsBoolean()) {
          return REST;
        }
        if (getInstance().m_climbAlignButton.getAsBoolean()) {
          return CLIMB_ALIGN;
        }
        if (getInstance().m_climbToggle.getAsBoolean()) {
          return CLIMB;
        }
        return TOGGLE_ON;
      }
    },
    CLIMB_ALIGN {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().setState(DriveStates.CLIMB_ALIGN);
        ShooterSubsystem.getInstance().setState(ShooterStates.OFF);
        IntakeSubsystem.getInstance().setState(IntakeStates.OFF);
        ClimbSubsystem.getInstance().setState(ClimbStates.OFF);
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_climbAlignButton.getAsBoolean()) {
          return CLIMB_ALIGN;
        }
        if (getInstance().m_activeToggle.getAsBoolean()) {
          return TOGGLE_ON;
        }
        return REST;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().setState(DriveStates.DRIVER_CONTROL);
        ShooterSubsystem.getInstance().setState(ShooterStates.OFF);
        IntakeSubsystem.getInstance().setState(IntakeStates.OFF);
        ClimbSubsystem.getInstance().setState(ClimbStates.ON);
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_climbToggle.getAsBoolean()) {
          return CLIMB;
        }
        if (getInstance().m_activeToggle.getAsBoolean()) {
          return TOGGLE_ON;
        }
        return REST;
      }
    },
    REVERSE {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().setState(DriveStates.DRIVER_CONTROL);
        ShooterSubsystem.getInstance().setState(ShooterStates.OFF);
        IntakeSubsystem.getInstance().setState(IntakeStates.REVERSE);
        SerializationSubsystem.getInstance().setState(SerializationStates.REVERSE);
        ClimbSubsystem.getInstance().setState(ClimbStates.ON);
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_climbToggle.getAsBoolean()) {
          return CLIMB;
        }
        if (getInstance().m_activeToggle.getAsBoolean()) {
          return TOGGLE_ON;
        }
        return REST;
      }
    }
  }

  private static HeadHoncho s_headHoncho;
  private static DriveSubsystem DRIVE_SUBSYSTEM = DriveSubsystem.getInstance();
  private static IntakeSubsystem INTAKE_SUBSYSTEM = IntakeSubsystem.getInstance();
  private static ShooterSubsystem SHOOTER_SUBSYSTEM = ShooterSubsystem.getInstance();
  private static SerializationSubsystem SERIALIZATION_SUBSYSTEM =
      SerializationSubsystem.getInstance();
  private BooleanSupplier m_climbAlignButton;
  private BooleanSupplier m_climbToggle;
  private BooleanSupplier m_activeToggle;
  private BooleanSupplier m_reverseButton;

  public HeadHoncho() {
    super(HeadHonchoStates.REST);
    DRIVE_SUBSYSTEM = DriveSubsystem.getInstance();
    INTAKE_SUBSYSTEM = IntakeSubsystem.getInstance();
    SHOOTER_SUBSYSTEM = ShooterSubsystem.getInstance();
    SERIALIZATION_SUBSYSTEM = SerializationSubsystem.getInstance();
  }

  public void configureBindings(
      BooleanSupplier climbAlignButton,
      BooleanSupplier activeToggle,
      BooleanSupplier climbToggle,
      BooleanSupplier reverse) {
    m_climbAlignButton = climbAlignButton;
    m_activeToggle = activeToggle;
    m_climbToggle = climbToggle;
    m_reverseButton = reverse;
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

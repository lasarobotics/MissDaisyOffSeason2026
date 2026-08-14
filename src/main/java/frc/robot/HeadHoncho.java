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

public class HeadHoncho extends StateMachine implements AutoCloseable {

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
        if (getInstance().wantToActive()) {
          return ACTIVE;
        }
        if (getInstance().wantToClimb()) {
          return CLIMB;
        }
        if (getInstance().wantToClimbPrepare()) {
          return CLIMB_PREPARE;
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
        if (getInstance().wantToActive()) {
          return ACTIVE;
        }
        if (getInstance().wantToClimb()) {
          return CLIMB;
        }
        if (getInstance().wantToClimbPrepare()) {
          return CLIMB_PREPARE;
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
        if (getInstance().wantToActive()) {
          return ACTIVE;
        }
        if (getInstance().wantToClimb()) {
          return CLIMB;
        }
        if (getInstance().wantToClimbPrepare()) {
          return CLIMB_PREPARE;
        }
        if (getInstance().wantToReverse()) {
          return REVERSE;
        }
        return REST;
      }
    },

    CLIMB_PREPARE {
      @Override
      public void initialize() {
        getInstance().climbPrepareRobot();
      }

      @Override
      public SystemState nextState() {
        if (getInstance().wantToActive()) {
          return ACTIVE;
        }
        if (getInstance().wantToClimb()) {
          return CLIMB;
        }
        if (getInstance().wantToClimbPrepare()) {
          return CLIMB_PREPARE;
        }
        if (getInstance().wantToReverse()) {
          return REVERSE;
        }
        return REST;
      }
    },

    CLIMB {
      @Override
      public void initialize() {
        getInstance().climbRobot();
      }

      @Override
      public SystemState nextState() {
        if (getInstance().wantToActive()) {
          return ACTIVE;
        }
        if (getInstance().wantToClimb()) {
          return CLIMB;
        }
        if (getInstance().wantToClimbPrepare()) {
          return CLIMB_PREPARE;
        }
        if (getInstance().wantToReverse()) {
          return REVERSE;
        }
        return REST;
      }
    }
  }

  private static HeadHoncho s_headHoncho;
  private static ClimbSubsystem s_climbSubsystem;
  private static DriveSubsystem s_driveSubsystem;
  private static IntakeSubsystem s_intakeSubsystem;
  private static SerializationSubsystem s_serializationSubsystem;
  private static ShooterSubsystem s_shooterSubsystem;

  private BooleanSupplier m_active;
  private BooleanSupplier m_climbPrepareButton;
  private BooleanSupplier m_climb;
  private BooleanSupplier m_reverseActive;

  public HeadHoncho() {
    super(HeadHonchoStates.ACTIVE);
    s_climbSubsystem = ClimbSubsystem.getInstance();
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

  public void configureBindings(
      BooleanSupplier activeButton,
      BooleanSupplier climbButton,
      BooleanSupplier reverseActive,
      BooleanSupplier climbPrepareButton) {
    getInstance().m_active = activeButton;
    getInstance().m_climbPrepareButton = climbPrepareButton;
    getInstance().m_climb = climbButton;
    getInstance().m_reverseActive = reverseActive;
  }

  public boolean wantToActive() {
    return getInstance().m_active.getAsBoolean();
  }

  public boolean wantToClimb() {
    return getInstance().m_climb.getAsBoolean();
  }

  public boolean wantToReverse() {
    return getInstance().m_reverseActive.getAsBoolean();
  }

  public boolean wantToClimbPrepare() {
    return getInstance().m_climbPrepareButton.getAsBoolean();
  }

  public void restRobot() {
    s_climbSubsystem.setState(ClimbStates.REST);
    s_driveSubsystem.setState(DriveStates.DRIVER_CONTROL);
    s_intakeSubsystem.setState(IntakeStates.STOW);
    s_serializationSubsystem.setState(SerializationStates.REST);
    s_shooterSubsystem.setState(ShooterStates.REST);
  }

  public void activeRobot() {
    s_climbSubsystem.setState(ClimbStates.REST);
    s_driveSubsystem.setState(DriveStates.DRIVER_CONTROL);
    s_intakeSubsystem.setState(IntakeStates.INTAKE);
    s_serializationSubsystem.setState(SerializationStates.ACTIVE);
    s_shooterSubsystem.setState(ShooterStates.SHOOT);
  }

  public void reverseRobot() {
    s_climbSubsystem.setState(ClimbStates.REST);
    s_driveSubsystem.setState(DriveStates.DRIVER_CONTROL);
    s_intakeSubsystem.setState(IntakeStates.REVERSE);
    s_serializationSubsystem.setState(SerializationStates.REVERSE);
    s_shooterSubsystem.setState(ShooterStates.SHOOT);
  }

  public void climbRobot() {
    s_climbSubsystem.setState(ClimbStates.CLIMB);
    s_driveSubsystem.setState(DriveStates.DRIVER_CONTROL);
    s_intakeSubsystem.setState(IntakeStates.STOW);
    s_serializationSubsystem.setState(SerializationStates.REST);
    s_shooterSubsystem.setState(ShooterStates.REST);
  }

  public void climbPrepareRobot() {
    s_climbSubsystem.setState(ClimbStates.READY);
    s_driveSubsystem.setState(DriveStates.CLIMB_ALIGN);
    s_intakeSubsystem.setState(IntakeStates.STOW);
    s_serializationSubsystem.setState(SerializationStates.REST);
    s_shooterSubsystem.setState(ShooterStates.REST);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void close() {}
}

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
    REST {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().setState(DriveStates.DRIVER_CONTROL);
        ShooterSubsystem.getInstance().setState(ShooterStates.OFF);
        IntakeSubsystem.getInstance().setState(IntakeStates.OFF);
      }

      @Override
      public SystemState nextState() {
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
        ShooterSubsystem.getInstance().setState(ShooterStates.ON);
        IntakeSubsystem.getInstance().setState(IntakeStates.ON);
      }

      @Override
      public SystemState nextState() {
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
      public void execute() {}

      @Override
      public SystemState nextState() {
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

  public HeadHoncho() {
    super(HeadHonchoStates.REST);
  }

  public void configureBindings(BooleanSupplier activeToggle, BooleanSupplier reverse) {
    m_activeToggle = activeToggle;
    m_reverseButton = reverse;
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
    Logger.recordOutput("Field/BLUE_HUB_POS", Constants.FieldConstants.BLUE_HUB_POS);
    Logger.recordOutput("Field/BLUE_LEFT_BUMP", Constants.FieldConstants.BLUE_LEFT_BUMP);
    Logger.recordOutput("Field/BLUE_RIGHT_BUMP", Constants.FieldConstants.BLUE_RIGHT_BUMP);
    Logger.recordOutput("Field/RED_HUB_POS", Constants.FieldConstants.RED_HUB_POS);
    Logger.recordOutput("Field/RED_RIGHT_BUMP", Constants.FieldConstants.RED_RIGHT_BUMP);
    Logger.recordOutput("Field/RED_RIGHT_BUMP", Constants.FieldConstants.RED_RIGHT_BUMP);
  }
}

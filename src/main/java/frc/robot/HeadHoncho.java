// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
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

  private final CommandXboxController PRIMARY_CONTROLLER =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private static final DriveSubsystem DRIVE_SUBSYSTEM = DriveSubsystem.getInstance();
  private static final IntakeSubsystem INTAKE_SUBSYSTEM = IntakeSubsystem.getInstance();
  private static final SerializationSubsystem SERIALIZATION_SUBSYSTEM =
      SerializationSubsystem.getInstance();
  private static final ShooterSubsystem SHOOTER_SUBSYSTEM = ShooterSubsystem.getInstance();
  private static final ClimbSubsystem CLIMB_SUBSYSTEM = ClimbSubsystem.getInstance();

  public enum HeadHonchoStates implements SystemState {
    CycleOff {
      @Override
      public void initialize() {
        INTAKE_SUBSYSTEM.setIntakeState(IntakeStates.CycleOff);
        SERIALIZATION_SUBSYSTEM.setSerializationState(SerializationStates.CycleOff);
        SHOOTER_SUBSYSTEM.setShooterState(ShooterStates.CycleOff);
        CLIMB_SUBSYSTEM.setClimbState(ClimbStates.REST);
        DRIVE_SUBSYSTEM.setDriveState(DriveStates.REST);
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_ballToggle.getAsBoolean() && !getInstance().m_cycleChangeBlock) {
          getInstance().m_cycleChangeBlock = true;
          return CycleOn;
        }
        if (getInstance().m_climbToggle.getAsBoolean() && !getInstance().m_climbChangeBlock) {
          getInstance().m_climbChangeBlock = true;
          getInstance().m_climbPreviousState = HeadHonchoStates.CycleOff;
          return Climbing;
        }
        if (getInstance().m_climbAlign.getAsBoolean()) {
          return Climb_Align;
        }
        if (getInstance().m_reverse.getAsBoolean()) {
          return Reverse;
        }
        return CycleOff;
      }
    },

    CycleOn {
      @Override
      public void initialize() {
        INTAKE_SUBSYSTEM.setIntakeState(IntakeStates.CycleOn);
        SERIALIZATION_SUBSYSTEM.setSerializationState(SerializationStates.CycleOn);
        SHOOTER_SUBSYSTEM.setShooterState(ShooterStates.CycleOn);
        CLIMB_SUBSYSTEM.setClimbState(ClimbStates.REST);
        DRIVE_SUBSYSTEM.setDriveState(DriveStates.DRIVER_CONTROL);
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_ballToggle.getAsBoolean() && !getInstance().m_cycleChangeBlock) {
          getInstance().m_cycleChangeBlock = true;
          return CycleOn;
        }
        if (getInstance().m_climbToggle.getAsBoolean() && !getInstance().m_climbChangeBlock) {
          getInstance().m_climbChangeBlock = true;
          getInstance().m_climbPreviousState = HeadHonchoStates.CycleOn;
          return Climbing;
        }
        if (getInstance().m_climbAlign.getAsBoolean()) {
          return Climb_Align;
        }
        if (getInstance().m_reverse.getAsBoolean()) {
          return Reverse;
        }
        return CycleOff;
      }
    },

    Reverse {
      @Override
      public void initialize() {
        INTAKE_SUBSYSTEM.setIntakeState(IntakeStates.Reverse);
        SERIALIZATION_SUBSYSTEM.setSerializationState(SerializationStates.Reverse);
        SHOOTER_SUBSYSTEM.setShooterState(ShooterStates.Reverse);
        CLIMB_SUBSYSTEM.setClimbState(ClimbStates.REST);
        DRIVE_SUBSYSTEM.setDriveState(DriveStates.DRIVER_CONTROL);
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_ballToggle.getAsBoolean() && !getInstance().m_cycleChangeBlock) {
          getInstance().m_cycleChangeBlock = true;
          return CycleOn;
        }
        if (getInstance().m_climbToggle.getAsBoolean() && !getInstance().m_climbChangeBlock) {
          getInstance().m_climbChangeBlock = true;
          getInstance().m_climbPreviousState = HeadHonchoStates.Reverse;
          return Climbing;
        }
        if (getInstance().m_climbAlign.getAsBoolean()) {
          return Climb_Align;
        }
        if (getInstance().m_reverse.getAsBoolean()) {
          return Reverse;
        }
        return CycleOn;
      }
    },

    Climb_Align {
      @Override
      public void initialize() {
        INTAKE_SUBSYSTEM.setIntakeState(IntakeStates.CycleOff);
        SERIALIZATION_SUBSYSTEM.setSerializationState(SerializationStates.CycleOff);
        SHOOTER_SUBSYSTEM.setShooterState(ShooterStates.CycleOff);
        CLIMB_SUBSYSTEM.setClimbState(ClimbStates.REST);
        DRIVE_SUBSYSTEM.setDriveState(DriveStates.CLIMB_ALIGN);
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_climbAlign.getAsBoolean()) {
          return Climb_Align;
        }
        return CycleOff;
      }
    },

    Climbing {
      @Override
      public void initialize() {
        INTAKE_SUBSYSTEM.setIntakeState(IntakeStates.CycleOff);
        SERIALIZATION_SUBSYSTEM.setSerializationState(SerializationStates.CycleOff);
        SHOOTER_SUBSYSTEM.setShooterState(ShooterStates.CycleOff);
        CLIMB_SUBSYSTEM.setClimbState(ClimbStates.CLIMB);
        DRIVE_SUBSYSTEM.setDriveState(DriveStates.REST);
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_climbToggle.getAsBoolean() && !getInstance().m_climbChangeBlock) {
          getInstance().m_climbChangeBlock = true;
          return getInstance().m_climbPreviousState;
        }
        if (getInstance().m_climbAlign.getAsBoolean()) {
          return Climb_Align;
        }
        return Climbing;
      }
    }
  }

  private static HeadHoncho s_headHoncho;
  private BooleanSupplier m_ballToggle;
  private BooleanSupplier m_climbToggle;
  private BooleanSupplier m_climbAlign;
  private BooleanSupplier m_reverse;
  private boolean m_cycleChangeBlock = false;
  private boolean m_climbChangeBlock = false;
  private SystemState m_climbPreviousState = null;

  public HeadHoncho() {
    super(HeadHonchoStates.CycleOff);

    configure_bindings();

    PRIMARY_CONTROLLER.rightBumper().onFalse(Commands.runOnce(() -> m_cycleChangeBlock = false));
    PRIMARY_CONTROLLER.leftBumper().onFalse(Commands.runOnce(() -> m_climbChangeBlock = false));
  }

  public static HeadHoncho getInstance() {
    if (s_headHoncho == null) {
      s_headHoncho = new HeadHoncho();
    }
    return s_headHoncho;
  }

  private void configure_bindings() {
    m_ballToggle = PRIMARY_CONTROLLER.rightBumper();
    m_climbAlign = PRIMARY_CONTROLLER.y();
    m_climbToggle = PRIMARY_CONTROLLER.leftBumper();
    m_reverse = PRIMARY_CONTROLLER.rightTrigger();

    DRIVE_SUBSYSTEM.configure_bindings(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> PRIMARY_CONTROLLER.getRightX());
  }

  @Override
  public void periodic() {}
}

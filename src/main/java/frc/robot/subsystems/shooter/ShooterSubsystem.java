// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends StateMachine {

  public enum ShooterStates implements SystemState {
    OFF {
      @Override
      public void initialize() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    },
    ON {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    }
  }

  private static ShooterSubsystem s_shooterInstance;
  private ShooterStates m_selectedState;
  private TalonFX m_shooterLeader;
  private TalonFX m_shooterFollower;
  private TalonFX m_hoodMotor;
  private VelocityVoltage m_velocityVoltage;
  private PositionVoltage m_positionVoltage;
  private TalonFXConfiguration m_shooterConfig;
  private TalonFXConfiguration m_hoodConfig;
  private TalonFX m_turretMotor;
  private TalonFXConfiguration m_turretConfig;

  public ShooterSubsystem() {
    super(ShooterStates.OFF);
    setState(ShooterStates.OFF);
    m_shooterLeader = new TalonFX(Constants.ShooterConstants.SHOOTER_LEADER_ID);
    m_shooterFollower = new TalonFX(Constants.ShooterConstants.SHOOTER_FOLLOWER_ID);
    m_hoodMotor = new TalonFX(Constants.ShooterConstants.HOOD_MOTOR_ID);
    m_turretMotor = new TalonFX(Constants.ShooterConstants.TURRET_MOTOR_ID);
    m_velocityVoltage = new VelocityVoltage(0);
    m_positionVoltage = new PositionVoltage(0);
    m_shooterFollower.setControl(
        new Follower(m_shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    m_shooterConfig = new TalonFXConfiguration();
    m_shooterConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_hoodConfig = new TalonFXConfiguration();
    m_hoodConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_turretConfig = new TalonFXConfiguration(); // TODO SET PID SV VALUES FOR ALL SUBSYSTEMS
    m_turretConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_shooterLeader.getConfigurator().apply(m_shooterConfig);
    m_shooterFollower.getConfigurator().apply(m_shooterConfig);
    m_hoodMotor.getConfigurator().apply(m_hoodConfig);
    m_turretMotor.getConfigurator().apply(m_turretConfig);
  }

  public static ShooterSubsystem getInstance() {
    if (s_shooterInstance == null) {
      s_shooterInstance = new ShooterSubsystem();
    }
    return s_shooterInstance;
  }

  public void setState(ShooterStates state) {
    m_selectedState = state;
  }

  private boolean inAZ() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return DriveSubsystem.getPose().getX() < Constants.FieldConstants.NZ_BLUE_X;
    } else {
      return DriveSubsystem.getPose().getX() > Constants.FieldConstants.NZ_RED_X;
    }
  }

  private boolean inNZ() {
    return DriveSubsystem.getPose().getX() > Constants.FieldConstants.NZ_BLUE_X
        && DriveSubsystem.getPose().getX() < Constants.FieldConstants.NZ_RED_X;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("ShooterSubsystem/InNZ", inNZ());
    Logger.recordOutput("ShooterSubsystem/InAZ", inAZ());

    // This method will be called once per scheduler run
  }
}

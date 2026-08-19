// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.generated.TunerConstants;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends StateMachine {

  public enum DriveStates implements SystemState {
    REST {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_driveState;
      }
    },
    DRIVER_CONTROL {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        AngularVelocity rotationRate =
            Constants.DriveConstants.MAX_ANGULAR_RATE.times(
                -getInstance().m_rotateRequest.getAsDouble());
        getInstance()
            .m_driveTrain
            .setControl(
                getInstance()
                    .m_drive
                    .withVelocityX(
                        Constants.DriveConstants.MAX_SPEED.times(
                            -getInstance().m_strafeRequest.getAsDouble()
                                * Math.abs(getInstance().m_strafeRequest.getAsDouble())))
                    .withVelocityY(
                        Constants.DriveConstants.MAX_SPEED.times(
                            -getInstance().m_driveRequest.getAsDouble()
                                * Math.abs(getInstance().m_driveRequest.getAsDouble())))
                    .withRotationalRate(rotationRate));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_driveState;
      }
    },
    CLIMB_ALIGN {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_driveState;
      }
    }
  }

  private static DriveSubsystem s_driveInstance;
  private static DriveStates m_driveState = DriveStates.REST;
  private DoubleSupplier m_driveRequest;
  private DoubleSupplier m_strafeRequest;
  private DoubleSupplier m_rotateRequest;

  private CommandSwerveDrivetrain m_driveTrain;
  private SwerveRequest.FieldCentric m_drive;
  private Translation2d m_hubPos;

  public DriveSubsystem() {
    super(DriveStates.REST);

    m_driveTrain = TunerConstants.createDrivetrain();
    setPerspective();

    m_drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(
                Constants.DriveConstants.MAX_SPEED.times(Constants.DriveConstants.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.DriveConstants.MAX_ANGULAR_RATE.times(0.1)) // Add a
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
  }

  public static DriveSubsystem getInstance() {
    if (s_driveInstance == null) {
      s_driveInstance = new DriveSubsystem();
    }
    return s_driveInstance;
  }

  public void setPerspective() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        m_driveTrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
        m_hubPos = Constants.HubConstants.RED_HUB_POS;
      }
      if (ally.get() == Alliance.Blue) {
        m_driveTrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
        m_hubPos = Constants.HubConstants.BLUE_HUB_POS;
      }
    }
  }

  public void configure_bindings(
      DoubleSupplier driveRequest, DoubleSupplier strafeRequest, DoubleSupplier rotateRequest) {
    m_driveRequest = driveRequest;
    m_strafeRequest = strafeRequest;
    m_rotateRequest = rotateRequest;
  }

  public void setDriveState(DriveStates driveState) {
    m_driveState = driveState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

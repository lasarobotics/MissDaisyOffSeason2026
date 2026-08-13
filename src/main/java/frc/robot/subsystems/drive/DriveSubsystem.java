// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.generated.TunerConstants;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends StateMachine {

  public enum DriveStates implements SystemState {
    DRIVER_CONTROL {
      @Override
      public void initialize() {
        getInstance().updateDrivetrainRotation();
      }

      @Override
      public void execute() {
        getDrivetrain()
            .setControl(
                getInstance()
                    .m_driverSwerveRequest
                    .withVelocityX(
                        -getInstance().m_joystickLeftY.getAsDouble()
                            * Constants.Drive.MAX_SPEED.in(MetersPerSecond)
                            * Constants.Drive.MAX_DRIVE_SPEED_SCALAR)
                    .withVelocityY(
                        -getInstance().m_joystickLeftX.getAsDouble()
                            * Constants.Drive.MAX_SPEED.in(MetersPerSecond)
                            * Constants.Drive.MAX_DRIVE_SPEED_SCALAR)
                    .withRotationalRate(
                        -getInstance().m_joystickRightX.getAsDouble()
                            * Constants.Drive.MAX_ANGULAR_VELOCITY.in(RotationsPerSecond)
                            * Constants.Drive.MAX_ROTATION_SPEED_SCALAR));
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    },
    CLIMB_ALIGN {
      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    }
  }

  public static void setState(DriveStates nextState) {
    s_requestedNextState = nextState;
  }

  private static DriveSubsystem s_driveInstance;
  private static CommandSwerveDrivetrain s_drivetrain;
  private static DriveStates s_requestedNextState;

  private DoubleSupplier m_joystickLeftX;
  private DoubleSupplier m_joystickLeftY;
  private DoubleSupplier m_joystickRightX;

  private SwerveRequest.FieldCentric m_driverSwerveRequest;

  public DriveSubsystem() {
    super(DriveStates.DRIVER_CONTROL);
    setState(DriveStates.DRIVER_CONTROL);

    s_drivetrain = TunerConstants.createDrivetrain();

    m_driverSwerveRequest =
        new SwerveRequest.FieldCentric().withDeadband(Constants.Drive.MAX_SPEED.div(10));
  }

  public static DriveSubsystem getInstance() {
    if (s_driveInstance == null) {
      s_driveInstance = new DriveSubsystem();
    }
    return s_driveInstance;
  }

  public static CommandSwerveDrivetrain getDrivetrain() {
    return s_drivetrain;
  }

  public void configureBindings(
      DoubleSupplier driveRequest, DoubleSupplier strafeRequest, DoubleSupplier rotateRequest) {
    m_joystickLeftY = driveRequest;
    m_joystickLeftX = strafeRequest;
    m_joystickRightX = rotateRequest;
  }

  public void updateDrivetrainRotation() {
    // default to blue alliance
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
      s_drivetrain.setOperatorPerspectiveForward(Rotation2d.kZero);
    } else {
      s_drivetrain.setOperatorPerspectiveForward(Rotation2d.k180deg);
    }
  }
}

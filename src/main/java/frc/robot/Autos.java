package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Autos {
  public static record WAYPOINT(double x, double y, double maxSpeed, double rotation) {}

  public static Autos s_autoInstance;

  public volatile boolean m_active;

  private Thread m_autoThread;

  private String auto;

  public String[] blueAutos = {
    "Blue Left Trench Regular",
    "Blue Right Trench Regular",
    "Blue Left Bump Regular",
    "Blue Right Bump Regular",
    "Blue Left Trench Delayed",
    "Blue Right Trench Delayed",
    "Blue Left Bump Delayed",
    "Blue Right Bump Delayed",
    "Blue Right Trench Depot Steal",
    "Blue Left Trench Depot Steal",
    "Blue Right Bump Depot Steal",
    "Blue Left Bump Depot Steal",
    "Blue Left Trench Depot",
    "Blue Left Bump Depot",
    "Blue Center Depot",
  };
  public String[] redAutos = {
    "Red Left Trench Regular",
    "Red Right Trench Regular",
    "Red Left Bump Regular",
    "Red Right Bump Regular",
    "Red Left Trench Delayed",
    "Red Right Trench Delayed",
    "Red Left Bump Delayed",
    "Red Right Bump Delayed",
    "Red Right Trench Depot Steal",
    "Red Left Trench Depot Steal",
    "Red Right Bump Depot Steal",
    "Red Left Bump Depot Steal",
    "Red Left Trench Depot",
    "Red Left Bump Depot",
    "Red Center Depot",
  };

  public Autos() {}

  public static Autos getInstance() {
    if (s_autoInstance == null) {
      s_autoInstance = new Autos();
    }
    return s_autoInstance;
  }

  public String[] getBlueAutos() {
    return blueAutos;
  }

  public String[] getRedAutos() {
    return blueAutos;
  }

  public void setAuto(String auto) {
    this.auto = auto;
  }

  public void assignAuto() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      switch (auto) {
        case "BLUE_LEFT_TRENCH_REGULAR":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueLeftTrenchRegular();
                  });
          break;

        case "BLUE_RIGHT_TRENCH_REGULAR":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueRightTrenchRegular();
                  });
          break;

        case "BLUE_LEFT_BUMP_REGULAR":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueLeftBumpRegular();
                  });
          break;

        case "BLUE_RIGHT_BUMP_REGULAR":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueRightBumpRegular();
                  });
          break;

        case "BLUE_LEFT_TRENCH_DELAYED":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueLeftTrenchDelayed();
                  });
          break;

        case "BLUE_RIGHT_TRENCH_DELAYED":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueRightTrenchDelayed();
                  });
          break;

        case "BLUE_LEFT_BUMP_DELAYED":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueLeftBumpDelayed();
                  });
          break;

        case "BLUE_RIGHT_BUMP_DELAYED":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueRightBumpDelayed();
                  });
          break;

        case "BLUE_RIGHT_TRENCH_DEPOT_STEAL":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueRightTrenchDepotSteal();
                  });
          break;

        case "BLUE_LEFT_TRENCH_DEPOT_STEAL":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueLeftTrenchDepotSteal();
                  });
          break;

        case "BLUE_RIGHT_BUMP_DEPOT_STEAL":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueRightBumpDepotSteal();
                  });
          break;

        case "BLUE_LEFT_BUMP_DEPOT_STEAL":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueLeftBumpDepotSteal();
                  });
          break;

        case "BLUE_LEFT_TRENCH_DEPOT":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueLeftTrenchDepot();
                  });
          break;

        case "BLUE_LEFT_BUMP_DEPOT":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueLeftBumpDepot();
                  });
          break;

        case "BLUE_CENTER_DEPOT":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    blueCenterDepot();
                  });
          break;

        default:
          break;
      }
    } else {
      switch (auto) {
        case "RED_LEFT_TRENCH_REGULAR":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redLeftTrenchRegular();
                  });
          break;

        case "RED_RIGHT_TRENCH_REGULAR":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redRightTrenchRegular();
                  });
          break;

        case "RED_LEFT_BUMP_REGULAR":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redLeftBumpRegular();
                  });
          break;

        case "RED_RIGHT_BUMP_REGULAR":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redRightBumpRegular();
                  });
          break;

        case "RED_LEFT_TRENCH_DELAYED":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redLeftTrenchDelayed();
                  });
          break;

        case "RED_RIGHT_TRENCH_DELAYED":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redRightTrenchDelayed();
                  });
          break;

        case "RED_LEFT_BUMP_DELAYED":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redLeftBumpDelayed();
                  });
          break;

        case "RED_RIGHT_BUMP_DELAYED":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redRightBumpDelayed();
                  });
          break;

        case "RED_RIGHT_TRENCH_DEPOT_STEAL":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redRightTrenchDepotSteal();
                  });
          break;

        case "RED_LEFT_TRENCH_DEPOT_STEAL":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redLeftTrenchDepotSteal();
                  });
          break;

        case "RED_RIGHT_BUMP_DEPOT_STEAL":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redRightBumpDepotSteal();
                  });
          break;

        case "RED_LEFT_BUMP_DEPOT_STEAL":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redLeftBumpDepotSteal();
                  });
          break;

        case "RED_LEFT_TRENCH_DEPOT":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redLeftTrenchDepot();
                  });
          break;

        case "RED_LEFT_BUMP_DEPOT":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redLeftBumpDepot();
                  });
          break;

        case "RED_CENTER_DEPOT":
          getInstance().m_autoThread =
              new Thread(
                  () -> {
                    redCenterDepot();
                  });
          break;

        default:
          break;
      }
    }
  }

  public void startAuto() {
    getInstance().m_active = true;
    getInstance().m_autoThread.start();
  }

  public void stopAuto() {
    getInstance().m_active = false;
  }

  public void goToWaypoint(
      Pose2d waypoint,
      double exitVelocity,
      double targetRot,
      double maxRotRate,
      boolean turnDirMatters,
      boolean... turnLeft) {

    DriveSubsystem.getInstance()
        .autoConfig(waypoint, exitVelocity, targetRot, maxRotRate, turnDirMatters, turnLeft);

    while (getInstance().m_active && !DriveSubsystem.getInstance().isAtWaypoint()) {
      try {
        Thread.sleep(1);
      } catch (InterruptedException e) {
        return;
      }
    }
  }

  public void blueLeftTrenchRegular() {
    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_TRENCH_NZ,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_LEFT_BOTTOM,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_RIGHT_BOTTOM,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_RIGHT_TOP,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_LEFT_TOP,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_TRENCH_NZ,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_TRENCH_AZ,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_CORNER_START,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_CORNER_END,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_TRENCH_AZ,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_TRENCH_NZ,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);

    goToWaypoint(
        Constants.AutoConstants.BLUE_LEFT_RIGHT_TOP,
        Constants.DriveConstants.MAX_SPEED
            .times(Constants.DriveConstants.FAST_SPEED_SCALAR)
            .in(MetersPerSecond),
        0,
        3,
        false);
  }

  public void blueRightTrenchRegular() {}

  public void blueLeftBumpRegular() {}

  public void blueRightBumpRegular() {}

  public void blueLeftTrenchDelayed() {}

  public void blueRightTrenchDelayed() {}

  public void blueLeftBumpDelayed() {}

  public void blueRightBumpDelayed() {}

  public void blueRightTrenchDepotSteal() {}

  public void blueLeftTrenchDepotSteal() {}

  public void blueRightBumpDepotSteal() {}

  public void blueLeftBumpDepotSteal() {}

  public void blueLeftTrenchDepot() {}

  public void blueLeftBumpDepot() {}

  public void blueCenterDepot() {}

  public void redLeftTrenchRegular() {}

  public void redRightTrenchRegular() {}

  public void redLeftBumpRegular() {}

  public void redRightBumpRegular() {}

  public void redLeftTrenchDelayed() {}

  public void redRightTrenchDelayed() {}

  public void redLeftBumpDelayed() {}

  public void redRightBumpDelayed() {}

  public void redRightTrenchDepotSteal() {}

  public void redLeftTrenchDepotSteal() {}

  public void redRightBumpDepotSteal() {}

  public void redLeftBumpDepotSteal() {}

  public void redLeftTrenchDepot() {}

  public void redLeftBumpDepot() {}

  public void redCenterDepot() {}
}

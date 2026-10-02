package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutoFollower {
  private static AutoFollower s_autoFollower;
  private static List<Robot.WaypointWithSpeed> selectedAuto;
  private static int currentStartPointIndex;
  private static int currentEndPointIndex;

  public static AutoFollower getInstance() {
    if (s_autoFollower == null) {
      s_autoFollower = new AutoFollower();
    }
    return s_autoFollower;
  }

  private static double getDesiredLookaheadDist() {
    double scalar =
        Math.hypot(
                DriveSubsystem.getInstance().getSpeeds().vxMetersPerSecond,
                DriveSubsystem.getInstance().getSpeeds().vyMetersPerSecond)
            / TunerConstants.kSpeedAt12Volts.magnitude();
    scalar =
        scalar
                * Constants.AutoConstants.LOOKAHEAD_SCALAR
                * Constants.AutoConstants.LOOKAHEAD_DIST_MAX
            + (1 - Constants.AutoConstants.LOOKAHEAD_SCALAR);
    return scalar;
  }

  public static void setAuto(List<Robot.WaypointWithSpeed> auto) {
    selectedAuto = auto;
    currentStartPointIndex = 0;
    currentEndPointIndex = 1;
    if (selectedAuto != null) {
      List<Pose2d> waypoints = new ArrayList<>();
      for (Robot.WaypointWithSpeed waypointWithSpeed : selectedAuto) {
        waypoints.add(waypointWithSpeed.waypoint());
      }
      Logger.recordOutput("Auto", waypoints.toArray(new Pose2d[0]));
    }
  }

  public static double[] getDesiredSpeeds() {
    /*
     * speedFlipper must be -1 if on red alliance
     */
    double speedFlipper = 1;
    if (!DriveSubsystem.getInstance().isBlueAlliance()) {
      speedFlipper = -1;
    }
    if (selectedAuto != null) {
      Pose2d currentStartPoint = selectedAuto.get(currentStartPointIndex).waypoint();
      Pose2d currentEndPoint = selectedAuto.get(currentEndPointIndex).waypoint();
      double startX = currentStartPoint.getX();
      double startY = currentStartPoint.getY();
      double endX = currentEndPoint.getX();
      double endY = currentEndPoint.getY();
      double robotX = DriveSubsystem.getInstance().getTranslation2d().getX();
      double robotY = DriveSubsystem.getInstance().getTranslation2d().getY();
      double radius = Inches.of(getDesiredLookaheadDist()).in(Meters);
      Translation2d target;
      double desiredAngle;
      double desiredSpeed;
      /*
       * We are essentially parameterizing x and y as functions of t, where 0 <= t <= 1
       * and the plugging x(t) and y(t) into the circle equation, and then simplifying,
       * to get a quadratic in the form stuffA * t^2 + stuffB * t + stuffC
       * and then we apply quadratic formula to a, b, and c (solving for t)
       * if discriminant is < 0, no solution exists
       * if disc. = 0, 1 solution
       * if disc > 0 2 solution
       * actual solutions for a, b, and c look like what is written below
       */
      double a = (endX - startX) * (endX - startX) + (endY - startY) * (endY - startY);
      double b =
          2 * (((endX - startX) * (startX - robotX)) + ((endY - startY) * (startY - robotY)));
      double c =
          (startX - robotX) * (startX - robotX)
              + (startY - robotY) * (startY - robotY)
              - (radius * radius);
      double discriminant = (b * b) - (4 * a * c);
      double t;
      if (discriminant < 0) {
        /*
         * find scalar projection of robot vector onto line vector, normalize, and thats our "t"
         */
        double dx = endX - startX;
        double dy = endY - startY;

        t = ((robotX - startX) * dx + (robotY - startY) * dy) / (dx * dx + dy * dy);
      } else if (discriminant == 0) {
        t = -b / (2 * a);
      } else {
        double sqrtDisc = Math.sqrt(discriminant);
        double t1 = (-b - sqrtDisc) / (2 * a);
        double t2 = (-b + sqrtDisc) / (2 * a);
        t = Math.max(t1, t2);
      }
      if (t > 1) {
        desiredAngle = DriveSubsystem.getInstance().getPose().getRotation().getRadians();
        if (currentEndPointIndex < selectedAuto.size() - 1) {
          currentStartPointIndex = currentEndPointIndex;
          currentEndPointIndex += 1;
          target = selectedAuto.get(currentStartPointIndex).waypoint().getTranslation();
        } else {
          return new double[] {0, 0, desiredAngle};
        }
        desiredSpeed = TunerConstants.kSpeedAt12Volts.magnitude();
      } else if (t < 0) {
        target = currentStartPoint.getTranslation();
        desiredAngle = DriveSubsystem.getInstance().getPose().getRotation().getRadians();
        desiredSpeed = TunerConstants.kSpeedAt12Volts.magnitude();
      } else {
        double xOfT = startX + t * (endX - startX);
        double yOfT = startY + t * (endY - startY);
        target = new Translation2d(xOfT, yOfT);
        desiredAngle = currentEndPoint.getRotation().getRadians();
        desiredSpeed =
            Math.min(
                selectedAuto.get(currentStartPointIndex).velocity(),
                TunerConstants.kSpeedAt12Volts.magnitude());
      }
      Translation2d delta = target.minus(new Translation2d(robotX, robotY));
      double deltaNorm = delta.getNorm();
      double xDesiredSpeed = desiredSpeed * (delta.getX() / deltaNorm);
      double YDesiredSpeed = desiredSpeed * (delta.getY() / deltaNorm);
      double turnSpeedScale = 1;
      double curvatureAngle;
      if (0 <= t && t <= 1) {
        if (currentEndPointIndex == selectedAuto.size() - 1) {
          curvatureAngle = Math.PI;
        } else {
          Translation2d pathVector1 = currentStartPoint.minus(currentEndPoint).getTranslation();
          Translation2d pathVector2 =
              selectedAuto
                  .get(currentEndPointIndex + 1)
                  .waypoint()
                  .minus(currentEndPoint)
                  .getTranslation();
          curvatureAngle =
              Math.acos(
                  pathVector1.dot(pathVector2) / (pathVector1.getNorm() * pathVector2.getNorm()));
        }
        double minScale = 0.25;
        curvatureAngle = Math.abs(curvatureAngle) / Math.PI;
        curvatureAngle = Math.min(curvatureAngle, 1.0);
        double targetScale = 1.0 - (1.0 - minScale) * curvatureAngle;
        turnSpeedScale = 1.0 - (1.0 - targetScale) * t * Constants.AutoConstants.TURN_AGGRESION;
        turnSpeedScale = Math.max(minScale, turnSpeedScale);
      }

      Logger.recordOutput("Auto/delta", delta);
      Logger.recordOutput("Auto/target", new Pose2d(target, new Rotation2d(0)));
      return new double[] {
        speedFlipper * turnSpeedScale * xDesiredSpeed, speedFlipper * YDesiredSpeed, desiredAngle
      };
    }
    return new double[] {0, 0, 0};
  }
}

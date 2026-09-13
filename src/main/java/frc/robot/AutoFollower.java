package frc.robot;

import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

public class AutoFollower {
  private static AutoFollower s_autoFollower;
  private static List<Pose2d> waypoints = new ArrayList<>();

  public AutoFollower() {}

  public static AutoFollower getInstance() {
    if (s_autoFollower == null) {
      s_autoFollower = new AutoFollower();
    }
    return s_autoFollower;
  }

  public static void setAuto(String auto) throws IOException, ParseException {
    waypoints = new ArrayList<>();
    List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(auto);

    for (PathPlannerPath path : paths) {
      for (Waypoint waypoint : path.getWaypoints()) {
        waypoints.add(new Pose2d(waypoint.anchor(), new Rotation2d(0)));
      }
    }
    Logger.recordOutput("AutoFollower/Waypoints", waypoints.toArray(new Pose2d[0]));
  }
}

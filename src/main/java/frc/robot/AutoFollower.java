package frc.robot;

import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public class AutoFollower {
  private static AutoFollower s_autoFollower;
  private static List<Pose2d> selectedAuto;

  public AutoFollower() {}

  public static AutoFollower getInstance() {
    if (s_autoFollower == null) {
      s_autoFollower = new AutoFollower();
    }
    return s_autoFollower;
  }

  public static void setAuto(List<Pose2d> auto) {}
}

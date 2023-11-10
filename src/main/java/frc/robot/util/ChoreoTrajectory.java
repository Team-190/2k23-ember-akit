package frc.robot.util;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.File;
import java.io.IOException;
import java.util.Collections;
import java.util.List;
import java.util.TreeMap;

public class ChoreoTrajectory {
  private static final ObjectMapper mapper = new ObjectMapper();
  private final TreeMap<Double, ChoreoTrajectoryState> states = new TreeMap<>();

  public ChoreoTrajectory(List<ChoreoTrajectoryState> states) {
    states.forEach(
        (state) -> {
          this.states.put(state.timestamp(), state);
        });
  }

  /** Loads a trajectory from an exported JSON file. */
  public static ChoreoTrajectory fromFile(File file) {
    try {
      return new ChoreoTrajectory(mapper.readValue(file, new TypeReference<>() {}));
    } catch (IOException e) {
      e.printStackTrace();
      return new ChoreoTrajectory(Collections.emptyList());
    }
  }

  /** Returns the trajectory state at the specified time, interpolating as necessary. */
  public ChoreoTrajectoryState sample(double timestamp) {
    var lastEntry = states.floorEntry(timestamp);
    var nextEntry = states.higherEntry(timestamp);
    if (lastEntry == null && nextEntry == null) {
      return new ChoreoTrajectoryState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    } else if (lastEntry == null) {
      return nextEntry.getValue();
    } else if (nextEntry == null) {
      return lastEntry.getValue();
    } else {
      double t = (timestamp - lastEntry.getKey()) / (nextEntry.getKey() - lastEntry.getKey());
      return lastEntry.getValue().interpolate(nextEntry.getValue(), t);
    }
  }

  /** Returns the total time of the trajectory in seconds. */
  public double getTotalTime() {
    var lastEntry = states.lastEntry();
    if (lastEntry == null) {
      return 0.0;
    } else {
      return lastEntry.getValue().timestamp();
    }
  }

  /** Returns the initial pose of the trajectory. */
  public Pose2d getStartPose() {
    var firstEntry = states.firstEntry();
    if (firstEntry == null) {
      return new Pose2d();
    } else {
      return firstEntry.getValue().getPose();
    }
  }

  /** Returns the end pose of the trajectory. */
  public Pose2d getEndPose() {
    var lastEntry = states.lastEntry();
    if (lastEntry == null) {
      return new Pose2d();
    } else {
      return lastEntry.getValue().getPose();
    }
  }

  /** Returns an array of all of the poses in the trajectory. */
  public Pose2d[] getPoses() {
    return states.values().stream().map(state -> state.getPose()).toArray(Pose2d[]::new);
  }
}

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record ChoreoTrajectoryState(
    double timestamp,
    double x,
    double y,
    double heading,
    double velocityX,
    double velocityY,
    double angularVelocity)
    implements Interpolatable<ChoreoTrajectoryState> {

  public Pose2d getPose() {
    return new Pose2d(x, y, Rotation2d.fromRadians(heading));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(velocityX, velocityY, angularVelocity);
  }

  @Override
  public ChoreoTrajectoryState interpolate(ChoreoTrajectoryState endValue, double t) {
    var pose = getPose().interpolate(endValue.getPose(), t);
    return new ChoreoTrajectoryState(
        MathUtil.interpolate(this.timestamp, endValue.timestamp, t),
        pose.getX(),
        pose.getY(),
        pose.getRotation().getRadians(),
        MathUtil.interpolate(this.velocityX, endValue.velocityX, t),
        MathUtil.interpolate(this.velocityY, endValue.velocityY, t),
        MathUtil.interpolate(this.angularVelocity, endValue.angularVelocity, t));
  }
}

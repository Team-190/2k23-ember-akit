package frc.robot.util.choreo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChoreoHolonomicDriveController {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  /**
   * Constructs a holonomic drive controller. The theta controller will be automatically configured
   * for continuous input.
   *
   * @param xController A PID Controller to respond to error in the field-relative x direction.
   * @param yController A PID Controller to respond to error in the field-relative y direction.
   * @param thetaController A PID controller to respond to error in angle.
   */
  public ChoreoHolonomicDriveController(
      PIDController xController, PIDController yController, PIDController thetaController) {
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the robot-relative chassis speeds to apply to the drive.
   *
   * @param currentPose The current field-relative pose
   * @param reference The trajectory state to follow, see {@link ChoreoTrajectory#sample(double)}
   * @return The robot-relative chassis speeds to apply to the drive
   */
  public ChassisSpeeds calculate(Pose2d currentPose, ChoreoTrajectoryState reference) {
    // Calculate feedforward velocities (field-relative)
    var chassisSpeeds = reference.getChassisSpeeds();
    double xFF = chassisSpeeds.vxMetersPerSecond;
    double yFF = chassisSpeeds.vyMetersPerSecond;
    double thetaFF = chassisSpeeds.omegaRadiansPerSecond;

    // Calculate feedback velocities (based on position error).
    var poseReference = reference.getPose();
    double xFeedback = xController.calculate(currentPose.getX(), poseReference.getX());
    double yFeedback = yController.calculate(currentPose.getY(), poseReference.getY());
    double thetaFeedback =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), poseReference.getRotation().getRadians());

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback, currentPose.getRotation());
  }
}

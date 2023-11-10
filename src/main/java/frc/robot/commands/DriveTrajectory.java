package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.choreo.ChoreoHolonomicDriveController;
import frc.robot.util.choreo.ChoreoTrajectory;
import java.io.File;
import org.littletonrobotics.junction.Logger;

public class DriveTrajectory extends Command {
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber("DriveTrajectory/DriveKp");
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber("DriveTrajectory/DriveKd");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveTrajectory/DriveTolerance");
  private static final LoggedTunableNumber turnKp =
      new LoggedTunableNumber("DriveTrajectory/TurnKp");
  private static final LoggedTunableNumber turnKd =
      new LoggedTunableNumber("DriveTrajectory/TurnKd");
  private static final LoggedTunableNumber turnToleranceDegrees =
      new LoggedTunableNumber("DriveTrajectory/TurnToleranceDegrees");

  private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

  private final ChoreoTrajectory trajectory;
  private final ChoreoHolonomicDriveController controller =
      new ChoreoHolonomicDriveController(xController, yController, thetaController);

  private final Drive drive;
  private final Timer timer = new Timer();

  static {
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        driveKp.initDefault(6.0);
        driveKd.initDefault(0.0);
        turnKp.initDefault(8.0);
        turnKd.initDefault(0.0);
        break;
      case SIM:
        driveKp.initDefault(3.0);
        driveKd.initDefault(0.0);
        turnKp.initDefault(16.0);
        turnKd.initDefault(0.0);
        break;
      default:
        break;
    }
  }

  /**
   * Creates a new DriveTrajectory.
   *
   * @param drive The drive subsystem
   * @param name The name of the trajectory to load, not including the folder name or JSON extension
   */
  public DriveTrajectory(Drive drive, String name) {
    this.drive = drive;
    addRequirements(drive);
    var file =
        new File(Filesystem.getDeployDirectory(), "choreo" + File.separator + name + ".json");
    trajectory = ChoreoTrajectory.fromFile(file);
  }

  @Override
  public void initialize() {
    // Log trajectory
    Logger.recordOutput("Odometry/Trajectory", trajectory.getPoses());

    // Reset all controllers
    timer.restart();
    xController.reset();
    yController.reset();
    thetaController.reset();

    // Reset PID gains
    xController.setP(driveKp.get());
    xController.setD(driveKd.get());
    yController.setP(driveKp.get());
    yController.setD(driveKd.get());
    thetaController.setP(turnKp.get());
    thetaController.setD(turnKd.get());
  }

  @Override
  public void execute() {
    // Update from tunable numbers
    if (driveKd.hasChanged(hashCode())
        || driveKp.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || turnKd.hasChanged(hashCode())
        || turnKp.hasChanged(hashCode())
        || turnToleranceDegrees.hasChanged(hashCode())) {
      xController.setP(driveKp.get());
      xController.setD(driveKd.get());
      xController.setTolerance(driveTolerance.get());
      yController.setP(driveKp.get());
      yController.setD(driveKd.get());
      yController.setTolerance(driveTolerance.get());
      thetaController.setP(turnKp.get());
      thetaController.setD(turnKd.get());
      thetaController.setTolerance(Units.degreesToRadians(turnToleranceDegrees.get()));
    }

    // Get setpoint
    var setpoint = trajectory.sample(timer.get());
    Logger.recordOutput("Odometry/TrajectorySetpoint", new Pose2d[] {setpoint.getPose()});

    // Calculate velocity
    ChassisSpeeds speeds = controller.calculate(drive.getPose(), setpoint);
    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.recordOutput("Odometry/Trajectory", new Pose2d[] {});
    Logger.recordOutput("Odometry/TrajectorySetpoint", new Pose2d[] {});
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime());
  }

  /** Returns the total time of the trajectory in seconds. */
  public double getDuration() {
    return trajectory.getTotalTime();
  }

  /** Returns the initial pose of the trajectory. */
  public Pose2d getStartPose() {
    return trajectory.getStartPose();
  }

  /** Returns the end pose of the trajectory. */
  public Pose2d getEndPose() {
    return trajectory.getEndPose();
  }
}

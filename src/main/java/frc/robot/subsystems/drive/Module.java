package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0);
  private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
  private double lastPositionMeters = 0.0; // Used for delta calculation

  // drive PD
  private static LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/DriveKp");
  private static LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/DriveKd");

  // turn PD
  private static LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/Turnkp");
  private static LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/TurnKd");

  // drive feedforward
  private static LoggedTunableNumber driveKs = new LoggedTunableNumber("Drive/DriveKs");
  private static LoggedTunableNumber driveKv = new LoggedTunableNumber("Drive/DriveKv");

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        driveKs.initDefault(0.154);
        driveKv.initDefault(0.112);
        driveKp.initDefault(0.05);
        driveKd.initDefault(0.0);
        turnKp.initDefault(7.0);
        turnKd.initDefault(0.0);
        break;
      case SIM:
        driveKs.initDefault(0.0);
        driveKv.initDefault(0.13);
        driveKp.initDefault(0.1);
        driveKd.initDefault(0.0);
        turnKp.initDefault(10.0);
        turnKd.initDefault(0.0);
        break;
      default:
        driveKs.initDefault(0.0);
        driveKv.initDefault(0.0);
        driveKp.initDefault(0.0);
        driveKd.initDefault(0.0);
        turnKp.initDefault(0.0);
        turnKd.initDefault(0.0);
        break;
    }

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Update from tunable numbers
    if (driveKp.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
      driveFeedback.setPID(driveKp.get(), 0.0, driveKd.get());
    }
    if (turnKp.hasChanged(hashCode()) || turnKd.hasChanged(hashCode())) {
      turnFeedback.setPID(turnKp.get(), 0.0, turnKd.get());
    }
    if (driveKs.hasChanged(hashCode()) || driveKv.hasChanged(hashCode())) {
      driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
    }

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
      turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
    }

    // Run closed loop turn control
    if (angleSetpoint != null) {
      io.setTurnVoltage(
          turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

        // Run drive controller
        double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
        io.setDriveVoltage(
            driveFeedforward.calculate(velocityRadPerSec)
                + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
      }
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (turnRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return inputs.turnPosition.plus(turnRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * WHEEL_RADIUS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module position delta since the last call to this method. */
  public SwerveModulePosition getPositionDelta() {
    var delta = new SwerveModulePosition(getPositionMeters() - lastPositionMeters, getAngle());
    lastPositionMeters = getPositionMeters();
    return delta;
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}

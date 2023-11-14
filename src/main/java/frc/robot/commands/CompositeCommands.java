package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller;

public class CompositeCommands {
  public static final Command deployAndIntakeCommand(
      Pivot pivot, Roller roller, CommandXboxController controller) {
    return pivot
        .deployCommand()
        .alongWith(
            roller.intakeCommand(),
            roller.intakeRumbleCommand(
                (double percent) ->
                    controller.getHID().setRumble(RumbleType.kBothRumble, percent)));
  }

  public static final Command deployAndIntakeCommand(Pivot pivot, Roller roller) {
    return pivot.deployCommand().alongWith(roller.intakeCommand()).withTimeout(3);
  }

  public static final Command dropCommand(Pivot pivot, Roller rollers) {
    return pivot
        .dropCommand()
        .alongWith(
            Commands.sequence(
                Commands.none(),
                Commands.waitUntil(pivot::atGoal),
                Commands.waitSeconds(0.06),
                rollers.dumpCommand()));
  }

  public static final Command autoDropCommand(Pivot pivot, Roller rollers) {
    return pivot
        .autoDropCommand()
        .alongWith(
            Commands.sequence(
                Commands.none(),
                Commands.waitUntil(pivot::atGoal),
                Commands.waitSeconds(0.06),
                rollers.dumpCommand()));
  }

  public static final Command highLaunchCommand(Pivot pivot, Roller rollers) {
    return pivot
        .launchCommand()
        .alongWith(
            Commands.sequence(
                Commands.none(), Commands.waitUntil(pivot::atGoal), Commands.waitSeconds(0.06)));
  }
}

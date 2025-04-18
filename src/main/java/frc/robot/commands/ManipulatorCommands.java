package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Manipulator;

public class ManipulatorCommands
{
    private ManipulatorCommands()
    {
    }

    public static Command intake()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().setVolts(-3)).andThen(Commands.waitSeconds(0.35)).finallyDo(() -> Manipulator.getInstance().stop());
    }

    public static Command output()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().setVolts(3)).andThen(Commands.waitSeconds(0.35)).finallyDo(() -> Manipulator.getInstance().stop());
    }

    public static Command stop()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().setVolts(0));
    }
}

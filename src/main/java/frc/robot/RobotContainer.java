package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class RobotContainer
{
    // Controller
    private final CommandJoystick       _driverJoystick  = new CommandJoystick(0);
    private final CommandJoystick       _driverButtons   = new CommandJoystick(1);
    private final CommandJoystick       _operatorButtons = new CommandJoystick(2);
    private final CommandXboxController _controller      = new CommandXboxController(3);

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        DriverStation.silenceJoystickConnectionWarning(true);
        CameraServer.startAutomaticCapture();
        // Dashboard.getInstance();
        // Configure the button bindings
        configureButtonBindings();
        // configureTestBindings();
    }

    @SuppressWarnings("unused")
    private void configureTestBindings()
    {
        Drive.getInstance().setDefaultCommand(DriveCommands.joystickDrive(() -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> -_controller.getRightX(), () -> false));
        _controller.a().onTrue(DriveCommands.setOdometer(new Pose2d(Units.inchesToMeters(297.5), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(0))));

        _controller.a().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Stow));
        _controller.leftBumper().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Intake));
        _controller.rightBumper().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Intake));
        _controller.b().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level1));
        _controller.x().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level2));
        _controller.y().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level3));
        _controller.start().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level4));

    }

    @SuppressWarnings("unused")
    private void configureButtonBindings()
    {
        // Trigger _hasCoral = new Trigger(() -> _manipulator.hasCoral());
        // Trigger _manipulatorRunning = new Trigger(() -> _manipulator.isRunning());
        // Trigger _operatorButton12 = _operatorButtons.axisGreaterThan(0, 0.5);

        Trigger _operatorButton14 = _operatorButtons.axisLessThan(1, -0.5);
        Trigger _operatorButton15 = _operatorButtons.axisGreaterThan(1, 0.5);

        // Default command, normal field-relative drive
        // Drive.getInstance().setDefaultCommand(DriveCommands.joystickDrive(() ->
        // -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // -_driverJoystick.getZ(), () -> robotCentric()));
        Drive.getInstance().setDefaultCommand(CompositeCommands.joystickDrive(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> -_driverButtons.getX(), () -> robotCentric(), 2, 5));

        // Driver Controls
        _driverJoystick.button(2).whileTrue(DriveCommands.reduceSpeed());
        // _driverJoystick.button(3).onTrue(null); // replace this with switching
        // cameras/
        _driverButtons.button(2).onTrue(DriveCommands.resetGyro());
        _driverButtons.trigger().onTrue(CompositeCommands.output());
        _driverJoystick.trigger().onTrue(CompositeCommands.output());

        /*
         * _driverButtons.button(10) .whileTrue(DriveCommands.driveAtOrientation(() ->
         * -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> robotCentric(),
         * () -> Constants.Field.BLUE_REEF_ANGLE_ONE,
         * Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)); _driverButtons.button(2)
         * .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), ()
         * -> -_driverJoystick.getX(), () -> robotCentric(), () ->
         * Constants.Field.BLUE_REEF_ANGLE_TWO,
         * Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)); _driverButtons.button(3)
         * .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), ()
         * -> -_driverJoystick.getX(), () -> robotCentric(), () ->
         * Constants.Field.BLUE_REEF_ANGLE_THREE,
         * Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)); _driverButtons.button(4)
         * .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), ()
         * -> -_driverJoystick.getX(), () -> robotCentric(), () ->
         * Constants.Field.BLUE_REEF_ANGLE_FOUR,
         * Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)); _driverButtons.button(5)
         * .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), ()
         * -> -_driverJoystick.getX(), () -> robotCentric(), () ->
         * Constants.Field.BLUE_REEF_ANGLE_FIVE,
         * Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)); _driverButtons.button(6)
         * .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), ()
         * -> -_driverJoystick.getX(), () -> robotCentric(), () ->
         * Constants.Field.BLUE_REEF_ANGLE_SIX,
         * Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)); _driverButtons.button(7)
         * .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), ()
         * -> -_driverJoystick.getX(), () -> robotCentric(), () ->
         * Constants.Field.BLUE_RIGHT_STATION_ANGLE,
         * Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)); _driverButtons.button(8)
         * .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), ()
         * -> -_driverJoystick.getX(), () -> robotCentric(), () ->
         * Constants.Field.BLUE_LEFT_STATION_ANGLE,
         * Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)); _driverButtons.button(9)
         * .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), ()
         * -> -_driverJoystick.getX(), () -> robotCentric(), () ->
         * Constants.Field.BLUE_PROCESSOR_ANGLE,
         * Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));
         */
        _controller.b().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level1));
        _controller.x().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level2));
        _controller.y().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level3));
        _controller.a().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level4));
        _controller.leftBumper().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Intake));
        _controller.rightBumper().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Intake));
        _controller.back().onTrue(ElevatorCommands.stow());
        _controller.start().onTrue(ElevatorCommands.stow());
        _controller.povUp().onTrue(ElevatorCommands.modifyHeight(Constants.Elevator.ELEVATOR_MODIFICATION_HEIGHT));
        _controller.povDown().onTrue(ElevatorCommands.modifyHeight(-Constants.Elevator.ELEVATOR_MODIFICATION_HEIGHT));
        _controller.rightTrigger().onTrue(CompositeCommands.output());

    }

    public Command getAutonomousCommand()
    {
        return Dashboard.getInstance().getSelectedAuto();
    }

    private boolean robotCentric()
    {
        return false;
        // return _driverJoystick.button(1).getAsBoolean();
    }
}

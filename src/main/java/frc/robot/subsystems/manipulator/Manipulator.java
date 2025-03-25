package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase
{
    private static Manipulator _instance;

    public static Manipulator getInstance()
    {
        if (_instance == null)
        {
            var io = switch (Constants.AdvantageKit.CURRENT_MODE)
            {
                case REAL -> new ManipulatorIOSparkMax();
                default -> new ManipulatorIO() {};
            };

            _instance = new Manipulator(io);
        }

        return _instance;
    }

    private final ManipulatorIO                 _io;
    private final ManipulatorIOInputsAutoLogged _inputs = new ManipulatorIOInputsAutoLogged();

    private Manipulator(ManipulatorIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);

    }

    public void output()
    {
        _io.setVolts(-3);
    }

    public void slowIntake()
    {
        _io.setVolts(3);
    }

    public void intake()
    {
        _io.setVolts(3);
    }

    public void stop()
    {
        _io.setVolts(0);
    }

    public void setVolts(double volts)
    {
        _io.setVolts(volts);
    }

    public boolean isRunning()
    {
        return _inputs.AppliedVolts > 0;
    }

    public double getLeftOutputSpeed()
    {
        return _inputs.AppliedVolts / Constants.General.MOTOR_VOLTAGE;
    }
}

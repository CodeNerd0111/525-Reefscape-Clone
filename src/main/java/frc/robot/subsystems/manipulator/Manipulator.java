package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.ManipulatorIO.ManipulatorIOInputs;

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
                case SIM -> new ManipulatorIOSim();
                default -> new ManipulatorIO() {};
            };

            _instance = new Manipulator(io);
        }

        return _instance;
    }

    private final ManipulatorIO                 _io;
    private final ManipulatorIOInputsAutoLogged _inputs = new ManipulatorIOInputsAutoLogged();
    private Debouncer                           _debouncer;
    private boolean                             _coralDetected;

    private Manipulator(ManipulatorIO io)
    {
        _io        = io;
        _debouncer = new Debouncer(Constants.Manipulator.DEBOUNCE_LOOP_COUNT);
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        _coralDetected = true;
        // _coralDetected = (/* !_inputs.startSensorTripped && _inputs.endSensorTripped)
        // || */ (Elevator.getInstance().getExtension() > ((Constants.Elevator.L3_HEIGHT
        // + Constants.Elevator.L4_HEIGHT) / 2.0) /* && _inputs.endSensorTripped */));
        Logger.processInputs("Manipulator", _inputs);
        Logger.recordOutput("Detected Coral", _coralDetected);
        Logger.recordOutput("Has Coral", hasCoral());
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

    public boolean detectedCoral()
    {
        return _coralDetected;
    }

    public boolean hasCoral()
    {
        return _debouncer.calculate(_coralDetected);
    }

    public boolean isRunning()
    {
        return _inputs.AppliedVolts > 0;
    }

    public double getLeftOutputSpeed()
    {
        return _inputs.AppliedVolts / Constants.General.MOTOR_VOLTAGE;
    }

    /*
     * public boolean isStartSensorTripped() { return _inputs.startSensorTripped; }
     * public boolean isEndSensorTripped() { return _inputs.endSensorTripped; }
     */
}

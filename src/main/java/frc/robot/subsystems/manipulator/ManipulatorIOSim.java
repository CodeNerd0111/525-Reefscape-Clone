package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ManipulatorIOSim implements ManipulatorIO
{
    private final DCMotorSim   _MotorSim;
    private final DigitalInput _lightSensorStart;
    private final DigitalInput _lightSensorEnd;
    private double             _AppliedVolts;

    public ManipulatorIOSim()
    {
        _MotorSim         = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Manipulator.MANIPULATOR_MOTOR, 0.02, Constants.Manipulator.MOTOR_REDUCTION), Constants.Drive.DRIVE_GEARBOX);
        _lightSensorEnd   = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_END);
        _lightSensorStart = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_START);
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs)
    {
        _MotorSim.update(Constants.General.LOOP_PERIOD_SECS);
        inputs.CurrentAmps        = Math.abs(_MotorSim.getCurrentDrawAmps());
        inputs.startSensorTripped = _lightSensorStart.get();
        inputs.endSensorTripped   = _lightSensorEnd.get();
    }

    @Override
    public void setVolts(double volts)
    {
        _AppliedVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);

        _MotorSim.setInputVoltage(volts);
    }
}

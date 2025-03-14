package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

public class ManipulatorIOSparkMax implements ManipulatorIO
{
    private final SparkMax     _Motor;
    private final DigitalInput _startSensor;
    private final DigitalInput _endSensor;

    public ManipulatorIOSparkMax()
    {
        _Motor       = new SparkMax(Constants.CAN.MANIPULATOR, MotorType.kBrushless);
        _startSensor = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_START);
        _endSensor   = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_END);

        var leftConfig  = new SparkMaxConfig();
        var rightConfig = new SparkMaxConfig();

        leftConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(Constants.General.MOTOR_VOLTAGE);
        rightConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        leftConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
        rightConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        _Motor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _Motor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs)
    {
        inputs.AppliedVolts = _Motor.getAppliedOutput() * _Motor.getBusVoltage();
        inputs.CurrentAmps  = _Motor.getOutputCurrent();

        inputs.startSensorTripped = !_startSensor.get();
        inputs.endSensorTripped   = !_endSensor.get();
    }

    @Override
    public void setVolts(double volts)
    {
        _Motor.setVoltage(volts);
    }
}

package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class ManipulatorIOSparkMax implements ManipulatorIO
{
    private final SparkMax _Motor;

    public ManipulatorIOSparkMax()
    {
        _Motor = new SparkMax(Constants.CAN.MANIPULATOR, MotorType.kBrushed);

        var leftConfig = new SparkMaxConfig();

        leftConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        leftConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        _Motor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs)
    {
        inputs.AppliedVolts = _Motor.getAppliedOutput() * _Motor.getBusVoltage();
        inputs.CurrentAmps  = _Motor.getOutputCurrent();

        // inputs.startSensorTripped = !_startSensor.get();
        // inputs.endSensorTripped = !_endSensor.get();
    }

    @Override
    public void setVolts(double volts)
    {
        _Motor.setVoltage(volts);
    }
}

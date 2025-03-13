package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class ModuleIOHardware implements ModuleIO
{
    private SparkMax            _driveMotor;
    private SparkMax            _turnMotor;
    private CANcoder            _absoulteTurnEncoder;
    private RelativeEncoder     _turnREncoder;
    private RelativeEncoder     _driveREncoder;
    private Rotation2d          _potOffset;
    private StatusSignal<Angle> _turnAbsoultePosition;

    public ModuleIOHardware(int module)
    {
        _driveMotor = new SparkMax(switch (module)
        {
            case 0 -> Constants.CAN.FL_DRIVE;
            case 1 -> Constants.CAN.FR_DRIVE;
            case 2 -> Constants.CAN.BL_DRIVE;
            case 3 -> Constants.CAN.BR_DRIVE;
            default -> 0;
        }, MotorType.kBrushless);

        _turnMotor           = new SparkMax(switch (module)
        {
            case 0 -> Constants.CAN.FL_TURN;
            case 1 -> Constants.CAN.FR_TURN;
            case 2 -> Constants.CAN.BL_TURN;
            case 3 -> Constants.CAN.BR_TURN;
            default -> 0;
        }, MotorType.kBrushless);
        _absoulteTurnEncoder = new CANcoder(switch (module)
        {
            case 0 -> Constants.AIO.FL_ENCODER;
            case 1 -> Constants.AIO.FR_ENCODER;
            case 2 -> Constants.AIO.BL_ENCODER;
            case 3 -> Constants.AIO.BR_ENCODER;
            default -> 0;
        });

        _potOffset = switch (module)
        {
            case 0 -> Constants.Drive.FL_ZERO_ROTATION;
            case 1 -> Constants.Drive.FR_ZERO_ROTATION;
            case 2 -> Constants.Drive.BL_ZERO_ROTATION;
            case 3 -> Constants.Drive.BR_ZERO_ROTATION;
            default -> new Rotation2d();
        };

        var driveConfig = new SparkMaxConfig();
        var turnConfig  = new SparkMaxConfig();

        turnConfig.inverted(Constants.Drive.TURN_INVERTED).idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.Drive.TURN_MOTOR_CURRENT_LIMIT).voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        turnConfig.encoder.uvwMeasurementPeriod(10).quadratureMeasurementPeriod(10).uvwAverageDepth(2).quadratureAverageDepth(2);
        turnConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderPositionPeriodMs((int)(1000.0 / Constants.Drive.ODOMETRY_FREQUENCY)).primaryEncoderVelocityAlwaysOn(true).primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        driveConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.Drive.TURN_MOTOR_CURRENT_LIMIT).voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        driveConfig.encoder.uvwMeasurementPeriod(10).quadratureMeasurementPeriod(10).uvwAverageDepth(2).quadratureAverageDepth(2);
        driveConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderPositionPeriodMs((int)(1000.0 / Constants.Drive.ODOMETRY_FREQUENCY)).primaryEncoderVelocityAlwaysOn(true).primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        _turnREncoder  = _turnMotor.getEncoder();
        _driveREncoder = _driveMotor.getEncoder();
        _turnREncoder.setPosition(0.0);
        _turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _turnAbsoultePosition = _absoulteTurnEncoder.getAbsolutePosition();
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, _turnAbsoultePosition);
        _absoulteTurnEncoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        BaseStatusSignal.refreshAll(_turnAbsoultePosition);
        inputs.driveAppliedVolts      = _driveMotor.getAppliedOutput() * _driveMotor.getBusVoltage();
        inputs.driveCurrentAmps       = _driveMotor.getOutputCurrent();
        inputs.drivePositionRad       = Units.rotationsToRadians(_driveREncoder.getPosition() / Constants.Drive.DRIVE_MOTOR_REDUCTION);
        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(_driveREncoder.getVelocity()) / Constants.Drive.TURN_MOTOR_REDUCTION;

        inputs.turnAbsolutePosition  = Rotation2d.fromRotations((_turnAbsoultePosition.getValueAsDouble())).minus(_potOffset);
        inputs.turnPosition          = Rotation2d.fromRotations(_turnREncoder.getPosition() / Constants.Drive.TURN_MOTOR_REDUCTION);
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(_turnREncoder.getVelocity()) / Constants.Drive.TURN_MOTOR_REDUCTION;
        inputs.turnAppliedVolts      = _turnMotor.getAppliedOutput() * _turnMotor.getBusVoltage();
        inputs.turnCurrentAmps       = _turnMotor.getOutputCurrent();
    }

    @Override
    public void setDriveVolts(double volts)
    {
        _driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVolts(double volts)
    {
        _turnMotor.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable)
    {
        var config = new SparkMaxConfig();
        config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        _driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setTurnBrakeMode(boolean enable)
    {
        var config = new SparkMaxConfig();
        config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        _turnMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setAngleOffset(Rotation2d offset)
    {
        _potOffset = offset;
    }
}

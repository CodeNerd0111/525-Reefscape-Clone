package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.util.Utilities;

public class ElevatorIOHardware implements ElevatorIO
{
    private final SparkMax        _leaderSparkMax;
    private final SparkMax        _followerSparkMax;
    private final RelativeEncoder _extensionEncoder;

    public ElevatorIOHardware()
    {
        _leaderSparkMax   = new SparkMax(Constants.CAN.LEAD_ELEVATOR, MotorType.kBrushless);
        _followerSparkMax = new SparkMax(Constants.CAN.FOLLOWER_ELEVATOR, MotorType.kBrushless);
        _extensionEncoder = _leaderSparkMax.getEncoder();

        var leaderConfig = new SparkMaxConfig();

        leaderConfig.idleMode(IdleMode.kBrake).voltageCompensation(Constants.General.MOTOR_VOLTAGE).inverted(false).smartCurrentLimit(120);
        leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).outputRange(0, Constants.Elevator.MAX_EXTENSION);

        var followerConfig = new SparkMaxConfig();

        followerConfig.idleMode(IdleMode.kBrake).voltageCompensation(Constants.General.MOTOR_VOLTAGE).follow(Constants.CAN.LEAD_ELEVATOR, false).smartCurrentLimit(120);
        followerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).outputRange(0, Constants.Elevator.MAX_EXTENSION);

        Utilities.tryUntilOk(_leaderSparkMax, 5, () -> _leaderSparkMax.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        Utilities.tryUntilOk(_followerSparkMax, 5, () -> _followerSparkMax.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        _extensionEncoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs)
    {
        inputs.extensionPosition = _extensionEncoder.getPosition();
        Utilities.ifOk(_leaderSparkMax, _extensionEncoder::getVelocity, (value) -> inputs.extensionVelocity = value * Constants.Elevator.EXTENSION_SCALE / Constants.Elevator.EXTENSION_MOTOR_REDUCTION);

        Utilities.ifOk(_leaderSparkMax, new DoubleSupplier[] { _leaderSparkMax::getAppliedOutput, _leaderSparkMax::getBusVoltage }, (values) -> inputs.leaderVolts = values[0] * values[1]);
        Utilities.ifOk(_leaderSparkMax, _leaderSparkMax::getOutputCurrent, (value) -> inputs.leaderCurrent = value);

        Utilities.ifOk(_followerSparkMax, new DoubleSupplier[] { _followerSparkMax::getAppliedOutput, _followerSparkMax::getBusVoltage }, (values) -> inputs.followerVolts = values[0] * values[1]);
        Utilities.ifOk(_followerSparkMax, _followerSparkMax::getOutputCurrent, (value) -> inputs.followerCurrent = value);
    }

    @Override
    public void setVolts(double volts)
    {
        _leaderSparkMax.setVoltage(volts);
    }
}

package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO
{
    @AutoLog // adds logging to ManipulatorIO interface
    public static class ManipulatorIOInputs
    {
        public double AppliedVolts = 0.0;
        public double CurrentAmps  = 0.0;
        // public boolean startSensorTripped = false;
        // public boolean endSensorTripped = false;
    }

    public default void updateInputs(ManipulatorIOInputs inputs)
    {
    }

    public default void setVolts(double volts)
    {
        setMotorVolts(volts);
    }

    public default void setMotorVolts(double volts)
    {
    }
}

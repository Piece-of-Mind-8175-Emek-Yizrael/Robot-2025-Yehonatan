package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.AutoLog;

public interface TransferIO {

    @AutoLog
    public static class TransferIOInputs {
        public double motorVoltage = 0.0;
        public boolean coralSensor = false;
    }

    public default void updateInputs(TransferIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default boolean getCoralSensor() {
        return false;
    }

    public default void stopMotor() {
    }

}

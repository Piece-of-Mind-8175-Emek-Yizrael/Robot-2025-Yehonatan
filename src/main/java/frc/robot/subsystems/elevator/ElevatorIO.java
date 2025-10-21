package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        boolean motorConnected = false;
        double elevatorVelocity = 0.0;
        double elevatorPosition = 0.0;
        double elevatorAppliedVolts = 0.0;
        boolean foldSwitch = false;
        boolean brakeSwitch = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setSpeed(double speed) {
    }

    public default void setVoltage(double volts) {
    }

    public default boolean getBrakeSwitch() {
        return false;
    }

    public default boolean getFoldSwitch() {
        return false;
    }

    public default boolean atGoal() {
        return false;
    }

    public default void resetEncoder() {
    }

    public default void stopMotor() {
    }

    public default void setGoal(double position) {
    }

    public default void resetPID() {
    }

    public default void resetPID(double goal) {
    }
    
    public default void resistGravity() {
    }

    public default void resetIfPressed() {
    }

}

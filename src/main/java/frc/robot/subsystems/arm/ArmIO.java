package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public double armPosition = 0.0;
        public double armVelocity = 0.0;
        public double armVoltage = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stopMotor() {
    }

    public default void resistGravity() {
    }

    public default void resetIfPressed() {
    }

    public default double getPosition() {
        return 0.0;
    }

    public default boolean getLowerSwitch() {
        return false;
    }

    public default boolean getUpperSwitch() {
        return false;
    }

    public default void resetPID() {
    }

    public default void resetPID(double goal) {
    }

    public default boolean atGoal() {
        return false;
    }

    public default void goToPosition(double radians) {
    }

}

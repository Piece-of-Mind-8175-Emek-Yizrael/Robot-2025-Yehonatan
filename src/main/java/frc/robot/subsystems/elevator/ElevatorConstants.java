package frc.robot.subsystems.elevator;

public class ElevatorConstants {

    public static final int MOTOR_ID = 0;
    public static final int BRAKE_SWITCH_ID = 1;
    public static final int FOLD_SWITCH_ID = 2;

    // PID Constants
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    // Feedforward Constants
    public static final double kS = 0.2; // Static gain
    public static final double kG = 0.5; // Gravity gain
    public static final double kV = 1.0; // Velocity gain

}

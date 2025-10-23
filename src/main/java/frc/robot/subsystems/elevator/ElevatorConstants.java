package frc.robot.subsystems.elevator;

public class ElevatorConstants {

    public static final int MOTOR_ID = 16;
    public static final int BRAKE_SWITCH_ID = 4;
    public static final int FOLD_SWITCH_ID = 3;
    public static final int CURRENT_LIMIT = 40;
    public static final int VOLTAGE_COMPENSATION = 12;
    public static final int POSITION_CONVERSION_FACTOR = 1;

    // PID Constants
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double TOLERANCE = 0.15;
    public static final double MAX_VELOCITY = 1;
    public static final double MAX_ACCELERATION = 1;
    // Feedforward Constants
    public static final double kS = 0.0; // Static gain
    public static final double kG = 0.5; // Gravity gain
    public static final double kV = 0.0; // Velocity gain

}

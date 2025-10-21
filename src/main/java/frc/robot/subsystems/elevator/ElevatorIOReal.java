package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.BRAKE_SWITCH_ID;
import static frc.robot.subsystems.elevator.ElevatorConstants.FOLD_SWITCH_ID;
import static frc.robot.subsystems.elevator.ElevatorConstants.MAX_ACCELERATION;
import static frc.robot.subsystems.elevator.ElevatorConstants.MAX_VELOCITY;
import static frc.robot.subsystems.elevator.ElevatorConstants.MOTOR_ID;
import static frc.robot.subsystems.elevator.ElevatorConstants.TOLERANCE;
import static frc.robot.subsystems.elevator.ElevatorConstants.kD;
import static frc.robot.subsystems.elevator.ElevatorConstants.kG;
import static frc.robot.subsystems.elevator.ElevatorConstants.kI;
import static frc.robot.subsystems.elevator.ElevatorConstants.kP;
import static frc.robot.subsystems.elevator.ElevatorConstants.kS;
import static frc.robot.subsystems.elevator.ElevatorConstants.kV;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class ElevatorIOReal implements ElevatorIO {

    POMSparkMax motor;
    POMDigitalInput brakeSwitch;
    POMDigitalInput foldSwitch;
    RelativeEncoder encoder;
    ProfiledPIDController pidController;
    ElevatorFeedforward feedforward;

    public ElevatorIOReal() {
        motor = new POMSparkMax(MOTOR_ID);
        encoder = motor.getEncoder();
        brakeSwitch = new POMDigitalInput(BRAKE_SWITCH_ID);
        foldSwitch = new POMDigitalInput(FOLD_SWITCH_ID);
        pidController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        feedforward = new ElevatorFeedforward(kS, kG, kV);
        pidController.setTolerance(TOLERANCE);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.elevatorVelocity = encoder.getVelocity();
        inputs.elevatorPosition = encoder.getPosition();
        inputs.elevatorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.brakeSwitch = brakeSwitch.get();
        inputs.foldSwitch = foldSwitch.get();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public boolean getBrakeSwitch() {
        return brakeSwitch.get();
    }

    @Override
    public boolean getFoldSwitch() {
        return foldSwitch.get();
    }

    @Override
    public boolean atGoal() {
        return pidController.atSetpoint();
    }

    @Override
    public void resetEncoder() {
        encoder.setPosition(0.0);
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public void setGoal(double position) {
        pidController.setGoal(position);
        double pos = encoder.getPosition();
        motor.set(pidController.calculate(pos) + feedforward.calculate(pidController.getSetpoint().velocity));
    }

    @Override
    public void resetPID() {
        pidController.reset(encoder.getPosition(), encoder.getVelocity());
    }

    @Override
    public void resetPID(double goal) {
        if (goal - encoder.getPosition() > 0) {
            pidController.reset(encoder.getPosition(), Math.max(encoder.getVelocity(), feedforward.calculate(1)));
        } else {
            pidController.reset(encoder.getPosition(), Math.min(encoder.getVelocity(), feedforward.calculate(1)));
        }
    }

    @Override
    public void resistGravity() {
        setVoltage(feedforward.calculate(0));
    }

    @Override
    public void resetIfPressed() {
        if (getFoldSwitch()) {
            resetEncoder();
        }

        if (getBrakeSwitch()) {
            motor.setBrake(false);
        } else {
            motor.setBrake(true);
        }
    }



}

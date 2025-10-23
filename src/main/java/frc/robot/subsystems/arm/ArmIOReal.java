package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;
import static frc.robot.subsystems.arm.ArmConstants.*;

public class ArmIOReal implements ArmIO {
    POMSparkMax motor;
    POMDigitalInput lowerSwitch;
    POMDigitalInput upperSwitch;
    RelativeEncoder encoder;
    ProfiledPIDController pidController;
    ArmFeedforward feedforward;

    public ArmIOReal() {
        this.motor = new POMSparkMax(MOTOR_ID);
        this.encoder = motor.getEncoder();
        this.lowerSwitch = new POMDigitalInput(LOWER_SWITCH_ID);
        this.upperSwitch = new POMDigitalInput(UPPER_SWITCH_ID);
        this.pidController = new ProfiledPIDController(kP, kI, kD,
                new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCELERATION));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armPosition = encoder.getPosition();
        inputs.armVelocity = encoder.getVelocity();
        inputs.armVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void resistGravity() {
        setVoltage(feedforward.calculate(getPosition(), 0));
    }

    @Override
    public void resetPID() {
        pidController.reset(encoder.getPosition(), encoder.getVelocity());
    }

    @Override
    public void resetPID(double goal) {
        if (goal - encoder.getPosition() > 0) {
            pidController.reset(encoder.getPosition(), Math.max(encoder.getVelocity(), feedforward.calculate(1, 0)));
        } else {
            pidController.reset(encoder.getPosition(), Math.min(encoder.getVelocity(), feedforward.calculate(1, 0)));
        }
    }

    @Override
    public boolean getLowerSwitch() {
        return lowerSwitch.get();
    }

    @Override
    public boolean getUpperSwitch() {
        return upperSwitch.get();
    }

    @Override
    public void resetIfPressed() {
        if (getLowerSwitch()) {
            encoder.setPosition(0.0);
        }

        // TODO: set to coast to if break switch pressed
    }

    @Override
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public void goToPosition(double radians) {
        double pidOutput = pidController.calculate(getPosition(), radians);
        double ffOutput = feedforward.calculate(radians, 0);
        setVoltage(pidOutput + ffOutput);
    }

}

package frc.robot.subsystems.transfer;

import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class TransferIOReal implements TransferIO {
    POMSparkMax motor;
    POMDigitalInput coralSensor;

    public TransferIOReal() {
        motor = new POMSparkMax(TransferConstants.MOTOR_ID);
        coralSensor = new POMDigitalInput(TransferConstants.SENSOR_ID);
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        inputs.motorVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.coralSensor = coralSensor.get();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public boolean getCoralSensor() {
        return coralSensor.get();
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

}

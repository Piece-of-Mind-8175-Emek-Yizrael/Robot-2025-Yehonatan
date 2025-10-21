package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

public class Elevator extends SubsystemBase{

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.inputs = new ElevatorIOInputsAutoLogged();

        setDefaultCommand(new RepeatCommand(new ConditionalCommand(this.runOnce(()-> io.setVoltage(0)), this.runOnce(()-> io.resistGravity()),()->io.getFoldSwitch())));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public ElevatorIO getIO() {
        return io;
    }

}

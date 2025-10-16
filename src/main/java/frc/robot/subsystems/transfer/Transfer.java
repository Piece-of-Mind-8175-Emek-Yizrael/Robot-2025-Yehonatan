package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.transfer.TransferIO.TransferIOInputs;

public class Transfer extends SubsystemBase {
    private final TransferIO io;
    private final TransferIOInputsAutoLogged inputs;

    public Transfer(TransferIO io) {
        this.io = io;
        this.inputs = new TransferIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Transfer", inputs);
    }

    public TransferIO getIO() {
        return io;
    }

}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;
import frc.robot.subsystems.transfer.TransferIO;
import static frc.robot.subsystems.transfer.TransferConstants.*;

public class TransferCommands {

    public static Command intakeCoral(Transfer transfer) {

        TransferIO io = transfer.getIO();

        return Commands
                .runEnd(() -> io.setVoltage(CORAL_INTAKE_VOLTAGE), () -> io.stopMotor(), transfer)
                .until(() -> !(io.getCoralSensor()));
    }

    public static Command outtakeCoral(Transfer transfer, boolean isFlipped) {

        TransferIO io = transfer.getIO();

        return Commands.runEnd(() -> io.setVoltage(isFlipped ? -CORAL_OUTTAKE_VOLTAGE
                : CORAL_OUTTAKE_VOLTAGE), () -> io.stopMotor(), transfer)
                .until(() -> (io.getCoralSensor()));

    }

}

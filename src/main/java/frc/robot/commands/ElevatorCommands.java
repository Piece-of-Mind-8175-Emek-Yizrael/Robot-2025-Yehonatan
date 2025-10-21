package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;

public class ElevatorCommands {
    public static Command setVoltage(Elevator elevator, double voltage) {
        return Commands.runOnce(() -> elevator.getIO().setVoltage(voltage), elevator);
    }

    public static Command goToPosition(Elevator elevator, double pos) {

        ElevatorIO io = elevator.getIO();

        return new FunctionalCommand(
            () -> {
                io.stopMotor();
                io.resetPID(pos);
            },
            () -> io.setGoal(pos)
            ,
            interrupted -> io.stopMotor(),
            () -> io.atGoal(),
            elevator
        );
    }
}

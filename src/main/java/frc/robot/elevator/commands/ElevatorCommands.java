package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.elevator.ElevatorConfig.ElevatorPosition;

public class ElevatorCommands {
    public static Command elevatorConeLow() {
        return new InstantCommand(() -> RobotContainer.elevator.setElevatorSetpoint(ElevatorPosition.kMid), RobotContainer.elevator);
    }

    public static Command elevatorConeHigh() {
        return new InstantCommand(() -> RobotContainer.elevator.setElevatorSetpoint(ElevatorPosition.kHigh), RobotContainer.elevator);
    }

    public static Command elevatorCubeLow() {
        return new InstantCommand(() -> RobotContainer.elevator.setElevatorSetpoint(ElevatorPosition.kMid), RobotContainer.elevator);
    }

    public static Command elevatorCubeHigh() {
        return new InstantCommand(() -> RobotContainer.elevator.setElevatorSetpoint(ElevatorPosition.kHigh), RobotContainer.elevator);
    }
}

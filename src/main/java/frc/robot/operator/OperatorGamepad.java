package frc.robot.operator;

import frc.SpectrumLib.gamepads.Gamepad;
import frc.robot.RobotContainer;
import frc.robot.claw.commands.ClawCommands;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.elevator.commands.RunElevatorManual;

public class OperatorGamepad extends Gamepad {
    public OperatorGamepad() {
        super("OperatorController", OperatorConfig.kOperatorPort);
    }

    @Override
    public void setupTeleopButtons() {
        gamepad.xButton.onTrue(ElevatorCommands.elevatorConeLow());
        gamepad.aButton.onTrue(ElevatorCommands.elevatorConeHigh());
        gamepad.bButton.onTrue(ElevatorCommands.elevatorCubeLow());
        gamepad.yButton.onTrue(ElevatorCommands.elevatorCubeHigh());

        gamepad.leftBumper.onTrue(ClawCommands.toggleClawState());

        gamepad.leftTriggerButton.whileTrue(new RunElevatorManual(RobotContainer.elevator, () -> gamepad.leftStick.getY()));
    }

    @Override
    public void setupDisabledButtons() {
    }

    @Override
    public void setupTestButtons() {
    }
}

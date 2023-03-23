package frc.robot.elevator.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConfig;
import frc.robot.elevator.ElevatorConfig.ElevatorPosition;

public class SetElevatorState extends CommandBase{
    private final Elevator elevator;

    private final ElevatorConfig.ElevatorPosition targetElevatorState;

    private TrapezoidProfile profile;
    private final ElevatorFeedforward elevatorFFController;
    private final PIDController elevatorPIDController;

    private Long startTs; 

    public SetElevatorState(Elevator elevator, ElevatorConfig.ElevatorPosition targetElevatorState) {
        this.elevator = elevator;
        this.targetElevatorState = targetElevatorState;

        elevatorFFController = new ElevatorFeedforward(ElevatorConfig.elevkS,ElevatorConfig.elevkG, ElevatorConfig.elevkV, ElevatorConfig.elevkA);
        elevatorPIDController = new PIDController(ElevatorConfig.kP, ElevatorConfig.kI, ElevatorConfig.kD);
        
        startTs = null;

        addRequirements(this.elevator);
    }

    @Override
    public String getName() {
        String elevStateName = targetElevatorState.name();
        String output = "SetElevatorState[" + elevStateName + "]";
        return output;
    }

    @Override
    public void initialize() {
        System.out.println(getName() + ": init");

        profile = generateElevatorProfile();
        
        startTs = null;
    }

    @Override
    public void execute() {
        final var duration = getElapsedTime();
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }

        final var targetState = profile.calculate(duration);

        final var ffOutput = elevatorFFController.calculate(targetState.velocity);
        final var pidOutput = elevatorPIDController.calculate(elevator.getElevatorVelocity(), targetState.velocity);

        elevator.setElevatorOutput((ffOutput + pidOutput) / 12.0);
    }


    @Override
    public void end(boolean interrupted) {
        System.out.println(getName() + ": end");
        boolean[] elevatorLimitSwitchStates = elevator.getElevatorLimitSwitchState();
    
        if(elevatorLimitSwitchStates[0]) {
            elevator.setElevatorState(ElevatorPosition.kZero);
        }
        else if(elevatorLimitSwitchStates[1]) {
            elevator.setElevatorState(ElevatorPosition.kHigh);
        }
        else {
            elevator.setElevatorState(interrupted ? ElevatorPosition.kUnspecified : targetElevatorState);
        }

        elevator.stopElevator();
    }

    @Override
    public boolean isFinished() {
        boolean[] elevatorLimitSwitchStates = elevator.getElevatorLimitSwitchState();
        return ((elevatorLimitSwitchStates[1]) && elevator.getElevatorVelocity() < 0) || (profile != null && profile.isFinished(getElapsedTime()));
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }


    private TrapezoidProfile generateElevatorProfile() {
        // final var startPos = elevator.getElevatorPosition();
        // final var endPos = targetElevatorState.targetPos;
        
        // final var distance = Math.abs(endPos - startPos);

        // System.out.println("Start: " + startPos + ", End: " + endPos + ", Dist: " + distance);

        // final TrapezoidProfile.State startState, goalState;
        
        // if(startPos < endPos) {
        //     goalState = new TrapezoidProfile.State(distance, 0.0);
        //     startState = new TrapezoidProfile.State(0.0, elevator.getElevatorVelocity());
        // }
        // else {
        //     goalState = new TrapezoidProfile.State(0,0);
        //     startState = new TrapezoidProfile.State(distance, elevator.getElevatorVelocity());
        // }

        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConfig.kMaxVelocity,
                ElevatorConfig.kMinVelocity
            ), 
            new TrapezoidProfile.State(targetElevatorState.targetPos, 0),  //goal state
            new TrapezoidProfile.State(elevator.getElevatorPosition(), elevator.getElevatorVelocity())); //start state
    }
}

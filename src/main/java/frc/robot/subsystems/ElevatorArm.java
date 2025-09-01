package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorArm {

    private static ElevatorArmPosition m_elevatorArmPos = ElevatorArmPosition.HOMED;
    private static ElevatorArmPosition m_elevatorArmDesiredPos = ElevatorArmPosition.HOMED;

    public enum ElevatorArmPosition {
        CORAL_L1, 
        CORAL_L2,
        CORAL_L3,
        CORAL_L4,
        HOMED,
        ALGAE_HIGH,
        ALGAE_LOW,
        BARGE,
        PROCESSOR,
        READY
    }

    public static ElevatorArmPosition getElevatorArmEnumPosition() {
        return m_elevatorArmPos;
    }

    public static void setElevatorArmEnumPosition(ElevatorArmPosition newElevatorArmPos) {
        m_elevatorArmPos = newElevatorArmPos;
    }

    public static void setElevatorArmDesiredPosition(ElevatorArmPosition newElevatorArmPos) {
        m_elevatorArmDesiredPos = newElevatorArmPos;
    }    

    public static void setPosition(ElevatorArmPosition position) {
        RobotContainer.m_elevatorSubsystem.setPosition(position);
    }

    public static void ready() {
        RobotContainer.m_armSubsystem.readyArm();
    }

    public static Command goToPosition(ElevatorArmPosition position) {
        return 
            RobotContainer.m_elevatorSubsystem.goToPosition(position).alongWith(
                Commands.none().until(() -> RobotContainer.m_elevatorSubsystem.atSafeHeight())).finallyDo(
                    () -> RobotContainer.m_armSubsystem.rotateArm(position));
    }

    public static Command goToPosition() {
        return 
            RobotContainer.m_elevatorSubsystem.goToPosition(m_elevatorArmDesiredPos).alongWith(
                Commands.none().until(() -> RobotContainer.m_elevatorSubsystem.atSafeHeight())).finallyDo(
                    () -> RobotContainer.m_armSubsystem.rotateArm(m_elevatorArmDesiredPos));
    }
}



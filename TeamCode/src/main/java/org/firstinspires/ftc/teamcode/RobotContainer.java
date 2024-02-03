package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.commands.PinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    private final XboxController mXboxController = new XboxController(0);
    private final XboxController mXboxController2 = new XboxController(1);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();
    private final PinceSubsystem mPinceSubsystem = new PinceSubsystem();
    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);
    private final BrasCommand mBrasCommand = new BrasCommand(mBrasSubsystem, mXboxController2);
    private final PinceCommand mPinceCommand = new PinceCommand(mPinceSubsystem);

    private final AutonomousCommandGroup mAutonomousCommandGroup = new AutonomousCommandGroup(mDriveSubsystem);;

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {
        //JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        //buttonX.whileTrue(mBrasCommandPos1);
    }

    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
        mBrasSubsystem.setDefaultCommand(mBrasCommand);
    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
   }
}

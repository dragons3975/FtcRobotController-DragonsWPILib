package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();

    private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);

    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final BrasCommandPos1 mBrasCommandPos1 = new BrasCommandPos1(mBrasSubsystem, 0);


   private final BrasCommand mBrasCommand = new BrasCommand(mBrasSubsystem, mXboxController);

    private final IntakeCommand mIntakeCommand = new IntakeCommand(mIntakeSubsystem);

    private final AutonomousCommandGroup mAutonomousCommandGroup;

    public RobotContainer() {

        mAutonomousCommandGroup = new AutonomousCommandGroup(mDriveSubsystem);
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {
        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.whileTrue(mBrasCommand);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.whileTrue(mIntakeCommand);

        JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        //buttonA.onTrue();

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.whileTrue(mIntakeCommand);



    }
    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
        mBrasSubsystem.setDefaultCommand(mBrasCommand);
    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
   }
}

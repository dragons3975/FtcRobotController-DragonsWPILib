package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.BrasIncrCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    //private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();

    private final  PinceSubsystem mPinceSubsystem = new PinceSubsystem();

    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);

    private final BrasIncrCommand mIncr1 = new BrasIncrCommand(mBrasSubsystem, 40, 0,0);
    private final BrasIncrCommand mIncr2 = new BrasIncrCommand(mBrasSubsystem, -40, 0,0);
    private final BrasIncrCommand mIncr3 = new BrasIncrCommand(mBrasSubsystem, 0, 0.05,0);
    private final BrasIncrCommand mIncr4 = new BrasIncrCommand(mBrasSubsystem, 0, -0.05,0);
    private final BrasIncrCommand mIncr5 = new BrasIncrCommand(mBrasSubsystem, 0, 0,0.05);
    private final BrasIncrCommand mIncr6 = new BrasIncrCommand(mBrasSubsystem, 0, 0,-0.05);




    private final AutonomousCommandGroup mAutonomousCommandGroup = new AutonomousCommandGroup(mBrasSubsystem);

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.onTrue(mIncr1);

        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.onTrue(mIncr2);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.onTrue(mIncr3);

        JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        buttonA.onTrue(mIncr4);

        JoystickButton DpadUp = new JoystickButton(mXboxController, XboxController.Button.kUp.value);
        DpadUp.onTrue(mIncr5);

        JoystickButton DpadDown = new JoystickButton(mXboxController, XboxController.Button.kDown.value);
        DpadDown.onTrue(mIncr6);
    }
    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
   }
}

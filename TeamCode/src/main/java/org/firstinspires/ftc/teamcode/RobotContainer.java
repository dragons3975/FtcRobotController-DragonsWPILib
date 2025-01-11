package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.BleuDroiteExtra;
import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.PinceCloseCommand;
import org.firstinspires.ftc.teamcode.commands.PinceOpenCommand;
import org.firstinspires.ftc.teamcode.commands.PivotDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.TestButtonCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(0);
    private final XboxController mXboxController2 = new XboxController(1);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();
    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final PinceSubsystem mPinceSubsystem = new PinceSubsystem();

    private final TestButtonCommand mTestButtonCommandA = new TestButtonCommand("A");
    private final TestButtonCommand mTestButtonCommandLB = new TestButtonCommand("LB");

    private final DriveDefaultCommand mDriveDefaultCommand = new DriveDefaultCommand(mDriveSubsystem, mXboxController);
    private final BrasDefaultCommand mBrasDefaultCommand = new BrasDefaultCommand(mBrasSubsystem, mXboxController2);

    private final PivotDefaultCommand mPivotDefaultCommand = new PivotDefaultCommand(mPinceSubsystem, mXboxController2);
    private final PinceOpenCommand mPinceOpenCommand = new PinceOpenCommand(mPinceSubsystem, mXboxController2);
    private final PinceCloseCommand mPinceCloseCommand = new PinceCloseCommand(mPinceSubsystem, mXboxController2);



    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    public void init() {
        //mCalibreBrasCommand.schedule();
    }

    private void configureButtonBindings() {
        JoystickButton buttonA = new JoystickButton(mXboxController2, XboxController.Button.kA.value);
        buttonA.onTrue(mPinceOpenCommand);

        JoystickButton buttonB = new JoystickButton(mXboxController2, XboxController.Button.kB.value);
        buttonB.onTrue(mPinceCloseCommand);
    }

    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveDefaultCommand);
        mBrasSubsystem.setDefaultCommand(mBrasDefaultCommand);
        mPinceSubsystem.setDefaultCommand(mPivotDefaultCommand);
    }

    public Command getAutonomousCommand() {
        return new BleuDroiteExtra(mDriveSubsystem);
    }
}

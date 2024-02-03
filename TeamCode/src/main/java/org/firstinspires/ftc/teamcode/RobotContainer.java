package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DescendBrasCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FermePinceCommand;
import org.firstinspires.ftc.teamcode.commands.OuvrePinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.MonteBrasCommand;
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


    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);
    private final OuvrePinceCommand mOuvrePinceCommand = new OuvrePinceCommand(mPinceSubsystem);
    private final FermePinceCommand mFermePinceCommand = new FermePinceCommand(mPinceSubsystem);


    private final MonteBrasCommand mMonteBrasCommand = new MonteBrasCommand(mBrasSubsystem);

    private final DescendBrasCommand mDescendBrasCommand = new DescendBrasCommand(mBrasSubsystem);


    private final AutonomousCommandGroup mAutonomousCommandGroup = new AutonomousCommandGroup(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {
        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.onTrue(mOuvrePinceCommand);
        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.onTrue(mFermePinceCommand);

        JoystickButton buttonflechehaut = new JoystickButton(mXboxController, XboxController.Button.kUp.value);
        buttonflechehaut.whileTrue(mMonteBrasCommand);

        JoystickButton buttonflechebas = new JoystickButton(mXboxController, XboxController.Button.kDown.value);
        buttonflechebas.whileTrue(mDescendBrasCommand);
    }

    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
   }
}

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.MainAutonomousCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.LanceurCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.RamasseurCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.RejetteCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RamasseurSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();
    private final RamasseurSubsystem mRamasseurSubsystem = new RamasseurSubsystem();
    private final LanceurSubsystem mLanceurSubsystem = new LanceurSubsystem();
    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);
    private final RamasseurCommand mRamasseurCommand = new RamasseurCommand(mRamasseurSubsystem);
    private final RejetteCommand mRejetteCommand = new RejetteCommand(mRamasseurSubsystem);
    private final LanceurCommand mLanceurCommand = new LanceurCommand(mLanceurSubsystem);
    private final MainAutonomousCommand mAuto = new MainAutonomousCommand(mDriveSubsystem, mRamasseurSubsystem, mLanceurSubsystem);

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    public void init() {
    }

    private void configureButtonBindings() {
        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.whileTrue(mRamasseurCommand);
        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.whileTrue(mRejetteCommand);
        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.whileTrue(mLanceurCommand);
        JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        buttonA.toggleOnTrue(mAuto);



    }

    private void configureDefaultCommands() {
       mDriveSubsystem.setDefaultCommand(mDriveCommand);
    }

    public Command getAutonomousCommand() {
       return mAuto;

    }

}

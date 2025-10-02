package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commands.BrasCommand.BobCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.OmeletCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.SushiCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.TestCommand;
import org.firstinspires.ftc.teamcode.subsystems.TestRecruesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final TestSubsystem mTestSubsystem = new TestSubsystem();
    private final TestRecruesSubsystem mTestRecruesSubsystem = new TestRecruesSubsystem();
    private final TestCommand mTestCommand = new TestCommand(mTestSubsystem);
    private final SushiCommand mSushiCommand = new SushiCommand(mTestRecruesSubsystem);
    private final OmeletCommand mOmeletCommand = new OmeletCommand(mTestRecruesSubsystem, mTestSubsystem);
    private final BobCommand mBobCommand = new BobCommand(mTestRecruesSubsystem, mTestSubsystem);
    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    public void init() {
    }

    private void configureButtonBindings() {
        JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        buttonA.whileTrue(mBobCommand);
        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.whileTrue(mSushiCommand);
        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.whileTrue(mOmeletCommand);
    }

    private void configureDefaultCommands() {
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void stop() {
    }
}

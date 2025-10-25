package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commands.BrasCommand.BobCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.LanceurCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.LanceurManetteCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.LanceurPeriodicCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.OmeletCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.RamasseurCommand;
import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RamasseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestRecruesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    //private final TestSubsystem mTestSubsystem = new TestSubsystem();
    private final LanceurSubsystem mLanceurSubsystem = new LanceurSubsystem();
    //private inal RamasseurSubsystem mRamasseurSubsystem = new RamasseurSubsystem();
    //private final TestRecruesSubsystem mTestRecruesSubsystem = new TestRecruesSubsystem();
    //private final RamasseurCommand mRamasseurCommand = new RamasseurCommand(mRamasseurSubsystem);
    private final LanceurCommand mLanceurCommand = new LanceurCommand(mLanceurSubsystem);
    private final LanceurPeriodicCommand mLanceurPeriodicCommand = new LanceurPeriodicCommand(mLanceurSubsystem);

    private final LanceurManetteCommand mLanceurManetteCommand = new LanceurManetteCommand(mLanceurSubsystem, mXboxController);
    //private final OmeletCommand mOmeletCommand = new OmeletCommand(mTestSubsystem);
    //private final BobCommand mBobCommand = new BobCommand(mTestSubsystem);

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    public void init() {
    }

    private void configureButtonBindings() {
        JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        buttonA.toggleOnTrue(mLanceurPeriodicCommand.withTimeout(10));
        /*JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.whileTrue(mRamasseurCommand);*/
        /*JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.whileTrue(mOmeletCommand);*/
        /*JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.whileTrue(mLanceurCommand);*/
    }

    private void configureDefaultCommands() {
        mLanceurSubsystem.setDefaultCommand(mLanceurManetteCommand);
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void stop() {
    }
}

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.BleuDroiteExtra;
import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.TestButtonCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final VisionSubsystem mVisionSubsystem = new VisionSubsystem();
    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem(mVisionSubsystem);

    private final TestButtonCommand mTestButtonCommandA = new TestButtonCommand("A");
    private final TestButtonCommand mTestButtonCommandLB = new TestButtonCommand("LB");

    private final DriveDefaultCommand mDriveDefaultCommand = new DriveDefaultCommand(mDriveSubsystem, mXboxController);
    private final BrasDefaultCommand mBrasDefaultCommand = new BrasDefaultCommand(mBrasSubsystem, mXboxController2);

    private final TrajectoryCommand mTrajectoryCommand = new TrajectoryCommand(mDriveSubsystem);



    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    public void init() {
        //mCalibreBrasCommand.schedule();
    }

    private void configureButtonBindings() {
        JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        buttonA.whileTrue(mTestButtonCommandA);

        JoystickButton buttonLB = new JoystickButton(mXboxController, XboxController.Button.kLeftBumper.value);
        buttonLB.whileTrue(mTestButtonCommandLB);

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.onTrue(mTrajectoryCommand);
    }

    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveDefaultCommand);
        mBrasSubsystem.setDefaultCommand(mBrasDefaultCommand);
    }

    public Command getAutonomousCommand() {
        return new BleuDroiteExtra(mDriveSubsystem);
    }

    public void stop() {
        mVisionSubsystem.close();
    }
}

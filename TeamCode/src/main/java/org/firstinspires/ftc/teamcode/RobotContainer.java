package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.BleuDroiteExtra;
import org.firstinspires.ftc.teamcode.commands.BrasPanier1Command;
import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.GrimperCommand;
import org.firstinspires.ftc.teamcode.commands.PinceRotationDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.TestButtonCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);


    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();
    private final ExtensionSubsystem mExtensionSubsystem = new ExtensionSubsystem();
    private final GrimpeurSubsystem mGrimpeurSubsystem = new GrimpeurSubsystem();
    private final PinceBrasSubsystem mPinceBrasSubsystem = new PinceBrasSubsystem();
    private final PinceExtensionSubsystem mPinceExtensionSubsystem = new PinceExtensionSubsystem();

    private final VisionSubsystem mVisionSubsystem = new VisionSubsystem();

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem(mVisionSubsystem);

    private final TestButtonCommand mTestButtonCommandA = new TestButtonCommand("A");
    private final TestButtonCommand mTestButtonCommandLB = new TestButtonCommand("LB");
    private final GrimperCommand mGrimperCommand = new GrimperCommand(mGrimpeurSubsystem);
    private final BrasPanier1Command mBrasPanier1Command = new BrasPanier1Command(mBrasSubsystem, mXboxController2);

    private final DriveDefaultCommand mDriveDefaultCommand = new DriveDefaultCommand(mDriveSubsystem, mXboxController);
    private final BrasDefaultCommand mBrasDefaultCommand = new BrasDefaultCommand(mBrasSubsystem, mXboxController2);
    private final ExtendCommand mExtendCommand = new ExtendCommand(mExtensionSubsystem);
    private final PinceRotationDefaultCommand mPinceRotationDefaultCommand = new PinceRotationDefaultCommand(mPinceExtensionSubsystem, mXboxController);

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

        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.onTrue(mExtendCommand);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.whileTrue(mGrimperCommand);

        ///JoystickButton buttonA2 = new JoystickButton(mXboxController2, XboxController.Button.kA.value);
        ///buttonA2.whileTrue(mBrasPanier1Command);


    }

    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveDefaultCommand);
        mBrasSubsystem.setDefaultCommand(mBrasPanier1Command);
    }

    public Command getAutonomousCommand() {
        return new BleuDroiteExtra(mDriveSubsystem);
    }

    public void stop() {
        mVisionSubsystem.close();
    }
}

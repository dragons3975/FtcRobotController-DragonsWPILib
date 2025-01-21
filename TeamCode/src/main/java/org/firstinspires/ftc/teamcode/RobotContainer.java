package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.BleuDroiteExtra;
import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.GrimpeAutoCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionMaxExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionMinExtCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.CordePositionCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.PlieurPositionCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.ClosePinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.PlieurDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.OpenPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PinceRotationDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMaxPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMinPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurCordeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);
    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();
    private final ExtensionSubsystem mExtensionSubsystem = new ExtensionSubsystem();
    private final GrimpeurSubsystem mGrimpeurSubsystem = new GrimpeurSubsystem();
    private final GrimpeurCordeSubsystem mGrimpeurCordeSubsystem = new GrimpeurCordeSubsystem();
    private final PinceBrasSubsystem mPinceBrasSubsystem = new PinceBrasSubsystem();
    private final PinceExtensionSubsystem mPinceExtensionSubsystem = new PinceExtensionSubsystem();
    private final VisionSubsystem mVisionSubsystem = new VisionSubsystem();
    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem(mVisionSubsystem);

    //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-//
    private final DriveDefaultCommand mDriveDefaultCommand = new DriveDefaultCommand(mDriveSubsystem, mXboxController);
    private final BrasDefaultCommand mBrasDefaultCommand = new BrasDefaultCommand(mBrasSubsystem, mXboxController2);
    private final ExtendCommand mExtendCommand = new ExtendCommand(mExtensionSubsystem);
    private final PinceRotationDefaultCommand mPinceRotationDefaultCommand = new PinceRotationDefaultCommand(mPinceExtensionSubsystem, mXboxController);
    private final TrajectoryCommand mTrajectoryCommand = new TrajectoryCommand(mDriveSubsystem);
    private final OpenPinceBrasCommand mOpenPinceBrasCommand = new OpenPinceBrasCommand(mPinceBrasSubsystem);
    private final ClosePinceBrasCommand mClosePinceBrasCommand = new ClosePinceBrasCommand(mPinceBrasSubsystem);
    private final PositionMinPinceBrasCommand mPositionMinPinceBrasCommand = new PositionMinPinceBrasCommand(mPinceBrasSubsystem);
    private final PositionMaxPinceBrasCommand mPositionMaxPinceBrasCommand = new PositionMaxPinceBrasCommand(mPinceBrasSubsystem);
    private final PlieurDefaultCommand mGrimperDefaultCommand = new PlieurDefaultCommand(mGrimpeurSubsystem, mGrimpeurCordeSubsystem, mXboxController2);
    private final BrasPositionCommand mBrasPositionCommandTest = new BrasPositionCommand(mBrasSubsystem, -400);
    private final PincePositionMinExtCommand mPincePositionMinExtCommand = new PincePositionMinExtCommand(mPinceExtensionSubsystem);
    private final PincePositionMaxExtCommand mPincePositionMaxExtCommand = new PincePositionMaxExtCommand(mPinceExtensionSubsystem);

    private final CordePositionCommand mCordePositionCommand = new CordePositionCommand(mGrimpeurCordeSubsystem, 10000);

    private final PlieurPositionCommand mPlieurPositionCommand = new PlieurPositionCommand(mGrimpeurSubsystem, 4500);

    private final PlieurPositionCommand mPlieurPositionCommand0 = new PlieurPositionCommand(mGrimpeurSubsystem, 0);

    private final GrimpeAutoCommand mGrimpeAutoCommand = new GrimpeAutoCommand(mGrimpeurSubsystem, mGrimpeurCordeSubsystem);



    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    public void init() {
        //mCalibreBrasCommand.schedule();
    }

    private void configureButtonBindings() {
        JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        buttonA.whileTrue(mBrasPositionCommandTest);

        JoystickButton buttonLB = new JoystickButton(mXboxController, XboxController.Button.kLeftBumper.value);
         buttonLB.onTrue(mPlieurPositionCommand);

        JoystickButton buttonRB = new JoystickButton(mXboxController, XboxController.Button.kRightBumper.value);
        buttonRB.onTrue(mCordePositionCommand);

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.onTrue(mTrajectoryCommand);

        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.onTrue(mExtendCommand);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.onTrue(mGrimpeAutoCommand);

        //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-//

        JoystickButton buttonA2 = new JoystickButton(mXboxController2, XboxController.Button.kA.value);
        buttonA2.onTrue(mOpenPinceBrasCommand);

        JoystickButton buttonB2 = new JoystickButton(mXboxController2, XboxController.Button.kB.value);
        buttonB2.onTrue(mClosePinceBrasCommand);

        JoystickButton buttonX2 = new JoystickButton(mXboxController2, XboxController.Button.kX.value);
        buttonX2.onTrue(mPositionMinPinceBrasCommand);

        JoystickButton buttonY2 = new JoystickButton(mXboxController2, XboxController.Button.kY.value);
        buttonY2.onTrue(mPositionMaxPinceBrasCommand);

        JoystickButton buttonLB2 = new JoystickButton(mXboxController2, XboxController.Button.kLeftBumper.value);
        buttonLB2.onTrue(mPincePositionMinExtCommand);

        //JoystickButton buttonRB2 = new JoystickButton(mXboxController2, XboxController.Button.kRightBumper.value);
        //buttonRB2.onTrue(mPincePositionMaxExtCommand);

        //JoystickButton buttonA2 = new JoystickButton(mXboxController2, XboxController.Button.kA.value);
        //buttonA2.whileTrue(mBrasPanier1Command);
    }

    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveDefaultCommand);
        mBrasSubsystem.setDefaultCommand(mBrasDefaultCommand);
        mGrimpeurSubsystem.setDefaultCommand(mGrimperDefaultCommand);
        mPinceExtensionSubsystem.setDefaultCommand(mPinceRotationDefaultCommand);
    }

    public Command getAutonomousCommand() {
        return new BleuDroiteExtra(mDriveSubsystem);
    }

    public void stop() {
        mVisionSubsystem.close();
    }
}

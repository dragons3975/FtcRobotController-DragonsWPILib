package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.BleuDroiteExtra;
import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.GrimpeAutoCommand;
import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.PanierAutoCommand;
import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.RamasseurCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommandtest;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.CalibrationCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ClosePinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ExtendPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.OpenPinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionDefaultExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionMaxExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionMinExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PinceRotationCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.TogglePinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.CordePositionCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.PlieurPositionCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.ClosePinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ExtendDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.PlieurDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.OpenPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PinceRotationIncrementCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMaxPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMinPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.TogglePinceBrasCommand;
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

    private final CalibrationCommand mCalibrationCommand = new CalibrationCommand(mExtensionSubsystem);

    private final ClosePinceExtCommand mClosePinceExtCommand = new ClosePinceExtCommand(mPinceExtensionSubsystem);


    private final OpenPinceExtCommand mOpenPinceExtCommand = new OpenPinceExtCommand(mPinceExtensionSubsystem);

    private final DriveDefaultCommand mDriveDefaultCommand = new DriveDefaultCommand(mDriveSubsystem, mXboxController);
    private final BrasDefaultCommand mBrasDefaultCommand = new BrasDefaultCommand(mBrasSubsystem, mXboxController2);
    private final ExtendDefaultCommand mExtendDefaultCommand = new ExtendDefaultCommand(mExtensionSubsystem, mXboxController2);

    private final ExtendPositionCommand mExtendPositionCommandSol = new ExtendPositionCommand(mExtensionSubsystem, 800);
    private final ExtendPositionCommand mExtendPositionCommandZero = new ExtendPositionCommand(mExtensionSubsystem, 0);
    //private final PinceRotationIncrementCommand mPinceRotationIncrementCommand = new PinceRotationIncrementCommand(mPinceExtensionSubsystem, mXboxController2);
    private final TrajectoryCommand mTrajectoryCommand = new TrajectoryCommand(mDriveSubsystem);
    private final OpenPinceBrasCommand mOpenPinceBrasCommand = new OpenPinceBrasCommand(mPinceBrasSubsystem);
    private final ClosePinceBrasCommand mClosePinceBrasCommand = new ClosePinceBrasCommand(mPinceBrasSubsystem);
    private final PositionMinPinceBrasCommand mPositionMinPinceBrasCommand = new PositionMinPinceBrasCommand(mPinceBrasSubsystem);
    private final PositionMaxPinceBrasCommand mPositionMaxPinceBrasCommand = new PositionMaxPinceBrasCommand(mPinceBrasSubsystem);
    private final PlieurDefaultCommand mGrimperDefaultCommand = new PlieurDefaultCommand(mGrimpeurSubsystem, mGrimpeurCordeSubsystem, mXboxController2);
    private final BrasPositionCommandtest mBrasPositionCommandTest = new BrasPositionCommandtest(mBrasSubsystem, -20, 1);
    private final PincePositionMinExtCommand mPincePositionMinExtCommand = new PincePositionMinExtCommand(mPinceExtensionSubsystem);
    private final PincePositionMaxExtCommand mPincePositionMaxExtCommand = new PincePositionMaxExtCommand(mPinceExtensionSubsystem);

    private final CordePositionCommand mCordePositionCommand = new CordePositionCommand(mGrimpeurCordeSubsystem, 10000);

    private final PlieurPositionCommand mPlieurPositionCommand = new PlieurPositionCommand(mGrimpeurSubsystem, 4500);

    private final PlieurPositionCommand mPlieurPositionCommand0 = new PlieurPositionCommand(mGrimpeurSubsystem, 0);

    private final TogglePinceExtCommand mTogglePinceExtCommand = new TogglePinceExtCommand(mPinceExtensionSubsystem);

    private final TogglePinceBrasCommand mTogglePinceBras = new TogglePinceBrasCommand(mPinceBrasSubsystem);

    private final PinceRotationIncrementCommand mPinceRotationMax = new PinceRotationIncrementCommand(mPinceExtensionSubsystem, 0.01);
    private final PinceRotationIncrementCommand mPinceRotationMin = new PinceRotationIncrementCommand(mPinceExtensionSubsystem, -0.01);

    private final PincePositionDefaultExtCommand mPincePositionDefaultCommand = new PincePositionDefaultExtCommand(mPinceExtensionSubsystem, mXboxController2);

    private final GrimpeAutoCommand mGrimpeAutoCommand = new GrimpeAutoCommand(mGrimpeurSubsystem, mGrimpeurCordeSubsystem);
    private final PanierAutoCommand mPanierAutoCommand = new PanierAutoCommand(mBrasSubsystem, mPinceBrasSubsystem);

    private final RamasseurCommand mRamasseurAutoCommand = new RamasseurCommand(mExtensionSubsystem, mPinceExtensionSubsystem, mPinceBrasSubsystem, mBrasSubsystem);



    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    public void init() {
        //mCalibreBrasCommand.schedule();
    }

    private void configureButtonBindings() {
        JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        buttonA.onTrue(mOpenPinceExtCommand);

        JoystickButton buttonLB = new JoystickButton(mXboxController, XboxController.Button.kLeftBumper.value);
         buttonLB.onTrue(mClosePinceExtCommand);

        JoystickButton buttonRB = new JoystickButton(mXboxController, XboxController.Button.kRightBumper.value);
        buttonRB.onTrue(mCordePositionCommand);

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.onTrue(mBrasPositionCommandTest);

        //JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        //buttonX.onTrue();

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.onTrue(mCalibrationCommand);

        //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-// //-//

//        JoystickButton buttonA2 = new JoystickButton(mXboxController2, XboxController.Button.kA.value);
//        //buttonA2.onTrue(mOpenPinceBrasCommand);
//        buttonA2.onTrue(mExtendPositionCommandSol);
//
//        JoystickButton buttonB2 = new JoystickButton(mXboxController2, XboxController.Button.kB.value);
//        buttonB2.onTrue(mPincePositionMaxExtCommand);
//
//        JoystickButton buttonX2 = new JoystickButton(mXboxController2, XboxController.Button.kX.value);
//        buttonX2.onTrue(mPincePositionMinExtCommand);
//
//        JoystickButton buttonY2 = new JoystickButton(mXboxController2, XboxController.Button.kY.value);
//        buttonY2.onTrue(mPositionMaxPinceBrasCommand);
//
//        JoystickButton buttonLB2 = new JoystickButton(mXboxController2, XboxController.Button.kLeftBumper.value);
//        buttonLB2.onTrue(mRamasseurCommand);

        JoystickButton buttonY2 = new JoystickButton(mXboxController2, XboxController.Button.kY.value);
        buttonY2.onTrue(mTogglePinceExtCommand);

        JoystickButton buttonX2 = new JoystickButton(mXboxController2, XboxController.Button.kX.value);
        buttonX2.onTrue(mCalibrationCommand);

        JoystickButton buttonB2 = new JoystickButton(mXboxController2, XboxController.Button.kB.value);
        buttonB2.onTrue(mTogglePinceBras);

        JoystickButton buttonA2 = new JoystickButton(mXboxController2, XboxController.Button.kA.value);
        buttonA2.onTrue(mRamasseurAutoCommand);

        JoystickButton buttonLB2 = new JoystickButton(mXboxController2, XboxController.Button.kLeftBumper.value);
        buttonLB2.onTrue(mPanierAutoCommand);

        JoystickButton buttonStart = new JoystickButton(mXboxController2, XboxController.Button.kStart.value);
        buttonStart.whileTrue(mPinceRotationMax);

        JoystickButton buttonBack = new JoystickButton(mXboxController2, XboxController.Button.kBack.value);
        buttonBack.whileTrue(mPinceRotationMin);
    }

    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveDefaultCommand);
        mBrasSubsystem.setDefaultCommand(mBrasDefaultCommand);
        mGrimpeurSubsystem.setDefaultCommand(mGrimperDefaultCommand);
        mPinceExtensionSubsystem.setDefaultCommand(mPincePositionDefaultCommand);
        mExtensionSubsystem.setDefaultCommand(mExtendDefaultCommand);
    }

    public Command getAutonomousCommand() {
        return new BleuDroiteExtra(mDriveSubsystem);
    }

    public void stop() {
        mVisionSubsystem.close();
    }
}

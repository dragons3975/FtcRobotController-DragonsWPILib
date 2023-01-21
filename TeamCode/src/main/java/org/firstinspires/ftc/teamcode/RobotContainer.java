package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.GenericHID;
import org.firstinspires.ftc.dragonswpilib.command.ParallelRaceGroup;
import org.firstinspires.ftc.dragonswpilib.command.button.JoystickButton;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandGroups.DroiteAutonomousLeftCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.DroiteAutonomousMiddleCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.DroiteAutonomousRightCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.GaucheAutonomousLeftCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.GaucheAutonomousMiddleCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.GaucheAutonomousRightCommandGroup;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.AscenseurDefaultDeltaCommand;
import org.firstinspires.ftc.teamcode.commands.CallibrateAscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.FermerPinceCommand;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.commands.OuvrirPinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.dragonswpilib.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class RobotContainer {

    private final Gamepad mGamepad1, mGamepad2;
    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;

    private final DriveSubsystem mDriveSubsystem;
    private final AscenseurSubsystem mAscenseurSubsystem;
    private final PinceSubsystem mPinceSubsystem;
    private final VisionSubsystem mVisionSubsystem;

    private final DriveDefaultCommand mDriveDefaultCommand;

    private final ParallelRaceGroup mOuvrirPinceCommandWithTimeout;
    private final ParallelRaceGroup mFermerPinceCommandWithTimeout;

    private final GaucheAutonomousMiddleCommandGroup mGaucheAutonomousMiddleCommandGroup;
    private final GaucheAutonomousLeftCommandGroup mGaucheAutonomousLeftCommandGroup;
    private final GaucheAutonomousRightCommandGroup mGaucheAutonomousRightCommandGroup;
    private final DroiteAutonomousMiddleCommandGroup mDroiteAutonomousMiddleCommandGroup;
    private final DroiteAutonomousLeftCommandGroup mDroiteAutonomousLeftCommandGroup;
    private final DroiteAutonomousRightCommandGroup mDroiteAutonomousRightCommandGroup;

    private final GoToAngleCommand mGoStraight;
    private final GoToAngleCommand mGoLeft;
    private final GoToAngleCommand mGoRight;
    private final GoToAngleCommand mGoBack;

    private final CallibrateAscenseurCommand mCallibrateAscenseurCommand;
    //private final ManualOverideCalibrateAscenseurCommand mManualOverideCalibrateAscenseurCommand;

    //private final AscenseurDescendreDeltaCommand mAscenseurDescendreDeltaCommand;
    private final AscenseurDefaultDeltaCommand mAscenseurDefaultDeltaCommand;
    //private final AscenseurMonterDeltaCommand mAscenseurMonterDeltaCommand;

    //private final AscenseurGaucheManualDescendreCommand mAscenseurGaucheManualDescendreCommand;
    //private final AscenseurGaucheManualMonterCommand mAscenseurGaucheManualMonterCommand;

    //private final AscenseurManualDescendreCommand mAscenseurManualDescendreCommand;
    //private final AscenseurManualMonterCommand mAscenseurManualMonterCommand;

    //private final PickUpSequentialCommandGroup mPickUpAndCalibrate;

    private final AscenseurCommand mGround;
    private final AscenseurCommand mLow;
    private final AscenseurCommand mMedium;
    private final AscenseurCommand mHigh;

    public RobotContainer(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap){
        mGamepad1 = gamepad1;
        mGamepad2 = gamepad2;
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mDriveSubsystem = new DriveSubsystem(mHardwareMap, mTelemetry);
        mAscenseurSubsystem = new AscenseurSubsystem(mHardwareMap, mTelemetry);
        mPinceSubsystem = new PinceSubsystem(mHardwareMap, mTelemetry);
        mVisionSubsystem = new VisionSubsystem(mHardwareMap, mTelemetry);
        mVisionSubsystem.enableVision(); //Il faudrait le désactiver quand on est en teleop

        mOuvrirPinceCommandWithTimeout = new OuvrirPinceCommand(mTelemetry, mPinceSubsystem).withTimeout(Constants.PinceConstants.kOuvrirFermerPinceTimeout);;
        mFermerPinceCommandWithTimeout = new FermerPinceCommand(mTelemetry, mPinceSubsystem).withTimeout(Constants.PinceConstants.kOuvrirFermerPinceTimeout);

        mDriveDefaultCommand = new DriveDefaultCommand(mTelemetry, mDriveSubsystem, mGamepad1);

        mGaucheAutonomousMiddleCommandGroup = new GaucheAutonomousMiddleCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);
        mGaucheAutonomousLeftCommandGroup = new GaucheAutonomousLeftCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);
        mGaucheAutonomousRightCommandGroup = new GaucheAutonomousRightCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);

        mDroiteAutonomousMiddleCommandGroup = new DroiteAutonomousMiddleCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);
        mDroiteAutonomousLeftCommandGroup = new DroiteAutonomousLeftCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);
        mDroiteAutonomousRightCommandGroup = new DroiteAutonomousRightCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);

        mGoStraight = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 0);
        mGoLeft = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 90);
        mGoRight = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, -90);
        mGoBack = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 180);

        mCallibrateAscenseurCommand = new CallibrateAscenseurCommand(mTelemetry, mAscenseurSubsystem);
        //mCallibrateAscenseurCommand.schedule(); //Calibrer automatiquement au demarrage du robot
        //mManualOverideCalibrateAscenseurCommand = new ManualOverideCalibrateAscenseurCommand(mTelemetry, mAscenseurSubsystem);

        //mAscenseurMonterDeltaCommand = new AscenseurMonterDeltaCommand(mTelemetry, mAscenseurSubsystem);
        mAscenseurDefaultDeltaCommand= new AscenseurDefaultDeltaCommand(mTelemetry, mAscenseurSubsystem, mGamepad2);
        //mAscenseurDescendreDeltaCommand = new AscenseurDescendreDeltaCommand(mTelemetry, mAscenseurSubsystem);

        //mAscenseurGaucheManualDescendreCommand = new AscenseurGaucheManualDescendreCommand(mTelemetry, mAscenseurSubsystem);
        //mAscenseurGaucheManualMonterCommand = new AscenseurGaucheManualMonterCommand(mTelemetry, mAscenseurSubsystem);

        //mAscenseurManualMonterCommand = new AscenseurManualMonterCommand(mTelemetry, mAscenseurSubsystem);
        //mAscenseurManualDescendreCommand = new AscenseurManualDescendreCommand(mTelemetry, mAscenseurSubsystem);

        mGround = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionSol);
        mLow = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionBas);
        mMedium = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionMoyen);
        mHigh = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);

        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {
        JoystickButton buttonY = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kY);
        buttonY.onTrue(mGoStraight);
        JoystickButton buttonX = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kX);
        buttonX.onTrue(mGoLeft);
        JoystickButton buttonB = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kB);
        buttonB.onTrue(mGoRight);
        JoystickButton buttonA = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kA);
        buttonA.onTrue(mGoBack);

        JoystickButton buttonY2 = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kY);
        buttonY2.onTrue(mHigh);
        JoystickButton buttonX2 = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kX);
        buttonX2.onTrue(mMedium);
        JoystickButton buttonB2 = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kB);
        buttonB2.onTrue(mLow);
        JoystickButton buttonA2 = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kA);
        buttonA2.onTrue(mGround);

        JoystickButton DPadDown = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kDpadDown);
        DPadDown.onTrue(mCallibrateAscenseurCommand);

        JoystickButton DPadLeft2 = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kDpadLeft);
        DPadLeft2.onTrue(mOuvrirPinceCommandWithTimeout);
        JoystickButton DPadRight2 = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kDpadRight);
        DPadRight2.onTrue(mFermerPinceCommandWithTimeout);

        //JoystickButton LB = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kLeftBumper);
        //LB.whileTrue(mAscenseurMonterCommand);
        //LB.whileTrue(mAscenseurManualMonterCommand);
        //JoystickButton RB = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kRightBumper);
        //RB.whileTrue(mAscenseurDescendreCommand);
        //RB.whileTrue(mAscenseurManualDescendreCommand);
        //JoystickButton start = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kStart);
        //start.onTrue(mManualOverideCalibrateAscenseurCommand);
        //JoystickButton LT = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kLeftTrigger);
        //LT.whileTrue(mAscenseurGaucheManualMonterCommand);
        //JoystickButton RT = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kRightTrigger);
        //RT.whileTrue(mAscenseurGaucheManualDescendreCommand);
    }

    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveDefaultCommand);
        mAscenseurSubsystem.setDefaultCommand(mAscenseurDefaultDeltaCommand);
    }

    public Command getAutonomousCommand() {
        //Stop detection
        mVisionSubsystem.disableVision();

        boolean gauche = true;
        if(gauche){
            switch (mVisionSubsystem.getAutonomousPosition()) {
                case 1:
                    return mGaucheAutonomousLeftCommandGroup;
                case 2:
                default:
                    return mGaucheAutonomousMiddleCommandGroup;
                case 3:
                    return mGaucheAutonomousRightCommandGroup;
            }
        }
        else {
            switch (mVisionSubsystem.getAutonomousPosition()) {
                case 1:
                    return mDroiteAutonomousLeftCommandGroup;
                case 2:
                default:
                    return mDroiteAutonomousMiddleCommandGroup;
                case 3:
                    return mDroiteAutonomousRightCommandGroup;
            }
        }
      }

    }

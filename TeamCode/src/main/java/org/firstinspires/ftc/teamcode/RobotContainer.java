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
import org.firstinspires.ftc.teamcode.commands.TogglePositionDepartCommand;
import org.firstinspires.ftc.teamcode.commands.tests.AscenseurDroitManualDescendreCommand;
import org.firstinspires.ftc.teamcode.commands.tests.AscenseurDroitManualMonterCommand;
import org.firstinspires.ftc.teamcode.commands.tests.AscenseurGaucheManualDescendreCommand;
import org.firstinspires.ftc.teamcode.commands.tests.AscenseurGaucheManualMonterCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ConfigSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.dragonswpilib.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class RobotContainer {

    private final Gamepad mGamepad1, mGamepad2;
    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;
    private final Constants.ModesConstants.Modes mMode;

    private final DriveSubsystem mDriveSubsystem;
    private final AscenseurSubsystem mAscenseurSubsystem;
    private final PinceSubsystem mPinceSubsystem;
    private final ConfigSubsystem mConfigSubsystem;
    private final VisionSubsystem mVisionSubsystem;

    private final DriveDefaultCommand mDriveDefaultCommand;

    private final ParallelRaceGroup mOuvrirPinceCommandWithTimeout;
    private final ParallelRaceGroup mFermerPinceCommandWithTimeout;

    private final GoToAngleCommand mGoStraight;
    private final GoToAngleCommand mGoLeft;
    private final GoToAngleCommand mGoRight;
    private final GoToAngleCommand mGoBack;

    private final CallibrateAscenseurCommand mCallibrateAscenseurCommand;

    private final AscenseurDefaultDeltaCommand mAscenseurDefaultDeltaCommand;

    private final AscenseurCommand mGround;
    private final AscenseurCommand mLow;
    private final AscenseurCommand mMedium;
    private final AscenseurCommand mHigh;

    private final TogglePositionDepartCommand mTogglePositionDepartCommand;


    public RobotContainer(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap, Constants.ModesConstants.Modes mode) {
        mGamepad1 = gamepad1;
        mGamepad2 = gamepad2;
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
        mMode = mode;

        mDriveSubsystem = new DriveSubsystem(mHardwareMap, mTelemetry);
        mAscenseurSubsystem = new AscenseurSubsystem(mHardwareMap, mTelemetry);
        mPinceSubsystem = new PinceSubsystem(mHardwareMap, mTelemetry);
        mConfigSubsystem = new ConfigSubsystem(mTelemetry);
        mVisionSubsystem = new VisionSubsystem(mHardwareMap, mTelemetry);

        if (mMode == Constants.ModesConstants.Modes.auto) {
            mVisionSubsystem.enableVision(); //Pas 100% correct, le thread est quand même crée, il faut trouver une autre façon de ne pas le créer en teleop
        }

        mOuvrirPinceCommandWithTimeout = new OuvrirPinceCommand(mTelemetry, mPinceSubsystem).withTimeout(Constants.PinceConstants.kOuvrirFermerPinceTimeout);
        mFermerPinceCommandWithTimeout = new FermerPinceCommand(mTelemetry, mPinceSubsystem).withTimeout(Constants.PinceConstants.kOuvrirFermerPinceTimeout);

        mDriveDefaultCommand = new DriveDefaultCommand(mTelemetry, mDriveSubsystem, mGamepad1);

        mGoStraight = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 0);
        mGoLeft = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 90);
        mGoRight = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, -90);
        mGoBack = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 180);

        mCallibrateAscenseurCommand = new CallibrateAscenseurCommand(mTelemetry, mAscenseurSubsystem);
        //mCallibrateAscenseurCommand.schedule(); //Calibrer automatiquement au demarrage du robot, mais pas à chaque programme

        mAscenseurDefaultDeltaCommand= new AscenseurDefaultDeltaCommand(mTelemetry, mAscenseurSubsystem, mGamepad2);

        mGround = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionSol);
        mLow = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionBas);
        mMedium = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionMoyen);
        mHigh = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);

        mTogglePositionDepartCommand = new TogglePositionDepartCommand(mConfigSubsystem);

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

        JoystickButton buttonStart = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kStart);
        buttonStart.onTrue(mTogglePositionDepartCommand);

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

        if (mMode == Constants.ModesConstants.Modes.test) {
            JoystickButton buttonLB = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kLeftBumper);
            buttonLB.whileTrue(new AscenseurGaucheManualMonterCommand(mTelemetry, mAscenseurSubsystem));
            JoystickButton buttonLT = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kLeftTrigger);
            buttonLT.whileTrue(new AscenseurGaucheManualDescendreCommand(mTelemetry, mAscenseurSubsystem));
            JoystickButton buttonRB = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kRightBumper);
            buttonRB.whileTrue(new AscenseurDroitManualMonterCommand(mTelemetry, mAscenseurSubsystem));
            JoystickButton buttonRT = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kRightTrigger);
            buttonRT.whileTrue(new AscenseurDroitManualDescendreCommand(mTelemetry, mAscenseurSubsystem));
        }
    }

    private void configureDefaultCommands() {
        if (mMode == Constants.ModesConstants.Modes.teleop) {
            mDriveSubsystem.setDefaultCommand(mDriveDefaultCommand);
            mAscenseurSubsystem.setDefaultCommand(mAscenseurDefaultDeltaCommand);
        }
    }

    public Command getAutonomousCommand() {
        //Stop detection
        mVisionSubsystem.disableVision();

        if(mConfigSubsystem.isPositionDepartGauche()){
            switch (mVisionSubsystem.getAutonomousPosition()) {
                case 1:
                    return new GaucheAutonomousLeftCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);
                case 2:
                default:
                    return new GaucheAutonomousMiddleCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);
                case 3:
                    return new GaucheAutonomousRightCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);
            }
        }
        else {
            switch (mVisionSubsystem.getAutonomousPosition()) {
                case 1:
                    return new DroiteAutonomousLeftCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);
                case 2:
                default:
                    return new DroiteAutonomousMiddleCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);
                case 3:
                    return new DroiteAutonomousRightCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mPinceSubsystem, mGamepad1);
            }
        }
      }

    }

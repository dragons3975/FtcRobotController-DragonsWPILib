package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.GenericHID;
import org.firstinspires.ftc.dragonswpilib.command.button.JoystickButton;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandGroups.GaucheAutonomousLeftCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.GaucheAutonomousMiddleCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.GaucheAutonomousRightCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.ConeSequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.AscenseurDescendreCommand;
import org.firstinspires.ftc.teamcode.commands.AscenseurMonterCommand;
import org.firstinspires.ftc.teamcode.commands.CallibrateAscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FermerPinceCommand;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.commands.OuvrirPinceCommand;
import org.firstinspires.ftc.teamcode.commands.RefreshVisionCommand;
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
    private final DriveCommand mDriveCommand;
    private final AscenseurSubsystem mAscenseurSubsystem;
    private final AscenseurDescendreCommand mAscenseurDescendreCommand;
    private final AscenseurMonterCommand mAscenseurMonterCommand;
    private final PinceSubsystem mPinceSubsystem;
    private final OuvrirPinceCommand mOuvrirPinceCommand;
    private final FermerPinceCommand mFermerPinceCommand;

    private final VisionSubsystem mVisionSubsystem;
    private final RefreshVisionCommand mRefreshVisionCommand;

    private final GaucheAutonomousMiddleCommandGroup mGaucheAutonomousMiddleCommandGroup;
    private final GaucheAutonomousLeftCommandGroup mGaucheAutonomousLeftCommandGroup;
    private final GaucheAutonomousRightCommandGroup mGaucheAutonomousRightCommandGroup;

    private final ConeSequentialCommandGroup mConeSequentialCommandGroup;

    private final GoToAngleCommand mGoStraight;
    private final GoToAngleCommand mGoLeft;
    private final GoToAngleCommand mGoRight;
    private final GoToAngleCommand mGoBack;
    private final AscenseurCommand mPickUp;
    private final AscenseurCommand mLow;
    private final AscenseurCommand mMedium;
    private final AscenseurCommand mHigh;

    private final CallibrateAscenseurCommand mCallibrateAscenseurCommand;

    public RobotContainer(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap){
        mGamepad1 = gamepad1;
        mGamepad2 = gamepad2;
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mDriveSubsystem = new DriveSubsystem(mHardwareMap, mTelemetry);
        mAscenseurSubsystem = new AscenseurSubsystem(mHardwareMap, mTelemetry);
        mAscenseurMonterCommand = new AscenseurMonterCommand(mTelemetry, mAscenseurSubsystem);
        mAscenseurDescendreCommand = new AscenseurDescendreCommand(mTelemetry, mAscenseurSubsystem);

        mCallibrateAscenseurCommand = new CallibrateAscenseurCommand(mTelemetry, mAscenseurSubsystem);
        mCallibrateAscenseurCommand.schedule();//***Callibrer automatiquement à chaque fois que nous descendons

        mPinceSubsystem = new PinceSubsystem(mHardwareMap, mTelemetry, mGamepad2);
        mOuvrirPinceCommand = new OuvrirPinceCommand(mTelemetry, mPinceSubsystem);
        mFermerPinceCommand = new FermerPinceCommand(mTelemetry, mPinceSubsystem);

        mVisionSubsystem = new VisionSubsystem(mHardwareMap, mTelemetry);
        mRefreshVisionCommand = new RefreshVisionCommand(mTelemetry, mVisionSubsystem);
        mRefreshVisionCommand.schedule();//***ne pas scheduler lors du mode teleop
        mDriveCommand = new DriveCommand(mTelemetry, mDriveSubsystem, mGamepad1);
        mGaucheAutonomousMiddleCommandGroup = new GaucheAutonomousMiddleCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mGamepad1);
        mGaucheAutonomousLeftCommandGroup = new GaucheAutonomousLeftCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mGamepad1);
        mGaucheAutonomousRightCommandGroup = new GaucheAutonomousRightCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mGamepad1);

        mConeSequentialCommandGroup = new ConeSequentialCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem);

        mGoStraight = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 0);
        mGoLeft = new GoToAngleCommand(mTelemetry,mDriveSubsystem,mGamepad1, 90);
        mGoRight = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, -90);
        mGoBack = new GoToAngleCommand(mTelemetry, mDriveSubsystem, mGamepad1, 180);

        mPickUp = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionSol);
        mLow = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionBas);
        mMedium = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionMoyen);
        mHigh = new AscenseurCommand(mTelemetry, mAscenseurSubsystem, Constants.AscenseurConstants.kPositionHaut);


        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {
        JoystickButton start = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kStart);
        start.onTrue(mGaucheAutonomousMiddleCommandGroup);

        JoystickButton buttonY = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kY);
        buttonY.onTrue(mGoStraight);
        JoystickButton buttonX = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kX);
        buttonX.onTrue(mGoLeft);
        JoystickButton buttonB = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kB);
        buttonB.onTrue(mGoRight);
        JoystickButton buttonA = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kA);
        buttonA.onTrue(mGoBack);

        //JoystickButton button2A = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kA);
        //button2A.onTrue(mConeSequentialCommandGroup);


        JoystickButton RT = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kRightTrigger);
        RT.whileTrue(mAscenseurMonterCommand);
        JoystickButton LT = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kLeftTrigger);
        LT.whileTrue(mAscenseurDescendreCommand);


        JoystickButton buttonA2 = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kA);
        buttonA2.onTrue(mOuvrirPinceCommand);
        JoystickButton buttonB2 = new JoystickButton(mGamepad2, GenericHID.XboxControllerConstants.kB);
        buttonB2.onTrue(mFermerPinceCommand);
    }

    private void configureDefaultCommands(){
        mDriveSubsystem.setDefaultCommand(mDriveCommand);

    }

    public Command getAutonomousCommand() {
        return mGaucheAutonomousLeftCommandGroup;
       /* mRefreshVisionCommand.cancel();
      int  position = mVisionSubsystem.getAutonomousPosition();
      switch(position) {
          default:
          case 1: return mAutonomousMiddleCommandGroup;
          case 2: return mAutonomousLeftCommandGroup;
*/
      }
    }


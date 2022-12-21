package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.GenericHID;
import org.firstinspires.ftc.dragonswpilib.command.button.JoystickButton;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousLeftCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousMiddleCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousRightCommandGroup;
import org.firstinspires.ftc.teamcode.commands.AscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.CallibrateAscenseurCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand;
import org.firstinspires.ftc.teamcode.commands.RefreshVisionCommand;
import org.firstinspires.ftc.teamcode.commands.TestGabrielCommand;
import org.firstinspires.ftc.teamcode.subsystems.AscenseurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.dragonswpilib.command.Command;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.testGabriel;

public class RobotContainer {

    private final Gamepad mGamepad1, mGamepad2;
    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;

    private final DriveSubsystem mDriveSubsystem;
    private final DriveCommand mDriveCommand;
    private final TestGabrielCommand mTestGabrielCommand;
    private final testGabriel mTestGabriel;
    private final AscenseurSubsystem mAscenseurSubsystem;
    private final VisionSubsystem mVisionSubsystem;
    private final RefreshVisionCommand mRefreshVisionCommand;

    private final AutonomousMiddleCommandGroup mAutonomousMiddleCommandGroup;
    private final AutonomousLeftCommandGroup mAutonomousLeftCommandGroup;
    private final AutonomousRightCommandGroup mAutonomousRightCommandGroup;
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
        mTestGabriel = new testGabriel(mHardwareMap, mTelemetry);

        mTestGabrielCommand = new TestGabrielCommand(mTelemetry, mTestGabriel, mGamepad1);
        mCallibrateAscenseurCommand = new CallibrateAscenseurCommand(mTelemetry, mAscenseurSubsystem);
        mCallibrateAscenseurCommand.schedule();//***Callibrer automatiquement à chaque fois que nous descendons


        mVisionSubsystem = new VisionSubsystem(mHardwareMap, mTelemetry);
        mRefreshVisionCommand = new RefreshVisionCommand(mTelemetry, mVisionSubsystem);
        mRefreshVisionCommand.schedule();//***ne pas scheduler lors du mode teleop
        mDriveCommand = new DriveCommand(mTelemetry, mDriveSubsystem, mGamepad1);
        mAutonomousMiddleCommandGroup = new AutonomousMiddleCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mGamepad1);
        mAutonomousLeftCommandGroup = new AutonomousLeftCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mGamepad1);
        mAutonomousRightCommandGroup = new AutonomousRightCommandGroup(mTelemetry, mDriveSubsystem, mAscenseurSubsystem, mGamepad1);

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
        start.onTrue(mAutonomousMiddleCommandGroup);

        JoystickButton buttonY = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kY);
        buttonY.onTrue(mGoStraight);
        JoystickButton buttonX = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kX);
        buttonX.onTrue(mGoLeft);
        JoystickButton buttonB = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kB);
        buttonB.onTrue(mGoRight);
        JoystickButton buttonA = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kA);
        buttonA.onTrue(mGoBack);
    }

    private void configureDefaultCommands(){
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
        mTestGabriel.setDefaultCommand(mTestGabrielCommand);

    }

    public Command getAutonomousCommand() {
        mRefreshVisionCommand.cancel();
      int  position = mVisionSubsystem.getAutonomousPosition();
      switch(position) {
          default:
          case 1: return mAutonomousMiddleCommandGroup;
          case 2: return mAutonomousLeftCommandGroup;

      }
    }
}

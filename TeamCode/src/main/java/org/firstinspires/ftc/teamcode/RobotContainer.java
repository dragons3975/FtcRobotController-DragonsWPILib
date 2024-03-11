package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurFermeCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurOuvreCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PinceCommandToggle;
import org.firstinspires.ftc.teamcode.commands.PixelDetectionStopCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
import org.firstinspires.ftc.teamcode.commands.CalibreBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.PixelDetectionSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();

    private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();

    private final PixelDetectionSubsystem mPixelDetectionSubsystem = new PixelDetectionSubsystem();

    private final  PinceSubsystem mPinceSubsystem = new PinceSubsystem();
    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);

    private final PixelDetectionStopCommand mPixelDetectionStopCommand = new PixelDetectionStopCommand(mPixelDetectionSubsystem);

    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final BrasCommandPos1 mBrasCommandPos1 = new BrasCommandPos1(mBrasSubsystem, 0);

    private final BrasCommandPos1 mBrasCommandPos2 = new BrasCommandPos1(mBrasSubsystem, 330);


   private final BrasCommand mBrasCommand = new BrasCommand(mBrasSubsystem, mXboxController2);

    private final IntakeCommand mIntakeCommand = new IntakeCommand(mIntakeSubsystem);

    private final CalibreBrasCommand mCalibreBrasCommand = new CalibreBrasCommand(mBrasSubsystem);

    private final PinceCommandToggle mPinceCommandToggle = new PinceCommandToggle(mPinceSubsystem);

    private final GrimpeurSubsystem mGrimpeurSubsystem = new GrimpeurSubsystem();

    private final GrimpeurOuvreCommand mGrimpeurOuvreCommand = new GrimpeurOuvreCommand(mGrimpeurSubsystem);

    private final GrimpeurFermeCommand mGrimpeurFermeCommand = new GrimpeurFermeCommand(mGrimpeurSubsystem);

    private final AutonomousCommandGroup mAutonomousCommandGroup = new AutonomousCommandGroup(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mPixelDetectionSubsystem);
    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.whileTrue(mIntakeCommand);

        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        JoystickButton buttonX2 = new JoystickButton(mXboxController2, XboxController.Button.kX.value);
        buttonX2.whileTrue(mBrasCommandPos1);

        JoystickButton buttonB2 = new JoystickButton(mXboxController2, XboxController.Button.kB.value);
        buttonB2.onTrue(mPinceCommandToggle);

        JoystickButton buttonY2 = new JoystickButton(mXboxController2, XboxController.Button.kY.value);
        buttonY2.whileTrue(mBrasCommandPos2);



    }
    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
        mBrasSubsystem.setDefaultCommand(mBrasCommand);
    }

    public Command getAutonomousCommand() {

        return mAutonomousCommandGroup;
   }
}

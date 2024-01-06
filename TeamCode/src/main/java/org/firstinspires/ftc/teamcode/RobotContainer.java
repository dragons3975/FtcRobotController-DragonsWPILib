package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commandGroups.AvancerAutoCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos2;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PinceCommandFerme;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.commands.PinceCommandBaisse;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
import org.firstinspires.ftc.teamcode.commands.CalibreBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceCommandFerme;
import org.firstinspires.ftc.teamcode.commands.PinceCommandOuvre;
import org.firstinspires.ftc.teamcode.commands.BrasCommandExtention;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();

    private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();

    private final  PinceSubsystem mPinceSubsystem = new PinceSubsystem();
    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);

    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final BrasCommandPos1 mBrasCommandPos1 = new BrasCommandPos1(mBrasSubsystem, 0);

    private final BrasCommandPos2 mBrasCommandPos2 = new BrasCommandPos2(mBrasSubsystem, 330);


   private final BrasCommand mBrasCommand = new BrasCommand(mBrasSubsystem, mXboxController2);

    private final IntakeCommand mIntakeCommand = new IntakeCommand(mIntakeSubsystem);

    private final PinceCommandBaisse mPinceCommandBaisse = new PinceCommandBaisse(mPinceSubsystem, mXboxController2);

    private final PinceCommandFerme mPinceCommandFerme = new PinceCommandFerme(mPinceSubsystem, mXboxController2);
    private final PinceCommandOuvre mPinceCommandOuvre = new PinceCommandOuvre(mPinceSubsystem, mXboxController2);
    private final CalibreBrasCommand mCalibreBrasCommand = new CalibreBrasCommand(mBrasSubsystem);

    //private final AvancerAutoCommand mAvancerAutoCommand = new AvancerAutoCommand(mDriveSubsystem, 0, 0, 0);

    private final AutonomousCommandGroup mAutonomousCommandGroup = new AutonomousCommandGroup(mDriveSubsystem);;

    private final AvancerAutoCommand mAvancerAutoCommand = new AvancerAutoCommand(mDriveSubsystem);

    private final BrasCommandExtention mBrasCommandExtention = new BrasCommandExtention(mBrasSubsystem, mXboxController, -1);

    private final BrasCommandExtention mBrasCommandRetention = new BrasCommandExtention(mBrasSubsystem, mXboxController, 1);
    public RobotContainer() {

        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {
        JoystickButton buttonY2 = new JoystickButton(mXboxController2, XboxController.Button.kY.value);
        buttonY2.whileTrue(mBrasCommandPos1);

        //JoystickButton buttonX = new JoystickButton(mXboxController2, XboxController.Button.kX.value);
        //buttonX.whileTrue(mBrasCommandPos2);


        //JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        //buttonB.whileTrue(mIntakeCommand);

        JoystickButton buttonA = new JoystickButton(mXboxController, XboxController.Button.kA.value);
        buttonA.whileTrue(mBrasCommandExtention);

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.whileTrue(mBrasCommandRetention);

        ////////

        JoystickButton buttonA2 = new JoystickButton(mXboxController2, XboxController.Button.kA.value);
        buttonA2.toggleOnTrue(mPinceCommandFerme);

        JoystickButton buttonB2 = new JoystickButton(mXboxController2, XboxController.Button.kB.value);
        buttonB2.toggleOnTrue(mPinceCommandOuvre);

        JoystickButton buttonX2 = new JoystickButton(mXboxController2, XboxController.Button.kX.value);
        buttonX2.whileTrue(mCalibreBrasCommand);




    }
    private void configureDefaultCommands() {
        //mDriveSubsystem.setDefaultCommand(mDriveCommand);
        mBrasSubsystem.setDefaultCommand(mBrasCommand);
        mPinceSubsystem.setDefaultCommand(mPinceCommandBaisse);
    }

    public Command getAutonomousCommand() {
        return mAvancerAutoCommand;
   }
}

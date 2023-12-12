package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
//import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
//import org.firstinspires.ftc.teamcode.commands.BrasCommandPos2;
import org.firstinspires.ftc.teamcode.commands.Arm_Down_CMD;
import org.firstinspires.ftc.teamcode.commands.Arm_Up_CMD;
import org.firstinspires.ftc.teamcode.commands.BrasCommand;
import org.firstinspires.ftc.teamcode.commands.BrasDownCommand;
import org.firstinspires.ftc.teamcode.commands.BrasStopCommand;
import org.firstinspires.ftc.teamcode.commands.BrasUpCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.FlyPlane_CMD;
import org.firstinspires.ftc.teamcode.subsystems.Arm_SS;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher_SS;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.commands.BrasCommand;
//import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
//import org.firstinspires.ftc.teamcode.commands.PinceCommand;
// import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
//import org.firstinspires.ftc.teamcode.commands.CalibreBrasCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class  RobotContainer {

    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();

   // private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();

    //private final  PinceSubsystem mPinceSubsystem = new PinceSubsystem();
    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);

    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    //  private final BrasCommandPos1 mBrasCommandPos1 = new BrasCommandPos1(mBrasSubsystem, 0);

    //private final BrasCommandPos2 mBrasCommandPos2 = new BrasCommandPos2(mBrasSubsystem, 330);


    private final BrasCommand mBrasCommand = new BrasCommand(mBrasSubsystem, mXboxController);

    private final BrasUpCommand mBrasUpCommand = new BrasUpCommand(mBrasSubsystem, mXboxController);

    private final BrasDownCommand mBrasDownCommand = new BrasDownCommand(mBrasSubsystem, mXboxController);

    private final BrasStopCommand mBrasStopCommand = new BrasStopCommand(mBrasSubsystem, mXboxController);

    // private final IntakeCommand mIntakeCommand = new IntakeCommand(mIntakeSubsystem);

    //// private final  PinceCommand mPinceCommand = new PinceCommand(mPinceSubsystem);

    // private final CalibreBrasCommand mCalibreBrasCommand = new CalibreBrasCommand(mBrasSubsystem);

    //private final AvancerAutoCommand mAvancerAutoCommand = new AvancerAutoCommand(mDriveSubsystem, 0, 0, 0);


    private final Arm_SS m_ArmSS = new Arm_SS();
    private final Arm_Up_CMD m_Arm_Up_CMD = new Arm_Up_CMD(m_ArmSS, mXboxController);
    private final Arm_Down_CMD m_Arm_Down_CMD = new Arm_Down_CMD(m_ArmSS, mXboxController);


    private final PlaneLauncher_SS m_PlaneLauncher_SSS = new PlaneLauncher_SS();
    private final FlyPlane_CMD m_FlyPlane_CMD = new FlyPlane_CMD(m_PlaneLauncher_SSS, mXboxController);


    private final AutonomousCommandGroup mAutonomousCommandGroup;

    public RobotContainer() {

        mAutonomousCommandGroup = new AutonomousCommandGroup(mDriveSubsystem);
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {

        /*

         JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
         buttonX.whileTrue(mBrasStopCommand);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
         buttonY.whileTrue(mBrasUpCommand);

         JoystickButton buttonA = new JoystickButton(mXboxController  , XboxController.Button.kA.value);
          buttonA.onTrue(mBrasDownCommand);

        // JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        //  buttonB.whileTrue(mIntakeCommand);
*/
        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.whileTrue(m_Arm_Up_CMD);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.whileTrue(m_Arm_Down_CMD);

        JoystickButton buttonA = new JoystickButton(mXboxController  , XboxController.Button.kA.value);
        buttonA.onTrue(m_FlyPlane_CMD);

    }
    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
        mBrasSubsystem.setDefaultCommand(mBrasStopCommand);
    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
   }
}

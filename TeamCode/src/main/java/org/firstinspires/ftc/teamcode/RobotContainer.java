package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.BleuDroiteTeamPropDroite;
import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.BleuDroiteTeamPropGauche;
import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite.BleuDroiteTeamPropMilieu;
import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Gauche.BleuGaucheTeamPropDroite;
import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Gauche.BleuGaucheTeamPropGauche;
import org.firstinspires.ftc.teamcode.commandGroups.Bleu.Gauche.BleuGaucheTeamPropMilieu;
import org.firstinspires.ftc.teamcode.commandGroups.Rouge.Droite.RougeDroiteTeamPropMilieu;
import org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche.RougeGaucheTeamPropDroite;
import org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche.RougeGaucheTeamPropGauche;
import org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche.RougeGaucheTeamPropMilieu;
import org.firstinspires.ftc.teamcode.commandGroups.Rouge.Droite.RougeDroiteTeamPropDroite;
import org.firstinspires.ftc.teamcode.commandGroups.Rouge.Droite.RougeDroiteTeamPropGauche;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.ToggleAllianceColorCommand;
import org.firstinspires.ftc.teamcode.commands.ToggleAlliancePositionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PinceCommandToggle;
import org.firstinspires.ftc.teamcode.commands.PixelDetectionStopCommand;
import org.firstinspires.ftc.teamcode.commands.ToggleTemporaryTeamPropPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.ConfigSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
import org.firstinspires.ftc.teamcode.commands.CalibreBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
    private ConfigSubsystem mConfigSubsystem = new ConfigSubsystem();
    private VisionSubsystem mVisionSubsystem = new VisionSubsystem(mConfigSubsystem);

    private final  PinceSubsystem mPinceSubsystem = new PinceSubsystem();
    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);


    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final BrasCommandPos1 mBrasCommandPos1 = new BrasCommandPos1(mBrasSubsystem, 0);

    private final BrasCommandPos1 mBrasCommandPos2 = new BrasCommandPos1(mBrasSubsystem, 330);

   private final BrasCommand mBrasCommand = new BrasCommand(mBrasSubsystem, mXboxController2);

    private final IntakeCommand mIntakeCommand = new IntakeCommand(mIntakeSubsystem);


    private final PinceCommandToggle mPinceCommandToggle = new PinceCommandToggle(mPinceSubsystem);

    private final ToggleAllianceColorCommand mToggleAllianceColorCommand = new ToggleAllianceColorCommand(mConfigSubsystem);
    private final ToggleAlliancePositionCommand mToggleAlliancePositionCommand = new ToggleAlliancePositionCommand(mConfigSubsystem);
    private final ToggleTemporaryTeamPropPositionCommand mToggleTemporaryTeamPropPositionCommand = new ToggleTemporaryTeamPropPositionCommand(mConfigSubsystem);

    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureButtonBindings() {

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.whileTrue(mIntakeCommand);


        JoystickButton kStart = new JoystickButton(mXboxController, XboxController.Button.kStart.value);
        kStart.onTrue(mToggleAllianceColorCommand);

        JoystickButton buttonX = new JoystickButton(mXboxController, XboxController.Button.kX.value);
        buttonX.onTrue(mToggleAlliancePositionCommand);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.onTrue(mToggleTemporaryTeamPropPositionCommand);

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

        /*if (mConfigSubsystem.allianceColor() == Constants.ConfigConstants.kBleu) {
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kGauche) {
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropGauche) {
                    return new BleuGaucheTeamPropGauche(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new BleuGaucheTeamPropMilieu(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropDroite) {
                    return new BleuGaucheTeamPropDroite(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kDroite) {
                if (mConfigSubsystem.alliancePosition() == Constants.VisionConstants.kTeamPropGauche) {
                    return new BleuDroiteTeamPropGauche(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mConfigSubsystem.alliancePosition() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new BleuDroiteTeamPropMilieu(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mConfigSubsystem.alliancePosition() == Constants.VisionConstants.kTeamPropDroite) {
                    return new BleuDroiteTeamPropDroite(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
        }
        if (mConfigSubsystem.allianceColor() == Constants.ConfigConstants.kRouge) {
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kGauche) {
                if (mConfigSubsystem.alliancePosition() == Constants.VisionConstants.kTeamPropGauche) {
                    return new RougeGaucheTeamPropGauche(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mConfigSubsystem.alliancePosition() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new RougeGaucheTeamPropMilieu(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mConfigSubsystem.alliancePosition() == Constants.VisionConstants.kTeamPropDroite) {
                    return new RougeGaucheTeamPropDroite(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kDroite) {
                if (mConfigSubsystem.alliancePosition() == Constants.VisionConstants.kTeamPropGauche) {
                    return new RougeDroiteTeamPropGauche(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mConfigSubsystem.alliancePosition() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new RougeDroiteTeamPropMilieu(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mConfigSubsystem.alliancePosition() == Constants.VisionConstants.kTeamPropDroite) {
                    return new RougeDroiteTeamPropDroite(mDriveSubsystem, mIntakeSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
        }*/
        return new AvanceAutoCommand(mDriveSubsystem, 1, 16);
    }
}

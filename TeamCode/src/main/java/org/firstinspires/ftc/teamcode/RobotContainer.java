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
import org.firstinspires.ftc.teamcode.commands.ActivateAprilTagPipelineCommand;
import org.firstinspires.ftc.teamcode.commands.ActivatePropPipelineCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.CalibreExtentionCommand;
import org.firstinspires.ftc.teamcode.commands.DeactivateAprilTagPipelineCommand;
import org.firstinspires.ftc.teamcode.commands.DeactivatePropPipelineCommand;
import org.firstinspires.ftc.teamcode.commands.LanceurCommand;
import org.firstinspires.ftc.teamcode.commands.PinceCommandToggleDroit;
import org.firstinspires.ftc.teamcode.commands.PinceCommandToggleGauche;
import org.firstinspires.ftc.teamcode.commands.PinceToggleInclinaisonCommand;
import org.firstinspires.ftc.teamcode.commands.ToggleAllianceColorCommand;
import org.firstinspires.ftc.teamcode.commands.ToggleAlliancePositionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.PinceCommandToggle;
import org.firstinspires.ftc.teamcode.commands.ToggleTemporaryTeamPropPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ToggleVisionPipelineCommand;
import org.firstinspires.ftc.teamcode.subsystems.ConfigSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.BrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LanceurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
import org.firstinspires.ftc.teamcode.commands.CalibreBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.TeamPropPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController mXboxController = new XboxController(Constants.OIConstants.kDriverControllerPort);
    private final XboxController mXboxController2 = new XboxController(Constants.OIConstants.kDriverControllerPort2);

    private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();
    private final ConfigSubsystem mConfigSubsystem = new ConfigSubsystem();
    private final VisionSubsystem mVisionSubsystem = new VisionSubsystem();
    public final TeamPropPipeline mTeamPropPipeline = new TeamPropPipeline();

    private final  PinceSubsystem mPinceSubsystem = new PinceSubsystem();
    private final DriveCommand mDriveCommand = new DriveCommand(mDriveSubsystem, mXboxController);


    private final BrasSubsystem mBrasSubsystem = new BrasSubsystem();

    private final BrasCommandPos1 mBrasCommandPos1 = new BrasCommandPos1(mBrasSubsystem, 5000);

    private final BrasCommandPos1 mBrasCommandPos2 = new BrasCommandPos1(mBrasSubsystem, 330);

   private final BrasCommand mBrasCommand = new BrasCommand(mBrasSubsystem, mXboxController2);

    private final ToggleVisionPipelineCommand mToggleVisionPipelineCommand = new ToggleVisionPipelineCommand(mVisionSubsystem);

    private final ActivatePropPipelineCommand mActivatePropPipelineCommand = new ActivatePropPipelineCommand(mVisionSubsystem);
    private final ActivateAprilTagPipelineCommand mActivateAprilTagPipelineCommand = new ActivateAprilTagPipelineCommand(mVisionSubsystem);

    private final DeactivatePropPipelineCommand mDeactivatePropPipelineCommand = new DeactivatePropPipelineCommand(mVisionSubsystem);

    private final DeactivateAprilTagPipelineCommand mDeactivateAprilTagPipelineCommand = new DeactivateAprilTagPipelineCommand(mVisionSubsystem);

    private final PinceCommandToggle mPinceCommandToggle = new PinceCommandToggle(mPinceSubsystem);

    private final LanceurSubsystem mLanceurSubsystem = new LanceurSubsystem();

    private final LanceurCommand mlanceurCommand = new LanceurCommand(mLanceurSubsystem);

    private final CalibreBrasCommand mCalibreBrasCommand = new CalibreBrasCommand(mBrasSubsystem);

    private final ToggleAllianceColorCommand mToggleAllianceColorCommand = new ToggleAllianceColorCommand(mConfigSubsystem);
    private final ToggleAlliancePositionCommand mToggleAlliancePositionCommand = new ToggleAlliancePositionCommand(mConfigSubsystem);
    private final ToggleTemporaryTeamPropPositionCommand mToggleTemporaryTeamPropPositionCommand = new ToggleTemporaryTeamPropPositionCommand(mConfigSubsystem);

    private final PinceCommandToggleDroit mPinceCommandToggleDroit = new PinceCommandToggleDroit(mPinceSubsystem);
    private final PinceCommandToggleGauche mPinceCommandToggleGauche = new PinceCommandToggleGauche(mPinceSubsystem);

    private final PinceToggleInclinaisonCommand mPinceToggleInclinaisonCommand = new PinceToggleInclinaisonCommand(mPinceSubsystem);

    private final CalibreExtentionCommand mCalibreExtentionCommand = new CalibreExtentionCommand(mBrasSubsystem);


    public RobotContainer() {
        configureButtonBindings();
        configureDefaultCommands();
    }

    public void init() {
        //mCalibreBrasCommand.schedule();
    }

    private void configureButtonBindings() {

        JoystickButton kUp = new JoystickButton(mXboxController, XboxController.Button.kUp.value);
        kUp.onTrue(mActivatePropPipelineCommand);

        JoystickButton kRight = new JoystickButton(mXboxController, XboxController.Button.kRight.value);
        kRight.onTrue(mActivateAprilTagPipelineCommand);

        JoystickButton kDown = new JoystickButton(mXboxController, XboxController.Button.kDown.value);
        kDown.onTrue(mDeactivateAprilTagPipelineCommand);

        JoystickButton kLeft = new JoystickButton(mXboxController, XboxController.Button.kLeft.value);
        kLeft.onTrue(mDeactivatePropPipelineCommand);
        ///////////////////////////////////////////////////////////////////
        JoystickButton kStart = new JoystickButton(mXboxController, XboxController.Button.kStart.value);
        kStart.onTrue(mToggleAllianceColorCommand);

        JoystickButton kBack = new JoystickButton(mXboxController, XboxController.Button.kBack.value);
        kBack.onTrue(mToggleAlliancePositionCommand);

        JoystickButton buttonY = new JoystickButton(mXboxController, XboxController.Button.kY.value);
        buttonY.onTrue(mToggleVisionPipelineCommand);

        JoystickButton buttonB = new JoystickButton(mXboxController, XboxController.Button.kB.value);
        buttonB.onTrue(mCalibreBrasCommand);



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        JoystickButton buttonX2 = new JoystickButton(mXboxController2, XboxController.Button.kX.value);
        buttonX2.whileTrue(mBrasCommandPos1);

        JoystickButton buttonB2 = new JoystickButton(mXboxController2, XboxController.Button.kB.value);
        buttonB2.onTrue(mPinceCommandToggle);

        JoystickButton buttonY2 = new JoystickButton(mXboxController2, XboxController.Button.kY.value);
        //buttonY2.whileTrue(mBrasCommandPos2);
        buttonY2.onTrue(mCalibreExtentionCommand);

        JoystickButton buttonA2 = new JoystickButton(mXboxController2, XboxController.Button.kA.value);
        buttonA2.onTrue(mPinceToggleInclinaisonCommand);

        JoystickButton buttonkLeft = new JoystickButton(mXboxController2, XboxController.Button.kLeft.value);
        buttonkLeft.onTrue(mPinceCommandToggleDroit);

        JoystickButton buttonkRight = new JoystickButton(mXboxController2, XboxController.Button.kRight.value);
        buttonkRight.onTrue(mPinceCommandToggleGauche);
    }
    private void configureDefaultCommands() {
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
        mBrasSubsystem.setDefaultCommand(mBrasCommand);
    }

    public Command getAutonomousCommand() {

        DriverStationJNI.getTelemetry().addData("couleur", Constants.ConfigConstants.kRouge);
        DriverStationJNI.getTelemetry().addData("position", Constants.ConfigConstants.kGauche);
        DriverStationJNI.getTelemetry().addData("TeamPropLocation", Constants.VisionConstants.kTeamPropMilieu);

        return new RougeGaucheTeamPropGauche(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
        /*
        if (mConfigSubsystem.allianceColor() == Constants.ConfigConstants.kBleu) {
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kGauche) {
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropGauche) {
                    return new BleuGaucheTeamPropGauche(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new BleuGaucheTeamPropMilieu(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropDroite) {
                    return new BleuGaucheTeamPropDroite(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kDroite) {
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropGauche) {
                    return new BleuDroiteTeamPropGauche(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new BleuDroiteTeamPropMilieu(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropDroite) {
                    return new BleuDroiteTeamPropDroite(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
        }
        if (mConfigSubsystem.allianceColor() == Constants.ConfigConstants.kRouge) {
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kGauche) {
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropGauche) {
                    return new RougeGaucheTeamPropGauche(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new RougeGaucheTeamPropMilieu(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropDroite) {
                    return new RougeGaucheTeamPropDroite(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
            if (mConfigSubsystem.alliancePosition() == Constants.ConfigConstants.kDroite) {
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropGauche) {
                    return new RougeDroiteTeamPropGauche(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropMilieu) {
                    return new RougeDroiteTeamPropMilieu(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
                if (mVisionSubsystem.getTeamPropLocation() == Constants.VisionConstants.kTeamPropDroite) {
                    return new RougeDroiteTeamPropDroite(mDriveSubsystem, mPinceSubsystem, mBrasSubsystem);
                }
            }
        }
        return null;*/
    }
}


package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ConfigSubsystem extends Subsystem {

    private int mAllianceColor = Constants.ConfigConstants.kRouge;
    private int mAlliancePosition = Constants.ConfigConstants.kGauche;

    // A supprimer quand on a la camera fonctionnelle
    private int mTemporaryTeamPropLocation = Constants.VisionConstants.kTeamPropMilieu;

    public ConfigSubsystem() {
    }

    @Override
    public void periodic() {

        DriverStationJNI.getTelemetry().addData("Alliance Color", allianceColorToString());
        DriverStationJNI.getTelemetry().addData("Alliance Position", alliancePositionToString());

    }
public void toggleAllianceColor() {
    if (++mAllianceColor > 1) mAllianceColor = 0;
}

public void toggleAlliancePosition() {
    if (++mAlliancePosition > 1) mAlliancePosition = 0;
}

// A supprimer quand on a la camera fonctionnelle
public void toggleTemporaryTeamPropPosition() {
    if (++mTemporaryTeamPropLocation > 2) mTemporaryTeamPropLocation = 0;
}

public int  allianceColor() {
        return mAllianceColor;
}
private String allianceColorToString() {
    return mAllianceColor == Constants.ConfigConstants.kRouge ? "Rouge" : "Bleu";
}

public int alliancePosition() {
        return mAlliancePosition;
}
private String alliancePositionToString() {
    return mAlliancePosition == Constants.ConfigConstants.kGauche ? "Gauche" : "Droite";
}
public int temporaryTeamPropLocation() {
    return mTemporaryTeamPropLocation;
}

}




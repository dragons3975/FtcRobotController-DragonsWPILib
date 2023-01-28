package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ConfigSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private boolean mPositionDepartGauche = false;

    public ConfigSubsystem(Telemetry telemetry) {
        mTelemetry = telemetry;
    }

    @Override
    public void periodic() {
        mTelemetry.addData("Position de départ", mPositionDepartGauche ? "gauche" : "droite");
    }

    public void togglePositionDepart() {
        mPositionDepartGauche = !mPositionDepartGauche;
    }

    public boolean isPositionDepartGauche() {
        return mPositionDepartGauche;
    }

}

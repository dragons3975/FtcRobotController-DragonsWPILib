package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class PinceSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;

    private final Servo mMoteurPince;

    public PinceSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mMoteurPince = mHardwareMap.get(Servo.class, "pince");
    }

    @Override
    public void periodic() {
        mTelemetry.addData("Pince position", mMoteurPince.getPosition());
    }

    public void ouvrir() {
        mMoteurPince.setPosition(Constants.PinceConstants.kOuvrirPincePosition);
    }

    public void fermer() {
        mMoteurPince.setPosition(Constants.PinceConstants.kFermerPincePosition);
    }

}

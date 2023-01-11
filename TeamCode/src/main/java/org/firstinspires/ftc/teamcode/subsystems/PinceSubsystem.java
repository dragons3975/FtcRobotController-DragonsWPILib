package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.motors.Matrix12vMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.math.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class PinceSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;
    private Gamepad mGamepad2;

    private final Servo mMoteurPince;

    private double position = 0;

    public PinceSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad2) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
        mGamepad2 = gamepad2;

        mMoteurPince = mHardwareMap.get(Servo.class, "pince");
    }

    @Override
    public void periodic() {
        mTelemetry.addData("Pince position", mMoteurPince.getPosition());
    }

    public void setPosition(double consigne) {
        mMoteurPince.setPosition(consigne);
    }

    public void ouvrir() {
        mMoteurPince.setPosition(0.27);
    }
    public void fermer() {
        mMoteurPince.setPosition(0);
    }

}


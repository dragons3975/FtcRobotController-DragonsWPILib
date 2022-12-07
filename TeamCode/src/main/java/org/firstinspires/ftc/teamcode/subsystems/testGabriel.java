package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class testGabriel extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;

    private final DcMotorSimple mMoteurG;

    public testGabriel(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mMoteurG = mHardwareMap.get(DcMotorSimple.class, "motorG");
    }

    @Override
    public void periodic() {

    }

    public void stop() {
        mMoteurG.setPower(0);
    }

    public void yann(double trigger) {
        mMoteurG.setPower(trigger);
    }
}


package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ConfigSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;
    private DriveSubsystem mDriveSubsystem;




    private double mX = 0;
    private double mY = 0;
    private double mZ = 0;

    // Pour suivre la position sur le terrain. Donnée par Vuforia.
    private double mPositionX = 0;
    private double mPositionY = 0;
    private double mRotationZ = 0;

    public ConfigSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;


    }


    @Override
    public void periodic() {

    }


    public void stop() {
        mDriveSubsystem.stop();
    }

}


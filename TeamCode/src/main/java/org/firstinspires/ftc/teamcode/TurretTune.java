package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp
public class TurretTune extends OpMode {
    private PIDController controller;

    public static double p=0, i=0, d=0;
    private DcMotorEx turret;
    private Limelight3A limelight;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class,"outakeR");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight.start();
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);

    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid() & !(limelight.getLatestResult() == null) & gamepad1.a) {

            double turretPosFromTargetInTicks = result.getTx();
            double pidPower = controller.calculate(turretPosFromTargetInTicks, 0);

            turret.setPower(pidPower);
            telemetry.addData("pos", turret.getCurrentPosition());
            telemetry.addData("target", turretPosFromTargetInTicks);
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("power", pidPower);
        }
        else {
            turret.setPower(0);
        }

    }
}
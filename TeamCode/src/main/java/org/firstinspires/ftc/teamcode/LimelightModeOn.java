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
@TeleOp
public class LimelightModeOn extends OpMode {
    private PIDController controller;

    public double p = 0.00055, i = 0.00015, d = 0.0145;

    public int target;


    private DcMotorEx turret;
    private Limelight3A limelight;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class,"outakeR");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        turretPos = turret.getCurrentPosition();
        limelight.start();
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);

    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid() || limelight.getLatestResult() == null) {
        controller.setPID(p, i, d);
        int turretPos = turret.getCurrentPosition();
        target = (int) (turretPos - (result.getTx()*7.35));
        double pid = controller.calculate(turretPos, target);

        double power = pid;

        turret.setPower(power);
            telemetry.addData("pos", turretPos);
            telemetry.addData("target", target);
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("power", power);
        }

    }
}
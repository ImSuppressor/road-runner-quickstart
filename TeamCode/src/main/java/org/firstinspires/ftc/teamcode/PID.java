package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class PID extends OpMode {
    private PIDController controller;

    public static double p=0, i=0, d=0;

    public static int target= 0;

//    private final double ticks_in_degrees = 537.7/360*0.153846154;

    private DcMotorEx turret;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = hardwareMap.get(DcMotorEx.class,"outakeR");
//        p=0.00055;
//        i=0.00015;
//        d=0.0145;
    }

    @Override
    public void loop() {
//        p=0.00055;
//        i=0.00015;
//        d=0.0145;
        controller.setPID(p, i, d);
        int turretPos = turret.getCurrentPosition();
        double pid = controller.calculate(turretPos, target);

        double power = pid;

        turret.setPower(power);

        telemetry.addData("pos", turretPos);
        telemetry.addData("target", target);
        telemetry.update();

    }
}

package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath_2", group = "Autonomous")
public class Pedro_Auto_2 extends LinearOpMode {

    private Follower follower;
    double mspower;
    double lwpower;
    private DcMotor ms, mi, lw;

    private ElapsedTime timer = new ElapsedTime();

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(8.087, 131.8125, 0);
    private final Pose shootpos = poseFactory.of(80.2389, 106.3022, 90);
    private final Pose park = poseFactory.of(10.0507, 117.4774, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shootpos()),
                Shoot_routine(),
                follow(follower, park())
        );
    }

    public Command Shoot_routine() {
        return Command.build()
                .setExecute(() -> {
                    if (getRuntime() > 10) {
                        mspower = 0.50;
                    }
                    if (getRuntime() > 15) {
                        lwpower = 1;
                    }

                })
                .setDone(() -> getRuntime() > 20 )
                .setEnd(endCondition -> {
                    lwpower = 0;
                    mspower = 0;
                });
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        ms = hardwareMap.get(DcMotor.class, "ms");
        lw = hardwareMap.get(DcMotor.class, "lw");

        ms.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            ms.setPower(mspower);
            lw.setPower(lwpower);

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Time", getRuntime());
            telemetry.addData("Shooter Power", ms.getPower());
            telemetry.addData("LW Power", lw.getPower());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path shootpos() {
        return Paths.line(start, shootpos).linear(start, shootpos);
    }

    public Path park() {
        return Paths.line(shootpos, park).constant(park);
    }
}

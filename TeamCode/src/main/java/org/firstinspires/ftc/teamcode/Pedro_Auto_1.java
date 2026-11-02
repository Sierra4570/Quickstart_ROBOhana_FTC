package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath_1", group = "Autonomous")
public class Pedro_Auto_1 extends LinearOpMode {

    private Follower follower;
    double mspower;
    double lwpower;
    private DcMotor ms, mi, lw;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose hivestart = poseFactory.of(57.9193, 7.764, 270);
    private final Pose shootpos = poseFactory.of(57.7777, 35.4317, 270);
    private final Pose point2 = poseFactory.of(17.7254, 35.0056, -179.2831);
    private final Pose point3 = poseFactory.of(17.0632, 100.5913, 90);
    private final Pose point4 = poseFactory.of(-8.3846, 130.7771, 0);


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shootpos()),
                Shoot_routine(),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4())
        );
    }

    public Command Shoot_routine() {
        return Command.build()
                .setExecute(() -> {
                    if (getRuntime() > 5) {
                        mspower = 0.50;
                    }
                    if (getRuntime() > 10) {
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
        follower.setPose(hivestart);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        ms = hardwareMap.get(DcMotor.class, "ms");
        lw = hardwareMap.get(DcMotor.class, "lw");

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path shootpos() {
        return Paths.line(hivestart, shootpos).constant(shootpos);
    }

    public Path path2() {
        return Paths.line(shootpos, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }
}

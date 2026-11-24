package org.firstinspires.ftc.teamcode.Autonoms;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.Launch;
import org.firstinspires.ftc.teamcode.commands.Suck;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalisationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name = "AutoFirst")
public class AutoFirst extends OpMode {

    private Follower follower;
    private IntakeSubsystem intake;
    private TurretSubsystem turret;
    private LocalisationSubsystem localisation;

    private final ElapsedTime timer = new ElapsedTime();
    private int state = 0;
    private Command suck, launch;

    private static Pose p(double x, double y, double deg) {
        return new Pose(x, y, Math.toRadians(deg));
    }

    private final Pose start     = p(56, 8, 90);
    private final Pose garden    = p(22.7254, 7.8105, 180);
    private final Pose gardenC   = p(33.5996, 32.2592, 0);
    private final Pose gardenEnd = p(8.6996, 7.8032, 180);
    private final Pose shoot2    = p(39.8076, 22.9898, 90);
    private final Pose shoot2C   = p(54.5941, 10.7623, 0);
    private final Pose flower    = p(12.161, 48.452, 185.2482);   // corectat (era -174.7518)
    private final Pose flowerC   = p(39.9241, 51.0021, 0);
    private final Pose shoot3    = p(39.6403, 21.3301, 86.974);
    private final Pose park      = p(38, 33, 90);

    private Path toGarden, intakeGarden, toShoot2, toFlower, toShoot3, toPark;

    private void buildPaths() {
        toGarden = curve(start, gardenC, garden)
                .heading(Interpolator.linear(start.heading(), garden.heading()));
        intakeGarden = line(garden, gardenEnd)
                .constant(gardenEnd.heading());
        toShoot2 = curve(gardenEnd, shoot2C, shoot2)
                .heading(Interpolator.linear(gardenEnd.heading(), shoot2.heading()));
        toFlower = curve(shoot2, flowerC, flower)
                .heading(Interpolator.linear(shoot2.heading(), flower.heading()));
        toShoot3 = line(flower, shoot3)
                .heading(Interpolator.linear(flower.heading(), shoot3.heading()));
        toPark = line(shoot3, park)
                .heading(Interpolator.linear(shoot3.heading(), park.heading()));
    }

    private void startSuck() {
        suck = new Suck(intake);
        CommandScheduler.getInstance().schedule(suck);
    }
    private void stopSuck() {
        if (suck != null) CommandScheduler.getInstance().cancel(suck);
    }
    private void startLaunch() {
        launch = new Launch(hardwareMap, turret, intake, localisation).withTimeout(3000);
        CommandScheduler.getInstance().schedule(launch);
    }
    private boolean launchDone() {
        return launch == null || !launch.isScheduled();
    }

    private void next(int s) { state = s; timer.reset(); }

    private void updateAuto() {
        switch (state) {
            case 0: startLaunch(); next(1); break;                       // preload
            case 1:
                if (launchDone()) {
                    startSuck();
                    if(timer.seconds()>2){
                    follower.follow(toGarden);
                    next(2);}
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.follow(intakeGarden);
                    next(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    next(4); // lasam intake-ul sa mai ruleze putin dupa gardenEnd
                }
                break;
            case 4:
                if (timer.seconds() > 0.1) {
                    stopSuck();
                    next(5);
                }
                break;
            case 5:                                                      // asteptam ai nostri
                if (timer.seconds() > 7.0) {
                    follower.follow(toShoot2);
                    next(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) { startLaunch(); next(7); }       // lansam garden
                break;
            case 7:
                if (launchDone()) {
                    startSuck();
                    if(timer.seconds() > 3){///aici este inainte de floare
                    follower.follow(toFlower);
                    next(8);}
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.follow(toShoot3);
                    next(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) { stopSuck(); startLaunch(); next(10); } // lansam floarea
                break;
            case 10:
                if (launchDone()) {
                    follower.follow(toPark);
                    next(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) next(-1);
                break;
        }
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        localisation = new LocalisationSubsystem(hardwareMap, follower);
        intake = new IntakeSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        buildPaths();
    }

    @Override
    public void start() { next(0); }

    @Override
    public void loop() {
        follower.update();
        CommandScheduler.getInstance().run();
        updateAuto();

        Pose pose = follower.pose();
        telemetry.addData("state", state);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("launchDone", launchDone());
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading", Math.toDegrees(pose.heading()));
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.Launch;
import org.firstinspires.ftc.teamcode.commands.Suck;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name = "autosolored")
public class autosolored extends LinearOpMode {

    private Follower follower;
    private IntakeSubsystem intake;
    private TurretSubsystem turret;
    private CameraSubsystem vedere;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Punctele si orientarile exportate de tine pentru RED.
    private final Pose startPos = poseFactory.of(55, 8.42, 90);
    private final Pose point1 = poseFactory.of(55, 8.42, 180);
    private final Pose redSolo = poseFactory.of(8.3512, 8.0813, 180);
    private final Pose point3Start = poseFactory.of(8.3512, 8.0813, 180);
    private final Pose point3 = poseFactory.of(26.4218, 101.1025, 90);
    private final Pose point3Control1 = poseFactory.of(36.9769, 89.0327, 0);
    private final Pose point4Start = poseFactory.of(26.4218, 101.1025, 0);
    private final Pose point4 = poseFactory.of(36.1594, 116.0615, -60);
    private final Pose point5 = poseFactory.of(46.461, 130.2562, 90);
    private final Pose point6Start = poseFactory.of(46.461, 130.2562, 90);
    private final Pose point6 = poseFactory.of(57.7184, 115.0765, 270);
    private final Pose point7Start = poseFactory.of(57.7184, 115.0765, 180);
    private final Pose point7 = poseFactory.of(11.5758, 46.3218, 180);
    private final Pose point7Control1 = poseFactory.of(13.5173, 83.8764, 0);
    private final Pose point8Start = poseFactory.of(11.5758, 46.3218, 180);
    private final Pose point8 = poseFactory.of(17.438, 13.8251, 50);
    private final Pose camera = poseFactory.of(57.72, 115.08, 0);

    // path1 avea aceleasi X/Y: il executam ca rotire pe loc in strategie.

    public Path redSolo() {
        return Paths.line(point1, redSolo).constant(redSolo);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3)
                .linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).constant(point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.curve(point7Start, point7Control1, point7)
                .linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path camera() {
        return Paths.line(point8, camera).constant(camera);
    }

    private void updateCamera() {
        vedere.update();
        telemetry.addData("Vede bile", vedere.VedeBile());
        telemetry.addData("Directie", vedere.Directie());
        telemetry.addData("Bile robot", vedere.BileRobot());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    // Executa un traseu complet, inclusiv curbele exportate.
    // null = fara comanda suplimentara; new Suck(intake) = intake pe drum.
    private void urmeaza(Path path, Command whileDriving) {
        if (!opModeIsActive()) return;

        follower.follow(path);
        if (whileDriving != null) {
            CommandScheduler.getInstance().schedule(whileDriving);
        }

        try {
            do {
                if (!opModeIsActive()) break;
                updateCamera();
                follower.update();
                CommandScheduler.getInstance().run();
                idle();
            } while (opModeIsActive() && follower.isBusy());
        } finally {
            if (whileDriving != null) {
                CommandScheduler.getInstance().cancel(whileDriving);
            }
        }
    }

    // Optional: dreapta de la pozitia actuala la un punct ales de tine.
    private void driveTo(Pose target, Command whileDriving) {
        if (!opModeIsActive()) return;
        Pose from = follower.pose();
        urmeaza(Paths.line(from, target).linear(from, target), whileDriving);
    }

    // Orientare absoluta pe teren, in grade. Nu este un numar de grade relative.
    private void intoarceLa(double grade) {
        if (!opModeIsActive()) return;

        Pose acum = follower.pose();
        Pose tinta = poseFactory.of(acum.x(), acum.y(), grade);
        follower.hold(tinta);

        while (opModeIsActive()) {
            updateCamera();
            follower.update();
            CommandScheduler.getInstance().run();

            double diferenta = tinta.heading() - follower.pose().heading();
            double eroare = Math.atan2(Math.sin(diferenta), Math.cos(diferenta));
            if (Math.abs(eroare) <= Math.toRadians(2)) break;
            idle();
        }
    }

    // Optional: apelezi tu aceasta metoda unde vrei colectare dupa camera.
    // Presupune Directie() normalizata si contor corect pentru bilele colectate.
    private void cautaSiColecteaza() {
        if (!opModeIsActive()) return;

        follower.manual(0, 0, 0);
        follower.update();
        Command suck = new Suck(intake);
        CommandScheduler.getInstance().schedule(suck);

        try {
            while (opModeIsActive()) {
                updateCamera();
                if (vedere.BileRobot() >= 4) break;

                double y = 0;
                double x = 0;
                double rx = 0;

                if (vedere.VedeBile()) {
                    double directie = vedere.Directie();
                    if (Double.isFinite(directie)) {
                        x = Math.max(-0.25, Math.min(0.25, directie * 0.25));
                        if (Math.abs(directie) < 0.2) y = 0.2;
                    }
                } else {
                    rx = 0.15;
                }

                follower.manual(y, x, rx);
                follower.update();
                CommandScheduler.getInstance().run();
                idle();
            }
        } finally {
            CommandScheduler.getInstance().cancel(suck);
            intake.stop();
            follower.manual(0, 0, 0);
            follower.update();
        }
    }

    // Optional: Launch trebuie sa se termine singura prin conditia ei isFinished().
    private void shoot() {
        if (!opModeIsActive()) return;

        Command launch = new Launch(turret, intake);
        CommandScheduler.getInstance().schedule(launch);
        try {
            while (opModeIsActive() && launch.isScheduled()) {
                updateCamera();
                follower.update();
                CommandScheduler.getInstance().run();
                idle();
            }
        } finally {
            CommandScheduler.getInstance().cancel(launch);
        }

        if (opModeIsActive()) {
            vedere.reseteazaBileRobot();
        }
    }

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPos);
        intake = new IntakeSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        vedere = new CameraSubsystem(hardwareMap);

        vedere.alegcebilevreau(0); // Schimba tu pipeline-ul daca este necesar.
        vedere.start();

        telemetry.addLine("autosolored pregatit.");
        telemetry.update();

        try {
            waitForStart();
            if (isStopRequested()) return;

            // ===== STRATEGIA TA: ADAUGI ACTIUNILE INTRE ACESTE LINII =====

            // Poti adauga shoot(); aici, inaintea rotirii initiale.
            intoarceLa(180); // Inlocuieste path1: aceeasi pozitie, alt unghi.

            // Pentru intake pe acest traseu:
            // urmeaza(redSolo(), new Suck(intake));
            urmeaza(redSolo(), null);

            urmeaza(path3(), null);

            // path3 se termina la 90; path4 incepe la 0.
            intoarceLa(0);
            urmeaza(path4(), null);

            // path4 se termina la -60; path5 cere orientare constanta 90.
            intoarceLa(90);
            urmeaza(path5(), null);
            urmeaza(path6(), null);

            // path6 se termina la 270; path7 incepe la 180.
            intoarceLa(180);
            urmeaza(path7(), null);
            urmeaza(path8(), null);

            // path8 se termina la 50; camera() cere orientare constanta 0.
            intoarceLa(0);
            urmeaza(camera(), null);

            // Optional, scrii tu unde doresti:
            // cautaSiColecteaza();
            // shoot();
            // intoarceLa(180);
            // driveTo(point8, null);

            // ===== SFARSITUL STRATEGIEI =====
        } finally {
            CommandScheduler.getInstance().reset();
            intake.stop();
            follower.manual(0, 0, 0);
            follower.update();
        }
    }
}

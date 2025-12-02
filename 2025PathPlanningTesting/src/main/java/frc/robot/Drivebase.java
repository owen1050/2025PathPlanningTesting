package frc.robot;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drivebase {

    public SwerveDrive swerveDrive;
    public PathPlannerPath path;
    public PathPlannerTrajectory traj;
    public PathConstraints pathConstraints = new PathConstraints(3, 2, 6, 3);

    public double pathStartTime = 0;

    public PIDController vXPidController = new PIDController(10, 0, 0);
    public PIDController vYPidController = new PIDController(10, 0, 0);
    public PIDController vRPidController = new PIDController(0.8, 0, 0);

    public Drivebase() {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(3);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void driveWithVelocity(double vx, double vy, double vr, boolean fr) {
        swerveDrive.drive(new Translation2d(vx, vy), vr, fr, false);
    }

    public void pathToTraj() {

        try {
            path = PathPlannerPath.fromChoreoTrajectory("ExampleChoreoTrajectory");
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }

        try {
            if (path != null) {
                traj = path.generateTrajectory(swerveDrive.getRobotVelocity(), swerveDrive.getOdometryHeading(),
                        RobotConfig.fromGUISettings());
                pathStartTime = Timer.getFPGATimestamp();

            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }

    }

    public void followPath() {

        double dt = Timer.getFPGATimestamp() - pathStartTime;
        PathPlannerTrajectoryState goalState = traj.sample(dt);

        double vXFB = vXPidController.calculate(swerveDrive.field.getRobotPose().getX(), goalState.pose.getX());
        double vYFB = vYPidController.calculate(swerveDrive.field.getRobotPose().getY(), goalState.pose.getY());
        double vRFB = vRPidController
                .calculate(swerveDrive.getOdometryHeading().minus(goalState.pose.getRotation()).getDegrees(), 0);

        driveWithVelocity(goalState.fieldSpeeds.vxMetersPerSecond + vXFB,
                goalState.fieldSpeeds.vxMetersPerSecond + vYFB, goalState.fieldSpeeds.omegaRadiansPerSecond + vRFB,
                true);

    }
}

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drivebase {

    public SwerveDrive swerveDrive;
    public PathPlannerPath path;
    public PathPlannerTrajectory traj;
    public PathConstraints pathConstraints = new PathConstraints(3, 2, 6, 3);

    public Field2d otfGoalField2d = new Field2d();
    public boolean pathValid = false;
    public double pathStartTime = 0;

    public PIDController vXPidController = new PIDController(10, 0, 0);
    public PIDController vYPidController = new PIDController(10, 0, 0);
    public PIDController vRPidController = new PIDController(0.8, 0, 0);

    public Translation2d goalTranslation2d = new Translation2d(5.51, 5.65);

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

    public void initPath() {
        Pathfinding.ensureInitialized();
        Pathfinding.setStartPosition(swerveDrive.field.getRobotPose().getTranslation());
        Pathfinding.setGoalPosition(goalTranslation2d);
    }

    public void pathToTraj() {
        if (Pathfinding.isNewPathAvailable()) {
            path = Pathfinding.getCurrentPath(pathConstraints, new GoalEndState(0, new Rotation2d(-Math.PI*2/3)));

            try {
                if (path != null) {
                    replaceFirstWaypointInPath();
                    traj = path.generateTrajectory(swerveDrive.getRobotVelocity(), swerveDrive.getOdometryHeading(),
                            RobotConfig.fromGUISettings());
                    otfGoalField2d.getObject("traj").setTrajectory(ppTrajToWPITraj(traj));
                    SmartDashboard.putData("Traj", otfGoalField2d);
                    //printPath();
                    pathValid = true;
                    pathStartTime = Timer.getFPGATimestamp();

                }
            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }
        }
    }

    public Trajectory ppTrajToWPITraj(PathPlannerTrajectory traj) {

        List<PathPlannerTrajectoryState> stateList = traj.getStates();
        List<Trajectory.State> wpiStateLists = new ArrayList<Trajectory.State>();

        for (PathPlannerTrajectoryState state : stateList) {
            Trajectory.State thisWPIState = new Trajectory.State(state.timeSeconds,
                    state.linearVelocity,
                    0,
                    state.pose,
                    0);
            wpiStateLists.add(thisWPIState);
        }
        return new Trajectory(wpiStateLists);
    }

    public void followPath() {
        
        if (pathValid) {
            PathPlannerTrajectoryState firstState = traj.sample(0);
            System.out.println(firstState.feedforwards);
            double dt = Timer.getFPGATimestamp() - pathStartTime;
            PathPlannerTrajectoryState goalState = traj.sample(dt);
            otfGoalField2d.setRobotPose(goalState.pose);
            double vXFB = vXPidController.calculate(swerveDrive.field.getRobotPose().getX(), goalState.pose.getX());
            double vYFB = vYPidController.calculate(swerveDrive.field.getRobotPose().getY(), goalState.pose.getY());
            double vRFB = vRPidController
                    .calculate(swerveDrive.getOdometryHeading().minus(goalState.pose.getRotation()).getDegrees(), 0);

            driveWithVelocity(goalState.fieldSpeeds.vxMetersPerSecond + vXFB,
                    goalState.fieldSpeeds.vxMetersPerSecond + vYFB, goalState.fieldSpeeds.omegaRadiansPerSecond + vRFB,
                    true);
        } else {
            //driveWithVelocity(0, 0, 0, true);
        }

    }

    public void printPath() {
        List<Waypoint> points = path.getWaypoints();
        System.out.println("_______________________________________");
        for (int i = 0; i < points.size(); i++) {
            Waypoint thisPoint = points.get(i);
            System.out.print(thisPoint.anchor() + ":");
            System.out.print(thisPoint.prevControl() + ":");
            System.out.print(thisPoint.nextControl() + ":");
            System.out.println();
        }
        System.out.println("_______________________________________");
    }

    public void replaceFirstWaypointInPath() {
        List<Waypoint> points = path.getWaypoints();
        Translation2d currentPos = swerveDrive.field.getRobotPose().getTranslation();
        double factor = 0.5;
        double dx = swerveDrive.getFieldVelocity().vxMetersPerSecond * factor;
        double dy = swerveDrive.getFieldVelocity().vyMetersPerSecond * factor;
        Waypoint newWaypoint = new Waypoint(null, points.get(0).anchor(),
                new Translation2d(currentPos.getX() + dx, currentPos.getY() + dy));
        points.set(0, newWaypoint);
        double robotV = Math.sqrt(swerveDrive.getFieldVelocity().vxMetersPerSecond
                * swerveDrive.getFieldVelocity().vxMetersPerSecond
                + swerveDrive.getFieldVelocity().vyMetersPerSecond * swerveDrive.getFieldVelocity().vyMetersPerSecond);
        path = new PathPlannerPath(points, pathConstraints,
                new IdealStartingState(robotV, swerveDrive.getOdometryHeading()),
                new GoalEndState(0,  new Rotation2d(-Math.PI*2/3)));
    }
}

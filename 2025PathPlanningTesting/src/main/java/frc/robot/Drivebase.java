package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
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

    public PIDController vXPidController = new PIDController(12.5, 0, 0);
    public PIDController vYPidController = new PIDController(12.5, 0, 0);
    public PIDController vRPidController = new PIDController(1, 0, 0);

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
        Pathfinding.setGoalPosition(new Translation2d(1.5, 1.5));
    }

    public void pathToTraj() {
        if (Pathfinding.isNewPathAvailable()) {
            path = Pathfinding.getCurrentPath(pathConstraints, new GoalEndState(0, new Rotation2d()));
            try {
                if (path != null) {
                    traj = path.generateTrajectory(swerveDrive.getRobotVelocity(), swerveDrive.getOdometryHeading(),
                            RobotConfig.fromGUISettings());
                    otfGoalField2d.getObject("traj").setTrajectory(ppTrajToWPITraj(traj));
                    SmartDashboard.putData("Traj", otfGoalField2d);
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

    public void followPath(){
        if(pathValid){
            double dt = Timer.getFPGATimestamp() - pathStartTime;
            PathPlannerTrajectoryState goalState = traj.sample(dt);
            otfGoalField2d.setRobotPose(goalState.pose);
            double vXFB = vXPidController.calculate(swerveDrive.field.getRobotPose().getX(), goalState.pose.getX());
            double vYFB = vYPidController.calculate(swerveDrive.field.getRobotPose().getY(), goalState.pose.getY());
            double vRFB = vRPidController.calculate(swerveDrive.getOdometryHeading().minus(goalState.pose.getRotation()).getDegrees(), 0);

            driveWithVelocity(goalState.fieldSpeeds.vxMetersPerSecond + vXFB, goalState.fieldSpeeds.vxMetersPerSecond+ vYFB, goalState.fieldSpeeds.omegaRadiansPerSecond+ vRFB, true);
        } else {
            driveWithVelocity(0, 0, 0, true);
        }

    }
}

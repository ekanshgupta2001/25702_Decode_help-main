package org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class farTumPath {
    public Follower follower;

    public Pose start = new Pose(56, 8, Math.toRadians(288));
    public Pose scoreFirst = new Pose(56, 15, Math.toRadians(288));

    public Pose setFirstPick = new Pose(51.62162162162162, 24.783783783783782, Math.toRadians(360));
    public Pose firstPick = new Pose(32.027027027027046, 24.783783783783782, Math.toRadians(360));

    public Pose scoreSecond = new Pose(56, 15, Math.toRadians(288));

    public Pose setSecondPick = new Pose(9, 13, Math.toRadians(0));
    public Pose secondPick = new Pose(9, 6, Math.toRadians(0));

    public Pose thirdScore = new Pose(56, 15, Math.toRadians(288));
    public Pose park = new Pose(38, 12, Math.toRadians(270));

    private int index;

    private static final int PATH_COUNT = 8; // cases 0..7

    public farTumPath(Follower follower, Alliance alliance) {
        this.follower = follower;

        if (alliance == Alliance.Red) {
            start = start.mirror();
            scoreFirst = scoreFirst.mirror();
            setFirstPick = setFirstPick.mirror();
            firstPick = firstPick.mirror();
            scoreSecond = scoreSecond.mirror();
            setSecondPick = setSecondPick.mirror();
            secondPick = secondPick.mirror();
            thirdScore = thirdScore.mirror();
            park = park.mirror();
        }

        reset();
    }

    public PathChain scoreP() {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, scoreFirst))
                .setReversed()
                .setLinearHeadingInterpolation(start.getHeading(), scoreFirst.getHeading())
                .build();
    }
    public PathChain goToPickOne() {
        return follower.pathBuilder()
            .addPath(new BezierLine(start, setFirstPick))
            .setReversed()
            .setLinearHeadingInterpolation(start.getHeading(), setFirstPick.getHeading())
            .build();
    }
    public PathChain pickOne() {
        return follower.pathBuilder()
                .addPath(new BezierLine(setFirstPick, firstPick))
                .setReversed()
                .setBrakingStrength(.75)
                .setLinearHeadingInterpolation(setFirstPick.getHeading(), firstPick.getHeading())
                .build();
    }

    public PathChain scoreTwo() {
        return follower.pathBuilder()
                .addPath(new BezierLine(firstPick, scoreSecond))
                .setReversed()
                .setLinearHeadingInterpolation(firstPick.getHeading(), scoreSecond.getHeading())
                .build();
    }

    public PathChain pickTwo() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(scoreSecond, setSecondPick, secondPick))
                .setReversed()
                .setBrakingStrength(.75)
                .setLinearHeadingInterpolation(scoreSecond.getHeading(), secondPick.getHeading())
                .build();
    }

    public PathChain scoreThird() {
        return follower.pathBuilder()
                .addPath(new BezierLine(secondPick, thirdScore))
                .setReversed()
                .setLinearHeadingInterpolation(secondPick.getHeading(), thirdScore.getHeading())
                .build();
    }

    public PathChain parkPath() {
        return follower.pathBuilder()
                .addPath(new BezierLine(thirdScore, park))
                .setReversed()
                .setLinearHeadingInterpolation(thirdScore.getHeading(), park.getHeading())
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return scoreP();
            case 1: return pickOne();
            case 2: return scoreTwo();
            case 3: return pickTwo();
            case 4: return scoreThird();
            case 5: return parkPath();
            default: return null;
        }
    }

    public boolean hasNext() {
        return index < PATH_COUNT;
    }

    public void reset() {
        index = 0;
    }
}

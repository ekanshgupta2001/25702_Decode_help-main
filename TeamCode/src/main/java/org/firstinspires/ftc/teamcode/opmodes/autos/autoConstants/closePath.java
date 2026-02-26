package org.firstinspires.ftc.teamcode.opmodes.autos.autoConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Alliance;

public class closePath {

    public Follower follower;

    public Pose start = new Pose(21.913, 123, Math.toRadians(316));
    public Pose scorefirst = new Pose(45, 84, Math.toRadians(316));

    public Pose setFirstPick = new Pose(45, 84, Math.toRadians(0));
    public Pose firstPick = new Pose(16.5, 84, Math.toRadians(0));

    public Pose scoreSecond = new Pose(45, 100, Math.toRadians(316));
    public Pose setSecondPick = new Pose(45, 60, Math.toRadians(0));
    public Pose secondPick = new Pose(10, 60, Math.toRadians(0));

    public Pose setThirdScore = new Pose(45, 60, Math.toRadians(316));
    public Pose thirdScore = new Pose(45, 100, Math.toRadians(316));

    public Pose setThirdPick = new Pose(45, 36, Math.toRadians(0));
    public Pose thirdPick = new Pose(10, 36, Math.toRadians(0));

    public Pose fourthScore = new Pose(45, 100, Math.toRadians(316));

    private int index;

    private static final int PATH_COUNT = 7;

    public closePath(Follower follower, Alliance alliance) {
        this.follower = follower;

        if (alliance == Alliance.Red) {
            start = start.mirror();
            scorefirst = scorefirst.mirror();

            setFirstPick = setFirstPick.mirror();
            firstPick = firstPick.mirror();

            scoreSecond = scoreSecond.mirror();
            setSecondPick = setSecondPick.mirror();
            secondPick = secondPick.mirror();

            setThirdScore = setThirdScore.mirror();
            thirdScore = thirdScore.mirror();

            setThirdPick = setThirdPick.mirror();
            thirdPick = thirdPick.mirror();

            fourthScore = fourthScore.mirror();
        }

        reset();
    }

    public PathChain scoreP() {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, scorefirst))
                .setReversed()
                .setLinearHeadingInterpolation(start.getHeading(), scorefirst.getHeading())
                .build();
    }

    public PathChain pickOne() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(scorefirst, setFirstPick, firstPick))
                .setReversed()
                .setBrakingStrength(.75)
                .setLinearHeadingInterpolation(scorefirst.getHeading(), firstPick.getHeading())
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
                .addPath(new BezierCurve(secondPick, setThirdScore, thirdScore))
                .setReversed()
                .setLinearHeadingInterpolation(secondPick.getHeading(), thirdScore.getHeading())
                .build();
    }

    public PathChain pickThree() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(thirdScore, setThirdPick, thirdPick))
                .setReversed()
                .setBrakingStrength(.75)
                .setLinearHeadingInterpolation(setThirdPick.getHeading(), thirdPick.getHeading())
                .build();
    }

    public PathChain scoreFourth() {
        return follower.pathBuilder()
                .addPath(new BezierLine(thirdPick, fourthScore))
                .setReversed()
                .setLinearHeadingInterpolation(thirdPick.getHeading(), fourthScore.getHeading())
                .build();
    }

    public PathChain next() {
        switch (index++) {
            case 0: return scoreP();
            case 1: return pickOne();
            case 2: return scoreTwo();
            case 3: return pickTwo();
            case 4: return scoreThird();
            case 5: return pickThree();
            case 6: return scoreFourth();
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
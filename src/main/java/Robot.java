import Enums.RobotPrimitive;
import simbad.sim.Agent;
import simbad.sim.LightSensor;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class Robot extends Agent
{
    LightSensor centerLS, leftLS, rightLS;
    RangeSensorBelt sensorBelt;
    RobotPrimitive robotPrimitive;

    double lumIntensity1, lumIntensity2, lumIntensity3;
    double targetLum = 0.95;
    double distanceToObstacleLimit = 0.65;
    double iL, iH;

    static double K1 = 3;
    static double K2 = 0.4;
    static double K3 = 5;

    public Robot (Vector3d position, String name) {
        super(position, name);

        leftLS = RobotFactory.addLightSensorLeft(this);
        rightLS = RobotFactory.addLightSensorRight(this);
        centerLS = RobotFactory.addLightSensor(this);
        sensorBelt = RobotFactory.addSonarBeltSensor(this, 8);

        robotPrimitive = RobotPrimitive.ORIENTATE;
    }

    public void initBehavior()
    {
        lumIntensity1 = lumIntensity2 = lumIntensity3 = iL = iH = 0;
    }

    public void performBehavior()
    {
        double rightLightIntensity = rightLS.getLux() * 10;
        double leftLightIntensity = leftLS.getLux() * 10;
        double centerLightIntensity = centerLS.getLux() * 10;

        lumIntensity1 = lumIntensity2;
        lumIntensity2 = lumIntensity3;
        lumIntensity3 = centerLightIntensity;

        if (centerLightIntensity > targetLum) {
            robotPrimitive = RobotPrimitive.END;
        }

        switch (robotPrimitive) {
            case END:
                stop();
                break;
            case ORIENTATE:
                iL = centerLightIntensity;
                orientate(rightLightIntensity, leftLightIntensity, centerLightIntensity);
                break;
            case MOVE_FORWARD:
                moveForward(sensorBelt);
                centerLightIntensity = centerLS.getLux() * 10;
                if (iL != centerLightIntensity) {
                    iH = centerLightIntensity;
                }
                break;
            case FOLLOW:
                follow(sensorBelt);
                if (centerLightIntensity > iH && lumIntensity2 > lumIntensity1 && lumIntensity2 > lumIntensity3) {
                    robotPrimitive = RobotPrimitive.ORIENTATE;
                }
                break;
        }
    }

    private void stop()
    {
        setRotationalVelocity(0);
        setTranslationalVelocity(0);
    }

    private void orientate(double rightLightIntensity, double leftLightIntensity, double centerLightIntensity) {
        setTranslationalVelocity(0);

        double threshold = 0.001;
        if ((rightLightIntensity + leftLightIntensity + centerLightIntensity / 3) > 0.5) {
            threshold = 0.007;
        }

        if (Math.abs(rightLightIntensity - leftLightIntensity) > threshold) {
            setRotationalVelocity(Math.signum(leftLightIntensity - rightLightIntensity) * 1);
        } else if (centerLightIntensity > leftLightIntensity) {
            setRotationalVelocity(2);
        } else {
            setRotationalVelocity(0);
            robotPrimitive = RobotPrimitive.MOVE_FORWARD;
        }
    }

    private void moveForward(RangeSensorBelt sensorBelt) {
        boolean robotTooClose = sensorBelt.getFrontLeftQuadrantMeasurement() < distanceToObstacleLimit || sensorBelt.getFrontRightQuadrantMeasurement() < distanceToObstacleLimit;

        if (robotTooClose) {
            robotPrimitive = RobotPrimitive.FOLLOW;
        } else {
            setTranslationalVelocity(2);
            robotPrimitive = RobotPrimitive.ORIENTATE;
        }
    }

    private void follow(RangeSensorBelt sensorBelt) {
        int closestSensor = 0;
        double closestSensorMeasurement;
        for (int i = 0; i < sensorBelt.getNumSensors(); i++) {
            if(sensorBelt.getMeasurement(i) < sensorBelt.getMeasurement(closestSensor)) {
                closestSensor = i;
            }
        }
        closestSensorMeasurement = sensorBelt.getMeasurement(closestSensor);

        double sensedDistance;
        if (sensorBelt.hasHit(closestSensor)) {
            sensedDistance = getRadius() + closestSensorMeasurement;
        } else {
            sensedDistance = getRadius() + sensorBelt.getMaxRange();
        }

        Point3d pointOfImpact = new Point3d(
                sensedDistance * Math.cos(sensorBelt.getSensorAngle(closestSensor)),
                0,
                sensedDistance * Math.sin(sensorBelt.getSensorAngle(closestSensor))
        );
        double distanceToPointOfImpact = pointOfImpact.distance(new Point3d(0, 0, 0));

        Vector3d vector = new Vector3d(pointOfImpact.z, 0, -pointOfImpact.x); //COUNTER-CLOCK WISE
        double phLin = Math.atan2(vector.z, vector.x);
        double phRot = Math.atan(K3 * (distanceToPointOfImpact - distanceToObstacleLimit));
        double phRef = wrapToPi(phLin +  phRot);

        setRotationalVelocity(K1 * phRef);
        setTranslationalVelocity(K2 * Math.cos(phRef));
    }

    public double wrapToPi(double a)
    {
        while (a>Math.PI)
            a -= Math.PI*2;
        while (a<=-Math.PI)
            a += Math.PI*2;
        return a;
    }
}

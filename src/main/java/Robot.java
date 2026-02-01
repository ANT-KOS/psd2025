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
    double targetLum = 0.8;
    double distanceToObstacleLimit = 0.75;
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
        double rightLightIntensity = leftLS.getLux() * 10;
        double leftLightIntensity = rightLS.getLux() * 10;
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

        if (Math.abs(rightLightIntensity - leftLightIntensity) > 0.01) {
            setRotationalVelocity(Math.signum(rightLightIntensity - leftLightIntensity) * 0.7);
        } else if (centerLightIntensity > leftLightIntensity) {
            setRotationalVelocity(0.5);
        } else {
            setRotationalVelocity(0);
            robotPrimitive = RobotPrimitive.MOVE_FORWARD;
        }
    }

    private void moveForward(RangeSensorBelt sensorBelt) {
        boolean robotTooClose = false;
        for (int i = 0; i < sensorBelt.getNumSensors(); i++) {
            if (sensorBelt.getMeasurement(i) < distanceToObstacleLimit) {
                robotTooClose = true;
            }
        }

        if (robotTooClose) {
            robotPrimitive = RobotPrimitive.FOLLOW;
        } else {
            setTranslationalVelocity(1.5);
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

        double robotAngle;
        if (sensorBelt.hasHit(closestSensor)) {
            robotAngle = getRadius() + closestSensorMeasurement;
        } else {
            robotAngle = getRadius() + sensorBelt.getMaxRange();
        }

        Point3d pointOfImpact = new Point3d(
                robotAngle * Math.cos(sensorBelt.getSensorAngle(closestSensor)),
                0,
                robotAngle * Math.sin(sensorBelt.getSensorAngle(closestSensor))
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

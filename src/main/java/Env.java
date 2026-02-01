import simbad.sim.Box;
import simbad.sim.EnvironmentDescription;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

public class Env extends EnvironmentDescription
{
    public Env()
    {
        light1IsOn = true;
        light1SetPosition(-4, 2, -7);
        add(new Box(new Vector3d(0,0,7), new Vector3f(14,1,1),this));
        add(new Box(new Vector3d(-2,0,3), new Vector3f(12,1,1),this));
        add(new Box(new Vector3d(-1.5,0,-1), new Vector3f(6,1,1),this));
        add(new Box(new Vector3d(-1,0,-4), new Vector3f(8,1,1),this));
        add(new Robot(new Vector3d(-8, 0, 8), "Primary Robot"));
    }
}

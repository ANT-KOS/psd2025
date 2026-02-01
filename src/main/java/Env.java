import simbad.sim.Box;
import simbad.sim.EnvironmentDescription;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

public class Env extends EnvironmentDescription
{
    public Env()
    {
        light1IsOn = true;
        light1SetPosition(0, 2, -3);
        add(new Box(new Vector3d(0,0,0), new Vector3f(5,1,5), this));
        add(new Robot(new Vector3d(-6, 0, 5), "Primary Robot"));
    }
}

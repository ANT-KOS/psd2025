import simbad.gui.Simbad;
import simbad.sim.*;

public class Main {

    public static void main(String[] args) {
            Simbad simbad = new Simbad(new Env(), false);
            simbad.update(simbad.getGraphics());
        }
}

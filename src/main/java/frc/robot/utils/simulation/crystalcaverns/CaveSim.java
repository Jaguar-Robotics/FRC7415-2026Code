package frc.robot.utils.simulation.crystalcaverns;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.SimulatedArena;

/**
 *
 *
 * <h2>Simulates a <strong>CAVE</strong>s on the field.</h2>
 *
 * <p>This class simulates a <strong>CAVE</strong>s on the field where <strong>CRYSTAL</strong>s can be scored. This class
 * does not directly handle scoring uses an array of {@link CaveBranch} objects. However for all other purposes
 * this class behaves and can be used like a normal goal.
 */
public class CaveSim implements SimulatedArena.Simulatable {
    protected final List<CaveBranch> branches;
    Pose3d[] branchPoses;

    CaveSim(CrystalCavernsArena arena, boolean isBlue) {
        branches = new ArrayList<CaveBranch>(24);
        branchPoses = new Pose3d[24 * 2];
        int idx = 0;
        for (int face = 0; face < 8; face++) {
            branches.add(new CaveBranch(arena, isBlue, 0, face));
            Pose3d tempPose = branches.get(branches.size() - 1).getPose();
            branchPoses[idx++] = tempPose;
            branchPoses[idx++] = 
                    new Pose3d(tempPose.getTranslation(), Goal.flipRotation(tempPose.getRotation()));

        }
        for (int quadrant = 0; quadrant < 4; quadrant++) {
            branches.add(new CaveBranch(arena, isBlue, 1, quadrant));
            Pose3d tempPose = branches.get(branches.size() - 1).getPose();
            branchPoses[idx++] = tempPose;
            branchPoses[idx++] = 
                    new Pose3d(tempPose.getTranslation(), Goal.flipRotation(tempPose.getRotation()));

        }
        // for (int tower = 0; tower < 12; tower++) {
        //     for (int level = 0; level < 1; level++) {
        //         branches.add(new CaveBranch(arena, isBlue, level, tower));
        //         Pose3d tempPose = branches.get(branches.size() - 1).getPose();

        //         branchPoses[2 * (tower * 4 + level)] = tempPose;

        //         branchPoses[2 * (tower * 4 + level) + 1] =
        //                 new Pose3d(tempPose.getTranslation(), Goal.flipRotation(tempPose.getRotation()));
        //     }
        // }
    }

    public void draw(List<Pose3d> coralPosesToDisplay) {
        for (CaveBranch branch : branches) {
            branch.draw(coralPosesToDisplay);
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        for (CaveBranch branch : branches) {
            branch.simulationSubTick(subTickNum);
        }
    }

    public void clearCave() {
        for (CaveBranch branch : branches) {
            branch.clear();
        }
    }

    /**
     * Obtains the amount of <strong>CRYSTAL</strong> held on the <strong>BRANCHES</strong>.
     *
     * <p>This method returns a 2D array of size 12 x 4, where each entry represents the number of
     * <strong>CRYSTAL</strong>s held on a particular branch.
     *
     * <p>The <strong>BRANCHES</strong> are tracked in FMS as A, B, C, D, E, F, G, H, I, J, K, L (as per the game
     * manual), and are mapped to indices 0, 1, 2, ... in the array.
     *
     * <p>The [i][j] entry in the array represents the number of <strong>CRYSTAL</strong>(s) held on the L<code>j-1</code>
     * branch in the <code>i</code>th section.
     *
     * <p>For example, <code>getBranches()[2][3]</code> returns the number of CRYSTALs held on L4 of Branch C.
     *
     * <p>Note that L2, L3, and L4 can only hold one <strong>CRYSTAL</strong>, while L1 can hold up to two
     * <strong>CRYSTAL</strong>s.
     *
     * @return a 2D array where each entry represents the number of <strong>CRYSTAL</strong> held on each branch
     */
    public int[][] getBranches() {
        int[][] toReturn = new int[8][2];
        for (CaveBranch branch : branches) {
            toReturn[branch.column][branch.level] = branch.getGamePieceCount();
        }
        return toReturn;
    }
}
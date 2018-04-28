package NonRigid2D;

import java.util.List;

import ij.IJ;

import java.util.ArrayList;

public class ClusterTree {

	private List<Cluster1[]> subclusters = new ArrayList<Cluster1[]>();
	private Node root;
	private int iterations = 1;
	private ClosestPoint1 cp;

	public class Node {
		private Cluster1[] cluster;
		private Node left;
		private Node right;

		public Node(Cluster1[] c) {
			this.cluster = c;
			right = null;
			left = null;
		}
	}

	public ClusterTree(Cluster1 c1, Cluster1 c2) {
		root = new Node(new Cluster1[] { c1, c2 });
		root.left = null;
		root.right = null;
	}

	/**
	 * Recursively subdivide a Node containing two clusters in each two sub
	 * clusters until all parts match
	 * 
	 * @param node
	 * @return segmented parts
	 */
	public List<Cluster1[]> subdivide(Node node) {
		cp = new ClosestPoint1(node.cluster[0], node.cluster[1]);

		if (iterations > 50) {
			return subclusters;
		}

		if (!cp.match()) {
			split(node);
			IJ.log("Iteration #" + iterations++);

			subdivide(node.left);
			subdivide(node.right);
		} else {
			IJ.log("Add cluster to list!");
			subclusters.add(node.cluster);
		}
		return subclusters;
	}

	/**
	 * 
	 * @param currentNode
	 */
	private void split(Node currentNode) {
		Cluster1[] c1 = currentNode.cluster[0].divideCluster();
		Cluster1[] c2 = currentNode.cluster[1].divideCluster();

		currentNode.left = new Node(new Cluster1[] { c1[0], c2[0] });
		currentNode.right = new Node(new Cluster1[] { c1[1], c2[1] });
	}

	/**
	 * Merge subclusters of C1 and C2 to clusters that can still be matched
	 * 
	 * @param subclusters
	 * @return mergedParts
	 */
	public List<Cluster1[]> mergeClusters(List<Cluster1[]> subclusters) {
		List<Cluster1[]> mergedParts = new ArrayList<Cluster1[]>();

		if (subclusters != null) {

			Cluster1 toBeMergedC1 = subclusters.get(0)[0];
			Cluster1 toBeMergedC2 = subclusters.get(0)[1];

			for (int i = 1; i < subclusters.size(); i++) {

				Cluster1 currentSegment1 = subclusters.get(i)[0];
				Cluster1 currentSegment2 = subclusters.get(i)[1];

				Cluster1 merged1 = toBeMergedC1.mergeClusters(currentSegment1);
				Cluster1 merged2 = toBeMergedC2.mergeClusters(currentSegment2);

				ClosestPoint1 cp = new ClosestPoint1(merged1, merged2);
				if (cp.match()) {
					toBeMergedC1 = merged1;
					toBeMergedC2 = merged2;
				}

				else if (!cp.match()) {
					mergedParts.add(new Cluster1[] { toBeMergedC1, toBeMergedC2 });
					toBeMergedC1 = currentSegment1;
					toBeMergedC2 = currentSegment2;
				}
			}
			mergedParts.add(new Cluster1[] { toBeMergedC1, toBeMergedC2 });
		}
		return mergedParts;
	}

	public Node getRoot() {
		return root;
	}
}

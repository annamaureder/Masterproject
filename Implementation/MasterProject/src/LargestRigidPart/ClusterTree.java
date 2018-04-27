package LargestRigidPart;

import java.util.List;

import ij.IJ;

import java.util.ArrayList;

public class ClusterTree {

	private List<Cluster[]> subclusters = new ArrayList<Cluster[]>();
	private Node root;
	private int iterations = 1;
	private ClosestPoint cp;

	public class Node {
		private Cluster[] cluster;
		private Node left;
		private Node right;

		public Node(Cluster[] c) {
			this.cluster = c;
			right = null;
			left = null;
		}
	}

	public ClusterTree(Cluster c1, Cluster c2) {
		root = new Node(new Cluster[] { c1, c2 });
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
	public List<Cluster[]> subdivide(Node node) {
		cp = new ClosestPoint(node.cluster[0], node.cluster[1]);

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
		Cluster[] c1 = currentNode.cluster[0].divideCluster();
		Cluster[] c2 = currentNode.cluster[1].divideCluster();

		currentNode.left = new Node(new Cluster[] { c1[0], c2[0] });
		currentNode.right = new Node(new Cluster[] { c1[1], c2[1] });
	}

	/**
	 * Merge subclusters of C1 and C2 to clusters that can still be matched
	 * 
	 * @param subclusters
	 * @return mergedParts
	 */
	public List<Cluster[]> mergeClusters(List<Cluster[]> subclusters) {
		List<Cluster[]> mergedParts = new ArrayList<Cluster[]>();

		if (subclusters != null) {

			Cluster toBeMergedC1 = subclusters.get(0)[0];
			Cluster toBeMergedC2 = subclusters.get(0)[1];

			for (int i = 1; i < subclusters.size(); i++) {

				Cluster currentSegment1 = subclusters.get(i)[0];
				Cluster currentSegment2 = subclusters.get(i)[1];

				Cluster merged1 = toBeMergedC1.mergeClusters(currentSegment1);
				Cluster merged2 = toBeMergedC2.mergeClusters(currentSegment2);

				ClosestPoint cp = new ClosestPoint(merged1, merged2);
				if (cp.match()) {
					toBeMergedC1 = merged1;
					toBeMergedC2 = merged2;
				}

				else if (!cp.match()) {
					mergedParts.add(new Cluster[] { toBeMergedC1, toBeMergedC2 });
					toBeMergedC1 = currentSegment1;
					toBeMergedC2 = currentSegment2;
				}
			}
			mergedParts.add(new Cluster[] { toBeMergedC1, toBeMergedC2 });
		}
		return mergedParts;
	}

	public Node getRoot() {
		return root;
	}
}

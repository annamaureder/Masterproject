package NonRigid2D;

import java.util.List;

import ij.IJ;

import java.util.ArrayList;

public class ClusterTree {

	private List<Cluster[]> segmentedParts = new ArrayList<Cluster[]>();
	private Node root;
	private int iterations = 1;

	public class Node {
		private Cluster[] value;
		private Node left;
		private Node right;

		public Node(Cluster[] c) {
			this.value = c;
			right = null;
			left = null;
		}
	}

	public ClusterTree(Cluster c1, Cluster c2) {

		root = new Node(new Cluster[] { c1, c2 });
		root.left = null;
		root.right = null;

	}

	public void divideCluster(Node currentNode) {

		Cluster[] template = currentNode.value[0].divideCluster();
		Cluster[] clusters = currentNode.value[1].divideCluster();

		currentNode.left = new Node(new Cluster[] { template[0], clusters[0] });
		currentNode.right = new Node(new Cluster[] { template[1], clusters[1] });
	}

	public boolean checkClusters() {

		traverseTree(root);
		IJ.log("number of iterations: " + iterations);

		return true;

	}

	private void traverseTree(Node node) {

		// currentNode = node;

		ClosestPoint cp = new ClosestPoint(node.value[0], node.value[1]);

		if (!cp.match()) {
			divideCluster(node);
			iterations++;

			traverseTree(node.left);
			traverseTree(node.right);
		} else {
			segmentedParts.add(node.value);
		}
	}

	public List<Cluster[]> getSegmentedParts() {
		return segmentedParts;

	}

	public List<Cluster[]> getRigidParts() {

		List<Cluster[]> rigidParts = new ArrayList<Cluster[]>();

		if (segmentedParts != null) {

			// ToDo: From left to Right --> merge each two clusters, if they
			// still match, add another one.
			// in case of not matching --> save last cluster as rigid part

			Cluster toBeMerged1 = segmentedParts.get(0)[0];
			Cluster toBeMerged2 = segmentedParts.get(0)[1];

			for (int i = 1; i < segmentedParts.size(); i++) {

				Cluster currentSegment1 = segmentedParts.get(i)[0];
				Cluster currentSegment2 = segmentedParts.get(i)[1];

				Cluster merged1 = toBeMerged1.mergeClusters(currentSegment1);
				Cluster merged2 = toBeMerged2.mergeClusters(currentSegment2);

				ClosestPoint cp = new ClosestPoint(merged1, merged2);
				if (cp.match()) {
					toBeMerged1 = merged1;
					toBeMerged2 = merged2;
				}

				else if (!cp.match()) {
					rigidParts.add(new Cluster[] { toBeMerged1, toBeMerged2 });
					toBeMerged1 = currentSegment1;
					toBeMerged2 = currentSegment2;
				}
			}

			rigidParts.add(new Cluster[] { toBeMerged1, toBeMerged2 });

		}

		return rigidParts;

	}

}

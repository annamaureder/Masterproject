package NonRigid2D;

import java.util.List;

import ij.IJ;

import java.util.ArrayList;

public class _ClusterTree {

	private List<_Cluster[]> segmentedParts = new ArrayList<_Cluster[]>();
	private Node root;
	private int iterations = 1;
	private _ClosestPoint cp;

	public class Node {
		private _Cluster[] value;
		private Node left;
		private Node right;

		public Node(_Cluster[] c) {
			this.value = c;
			right = null;
			left = null;
		}
	}

	public _ClusterTree(_Cluster c1, _Cluster c2) {

		root = new Node(new _Cluster[] { c1, c2 });
		root.left = null;
		root.right = null;

	}

	public void divideCluster(Node currentNode) {

		_Cluster[] template = currentNode.value[0].divideCluster();
		_Cluster[] clusters = currentNode.value[1].divideCluster();

		currentNode.left = new Node(new _Cluster[] { template[0], clusters[0] });
		currentNode.right = new Node(new _Cluster[] { template[1], clusters[1] });
	}

	public List<_Cluster[]> subdivide(List<_Cluster[]> segmentedParts) {
		
		this.segmentedParts = segmentedParts;

		divide(root);
		IJ.log("number of iterations: " + iterations);

		return this.segmentedParts;

	}

	private void divide(Node node) {

		cp = new _ClosestPoint(node.value[0], node.value[1]);

		if (!cp.match()) {
			divideCluster(node);
			iterations++;

			divide(node.left);
			divide(node.right);
		} else {
			segmentedParts.add(node.value);
		}
	}

	public List<_Cluster[]> getSegmentedParts() {
		return segmentedParts;

	}

	public List<_Cluster[]> getRigidParts(List<_Cluster[]> segmentedParts) {

		List<_Cluster[]> rigidParts = new ArrayList<_Cluster[]>();

		if (segmentedParts != null) {

			// ToDo: From left to Right --> merge each two clusters, if they
			// still match, add another one.
			// in case of not matching --> save last cluster as rigid part

			_Cluster toBeMerged1 = segmentedParts.get(0)[0];
			_Cluster toBeMerged2 = segmentedParts.get(0)[1];

			for (int i = 1; i < segmentedParts.size(); i++) {

				_Cluster currentSegment1 = segmentedParts.get(i)[0];
				_Cluster currentSegment2 = segmentedParts.get(i)[1];

				_Cluster merged1 = toBeMerged1.mergeClusters(currentSegment1);
				_Cluster merged2 = toBeMerged2.mergeClusters(currentSegment2);

				_ClosestPoint cp = new _ClosestPoint(merged1, merged2);
				if (cp.match()) {
					toBeMerged1 = merged1;
					toBeMerged2 = merged2;
				}

				else if (!cp.match()) {
					rigidParts.add(new _Cluster[] { toBeMerged1, toBeMerged2 });
					toBeMerged1 = currentSegment1;
					toBeMerged2 = currentSegment2;
				}
			}

			rigidParts.add(new _Cluster[] { toBeMerged1, toBeMerged2 });

		}

		return rigidParts;

	}
	
	public List<double[]> calculateJoints(){
		
		//ToDo: Joints are points located at the intersection of the principal axis of two neighboring rigid parts.
		
		return null;
		
	}

}

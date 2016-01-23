/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.collision.shapes;

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.Stack;

// JAVA NOTE: CompoundShape from 2.71

/** CompoundShape allows to store multiple other {@link CollisionShape}s. This allows for moving concave collision objects. This
 * is more general than the {@link BvhTriangleMeshShape}.
 * 
 * @author jezek2 */
public class CompoundShape extends CollisionShape {

	private final ObjectArrayList<CompoundShapeChild> children = new ObjectArrayList<CompoundShapeChild>();
	private final Vector3 localAabbMin = new Vector3(1e30f, 1e30f, 1e30f);
	private final Vector3 localAabbMax = new Vector3(-1e30f, -1e30f, -1e30f);

	private OptimizedBvh aabbTree = null;

	private float collisionMargin = 0f;
	protected final Vector3 localScaling = new Vector3(1f, 1f, 1f);

	public void addChildShape (Transform localTransform, CollisionShape shape) {
		// m_childTransforms.push_back(localTransform);
		// m_childShapes.push_back(shape);
		CompoundShapeChild child = new CompoundShapeChild();
		child.transform.set(localTransform);
		child.childShape = shape;
		child.childShapeType = shape.getShapeType();
		child.childMargin = shape.getMargin();

		children.add(child);

		Stack stack = Stack.enter();

		// extend the local aabbMin/aabbMax
		Vector3 _localAabbMin = stack.allocVector3(), _localAabbMax = stack.allocVector3();
		shape.getAabb(localTransform, _localAabbMin, _localAabbMax);

		// JAVA NOTE: rewritten
// for (int i=0;i<3;i++)
// {
// if (this.localAabbMin[i] > _localAabbMin[i])
// {
// this.localAabbMin[i] = _localAabbMin[i];
// }
// if (this.localAabbMax[i] < _localAabbMax[i])
// {
// this.localAabbMax[i] = _localAabbMax[i];
// }
// }
		VectorUtil.setMin(this.localAabbMin, _localAabbMin);
		VectorUtil.setMax(this.localAabbMax, _localAabbMax);
		stack.leave();
	}

	/** Remove all children shapes that contain the specified shape. */
	public void removeChildShape (CollisionShape shape) {
		boolean done_removing;

		// Find the children containing the shape specified, and remove those children.
		do {
			done_removing = true;

			for (int i = 0; i < children.size(); i++) {
				if (children.getQuick(i).childShape == shape) {
					children.removeQuick(i);
					done_removing = false; // Do another iteration pass after removing from the vector
					break;
				}
			}
		} while (!done_removing);

		recalculateLocalAabb();
	}

	public int getNumChildShapes () {
		return children.size();
	}

	public CollisionShape getChildShape (int index) {
		return children.getQuick(index).childShape;
	}

	public Transform getChildTransform (int index, Transform out) {
		out.set(children.getQuick(index).transform);
		return out;
	}

	public ObjectArrayList<CompoundShapeChild> getChildList () {
		return children;
	}

	/** getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version. */
	@Override
	public void getAabb (Transform trans, Vector3 aabbMin, Vector3 aabbMax) {
		Stack stack = Stack.enter();
		Vector3 localHalfExtents = stack.allocVector3();
		localHalfExtents.set(localAabbMax).sub(localAabbMin);
		localHalfExtents.scl(0.5f);
		localHalfExtents.x += getMargin();
		localHalfExtents.y += getMargin();
		localHalfExtents.z += getMargin();

		Vector3 localCenter = stack.allocVector3();
		localCenter.set(localAabbMax).add(localAabbMin);
		localCenter.scl(0.5f);

		Matrix3 abs_b = stack.alloc(trans.basis);
		MatrixUtil.absolute(abs_b);

		Vector3 center = stack.alloc(localCenter);
		trans.transform(center);

		Vector3 tmp = stack.allocVector3();

		Vector3 extent = stack.allocVector3();
		tmp.x = abs_b.val[Matrix3.M00];
		tmp.y = abs_b.val[Matrix3.M01];
		tmp.z = abs_b.val[Matrix3.M02];
		extent.x = tmp.dot(localHalfExtents);
		tmp.x = abs_b.val[Matrix3.M10];
		tmp.y = abs_b.val[Matrix3.M11];
		tmp.z = abs_b.val[Matrix3.M12];
		extent.y = tmp.dot(localHalfExtents);
		tmp.x = abs_b.val[Matrix3.M20];
		tmp.y = abs_b.val[Matrix3.M21];
		tmp.z = abs_b.val[Matrix3.M22];
		extent.z = tmp.dot(localHalfExtents);

		aabbMin.set(center).sub(extent);
		aabbMax.set(center).add(extent);
		stack.leave();
	}

	/** Re-calculate the local Aabb. Is called at the end of removeChildShapes. Use this yourself if you modify the children or
	 * their transforms. */
	public void recalculateLocalAabb () {
		// Recalculate the local aabb
		// Brute force, it iterates over all the shapes left.
		Stack stack = Stack.enter();
		localAabbMin.set(1e30f, 1e30f, 1e30f);
		localAabbMax.set(-1e30f, -1e30f, -1e30f);

		Vector3 tmpLocalAabbMin = stack.allocVector3();
		Vector3 tmpLocalAabbMax = stack.allocVector3();

		// extend the local aabbMin/aabbMax
		for (int j = 0; j < children.size(); j++) {
			children.getQuick(j).childShape.getAabb(children.getQuick(j).transform, tmpLocalAabbMin, tmpLocalAabbMax);

			for (int i = 0; i < 3; i++) {
				if (VectorUtil.getCoord(localAabbMin, i) > VectorUtil.getCoord(tmpLocalAabbMin, i)) {
					VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(tmpLocalAabbMin, i));
				}
				if (VectorUtil.getCoord(localAabbMax, i) < VectorUtil.getCoord(tmpLocalAabbMax, i)) {
					VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(tmpLocalAabbMax, i));
				}
			}
		}
		stack.leave();
	}

	@Override
	public void setLocalScaling (Vector3 scaling) {
		localScaling.set(scaling);
	}

	@Override
	public Vector3 getLocalScaling (Vector3 out) {
		out.set(localScaling);
		return out;
	}

	@Override
	public void calculateLocalInertia (float mass, Vector3 inertia) {
		// approximation: take the inertia from the aabb for now
		Stack stack = Stack.enter();
		Transform ident = stack.allocTransform();
		ident.setIdentity();
		Vector3 aabbMin = stack.allocVector3(), aabbMax = stack.allocVector3();
		getAabb(ident, aabbMin, aabbMax);

		Vector3 halfExtents = stack.allocVector3();
		halfExtents.set(aabbMax).sub(aabbMin);
		halfExtents.scl(0.5f);

		float lx = 2f * halfExtents.x;
		float ly = 2f * halfExtents.y;
		float lz = 2f * halfExtents.z;

		inertia.x = (mass / 12f) * (ly * ly + lz * lz);
		inertia.y = (mass / 12f) * (lx * lx + lz * lz);
		inertia.z = (mass / 12f) * (lx * lx + ly * ly);
		stack.leave();
	}

	@Override
	public BroadphaseNativeType getShapeType () {
		return BroadphaseNativeType.COMPOUND_SHAPE_PROXYTYPE;
	}

	@Override
	public void setMargin (float margin) {
		collisionMargin = margin;
	}

	@Override
	public float getMargin () {
		return collisionMargin;
	}

	@Override
	public String getName () {
		return "Compound";
	}

	// this is optional, but should make collision queries faster, by culling non-overlapping nodes
	// void createAabbTreeFromChildren();

	public OptimizedBvh getAabbTree () {
		return aabbTree;
	}

	/** Computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the
	 * moment of inertia and the center of mass to the current coordinate system. "masses" points to an array of masses of the
	 * children. The resulting transform "principal" has to be applied inversely to all children transforms in order for the local
	 * coordinate system of the compound shape to be centered at the center of mass and to coincide with the principal axes. This
	 * also necessitates a correction of the world transform of the collision object by the principal transform. */
	public void calculatePrincipalAxisTransform (float[] masses, Transform principal, Vector3 inertia) {
		int n = children.size();
		Stack stack = Stack.enter();
		float totalMass = 0;
		Vector3 center = stack.allocVector3();
		center.set(0, 0, 0);
		for (int k = 0; k < n; k++) {
			center.x += masses[k] * children.getQuick(k).transform.origin.x;
			center.y += masses[k] * children.getQuick(k).transform.origin.y;
			center.z += masses[k] * children.getQuick(k).transform.origin.z;
			totalMass += masses[k];
		}
		center.scl(1f / totalMass);
		principal.origin.set(center);

		Matrix3 tensor = stack.allocMatrix3();
		tensor.idt();
		tensor.val[Matrix3.M00] = 0;
		tensor.val[Matrix3.M11] = 0;
		tensor.val[Matrix3.M22] = 0;

		for (int k = 0; k < n; k++) {
			Vector3 i = stack.allocVector3();
			children.getQuick(k).childShape.calculateLocalInertia(masses[k], i);

			Transform t = children.getQuick(k).transform;
			Vector3 o = stack.allocVector3();
			o.set(t.origin).sub(center);

			// compute inertia tensor in coordinate system of compound shape
			Matrix3 j = stack.allocMatrix3();
			j.set(t.basis).transpose();

			j.val[Matrix3.M00] *= i.x;
			j.val[Matrix3.M01] *= i.x;
			j.val[Matrix3.M02] *= i.x;
			j.val[Matrix3.M10] *= i.y;
			j.val[Matrix3.M11] *= i.y;
			j.val[Matrix3.M12] *= i.y;
			j.val[Matrix3.M20] *= i.z;
			j.val[Matrix3.M21] *= i.z;
			j.val[Matrix3.M22] *= i.z;

			j.mulLeft(t.basis);

			// add inertia tensor
			MatrixUtil.add(tensor, j);

			// compute inertia tensor of pointmass at o
			float o2 = o.len2();
			MatrixUtil.setRow(j, 0, o2, 0, 0);
			MatrixUtil.setRow(j, 1, 0, o2, 0);
			MatrixUtil.setRow(j, 2, 0, 0, o2);
			j.val[Matrix3.M00] += o.x * -o.x;
			j.val[Matrix3.M01] += o.y * -o.x;
			j.val[Matrix3.M02] += o.z * -o.x;
			j.val[Matrix3.M10] += o.x * -o.y;
			j.val[Matrix3.M11] += o.y * -o.y;
			j.val[Matrix3.M12] += o.z * -o.y;
			j.val[Matrix3.M20] += o.x * -o.z;
			j.val[Matrix3.M21] += o.y * -o.z;
			j.val[Matrix3.M22] += o.z * -o.z;

			// add inertia tensor of pointmass
			tensor.val[Matrix3.M00] += masses[k] * j.val[Matrix3.M00];
			tensor.val[Matrix3.M01] += masses[k] * j.val[Matrix3.M01];
			tensor.val[Matrix3.M02] += masses[k] * j.val[Matrix3.M02];
			tensor.val[Matrix3.M10] += masses[k] * j.val[Matrix3.M10];
			tensor.val[Matrix3.M11] += masses[k] * j.val[Matrix3.M11];
			tensor.val[Matrix3.M12] += masses[k] * j.val[Matrix3.M12];
			tensor.val[Matrix3.M20] += masses[k] * j.val[Matrix3.M20];
			tensor.val[Matrix3.M21] += masses[k] * j.val[Matrix3.M21];
			tensor.val[Matrix3.M22] += masses[k] * j.val[Matrix3.M22];
		}

		MatrixUtil.diagonalize(tensor, principal.basis, 0.00001f, 20);

		inertia.set(tensor.val[Matrix3.M00], tensor.val[Matrix3.M11], tensor.val[Matrix3.M22]);
		stack.leave();
	}

}

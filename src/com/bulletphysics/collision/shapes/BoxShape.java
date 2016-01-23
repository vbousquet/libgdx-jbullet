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

import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.ScalarUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

/**
 * BoxShape is a box primitive around the origin, its sides axis aligned with length
 * specified by half extents, in local shape coordinates. When used as part of a
 * {@link CollisionObject} or {@link RigidBody} it will be an oriented box in world space.
 *
 * @author jezek2
 */
public class BoxShape extends PolyhedralConvexShape {

	public BoxShape(Vector3 boxHalfExtents) {
		Vector3 margin = new Vector3(getMargin(), getMargin(), getMargin());
		VectorUtil.mul(implicitShapeDimensions, boxHalfExtents, localScaling);
		implicitShapeDimensions.sub(margin);
	}

	public Vector3 getHalfExtentsWithMargin(Vector3 out) {
	    Stack stack = Stack.enter();
		Vector3 halfExtents = getHalfExtentsWithoutMargin(out);
		Vector3 margin = stack.allocVector3();
		margin.set(getMargin(), getMargin(), getMargin());
		halfExtents.add(margin);
		stack.leave();
		return out;
	}

	public Vector3 getHalfExtentsWithoutMargin(Vector3 out) {
		out.set(implicitShapeDimensions); // changed in Bullet 2.63: assume the scaling and margin are included
		return out;
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.BOX_SHAPE_PROXYTYPE;
	}

	@Override
	public Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out) {
		Vector3 halfExtents = getHalfExtentsWithoutMargin(out);
		
		float margin = getMargin();
		halfExtents.x += margin;
		halfExtents.y += margin;
		halfExtents.z += margin;

		out.set(
				ScalarUtil.fsel(vec.x, halfExtents.x, -halfExtents.x),
				ScalarUtil.fsel(vec.y, halfExtents.y, -halfExtents.y),
				ScalarUtil.fsel(vec.z, halfExtents.z, -halfExtents.z));
		return out;
	}

	@Override
	public Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
		Vector3 halfExtents = getHalfExtentsWithoutMargin(out);

		out.set(
				ScalarUtil.fsel(vec.x, halfExtents.x, -halfExtents.x),
				ScalarUtil.fsel(vec.y, halfExtents.y, -halfExtents.y),
				ScalarUtil.fsel(vec.z, halfExtents.z, -halfExtents.z));
		return out;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut, int numVectors) {
	    Stack stack = Stack.enter();
	    Vector3 halfExtents = getHalfExtentsWithoutMargin(stack.allocVector3());

		for (int i = 0; i < numVectors; i++) {
			Vector3 vec = vectors[i];
			supportVerticesOut[i].set(ScalarUtil.fsel(vec.x, halfExtents.x, -halfExtents.x),
					ScalarUtil.fsel(vec.y, halfExtents.y, -halfExtents.y),
					ScalarUtil.fsel(vec.z, halfExtents.z, -halfExtents.z));
		}
		stack.leave();
	}

	@Override
	public void setMargin(float margin) {
	    Stack stack = Stack.enter();
		// correct the implicitShapeDimensions for the margin
		Vector3 oldMargin = stack.allocVector3();
		oldMargin.set(getMargin(), getMargin(), getMargin());
		Vector3 implicitShapeDimensionsWithMargin = stack.allocVector3();
		implicitShapeDimensionsWithMargin.set(implicitShapeDimensions).add(oldMargin);

		super.setMargin(margin);
		Vector3 newMargin = stack.allocVector3();
		newMargin.set(getMargin(), getMargin(), getMargin());
		implicitShapeDimensions.set(implicitShapeDimensionsWithMargin).sub(newMargin);
		stack.leave();
	}

	@Override
	public void setLocalScaling(Vector3 scaling) {
	    Stack stack = Stack.enter();
		Vector3 oldMargin = stack.allocVector3();
		oldMargin.set(getMargin(), getMargin(), getMargin());
		Vector3 implicitShapeDimensionsWithMargin = stack.allocVector3();
		implicitShapeDimensionsWithMargin.set(implicitShapeDimensions).add(oldMargin);
		Vector3 unScaledImplicitShapeDimensionsWithMargin = stack.allocVector3();
		VectorUtil.div(unScaledImplicitShapeDimensionsWithMargin, implicitShapeDimensionsWithMargin, localScaling);

		super.setLocalScaling(scaling);

		VectorUtil.mul(implicitShapeDimensions, unScaledImplicitShapeDimensionsWithMargin, localScaling);
		implicitShapeDimensions.sub(oldMargin);
		stack.leave();
	}

	@Override
	public void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
	    Stack stack = Stack.enter();
		AabbUtil2.transformAabb(getHalfExtentsWithoutMargin(stack.allocVector3()), getMargin(), t, aabbMin, aabbMax);
		stack.leave();
	}

	@Override
	public void calculateLocalInertia(float mass, Vector3 inertia) {
		//btScalar margin = btScalar(0.);
		Stack stack = Stack.enter();
		Vector3 halfExtents = getHalfExtentsWithMargin(stack.allocVector3());

		float lx = 2f * halfExtents.x;
		float ly = 2f * halfExtents.y;
		float lz = 2f * halfExtents.z;

		inertia.set(mass / 12f * (ly * ly + lz * lz),
				mass / 12f * (lx * lx + lz * lz),
				mass / 12f * (lx * lx + ly * ly));
		stack.leave();
	}

	@Override
	public void getPlane(Vector3 planeNormal, Vector3 planeSupport, int i) {
		// this plane might not be aligned...
	    Stack stack = Stack.enter();
		Quaternion plane = stack.allocQuaternion();
		getPlaneEquation(plane, i);
		planeNormal.set(plane.x, plane.y, plane.z);
		Vector3 tmp = stack.allocVector3();
		tmp.set(planeNormal).scl(-1);
		localGetSupportingVertex(tmp, planeSupport);
		stack.leave();
	}

	@Override
	public int getNumPlanes() {
		return 6;
	}

	@Override
	public int getNumVertices() {
		return 8;
	}

	@Override
	public int getNumEdges() {
		return 12;
	}

	@Override
	public void getVertex(int i, Vector3 vtx) {
	    Stack stack = Stack.enter();
		Vector3 halfExtents = getHalfExtentsWithoutMargin(stack.allocVector3());

		vtx.set(halfExtents.x * (1 - (i & 1)) - halfExtents.x * (i & 1),
				halfExtents.y * (1 - ((i & 2) >> 1)) - halfExtents.y * ((i & 2) >> 1),
				halfExtents.z * (1 - ((i & 4) >> 2)) - halfExtents.z * ((i & 4) >> 2));
		stack.leave();
	}
	
	public void getPlaneEquation(Quaternion plane, int i) {
	    Stack stack = Stack.enter();
		Vector3 halfExtents = getHalfExtentsWithoutMargin(stack.allocVector3());

		switch (i) {
			case 0:
				plane.set(1f, 0f, 0f, -halfExtents.x);
				break;
			case 1:
				plane.set(-1f, 0f, 0f, -halfExtents.x);
				break;
			case 2:
				plane.set(0f, 1f, 0f, -halfExtents.y);
				break;
			case 3:
				plane.set(0f, -1f, 0f, -halfExtents.y);
				break;
			case 4:
				plane.set(0f, 0f, 1f, -halfExtents.z);
				break;
			case 5:
				plane.set(0f, 0f, -1f, -halfExtents.z);
				break;
			default:
				assert (false);
		}
		stack.leave();
	}

	@Override
	public void getEdge(int i, Vector3 pa, Vector3 pb) {
		int edgeVert0 = 0;
		int edgeVert1 = 0;

		switch (i) {
			case 0:
				edgeVert0 = 0;
				edgeVert1 = 1;
				break;
			case 1:
				edgeVert0 = 0;
				edgeVert1 = 2;
				break;
			case 2:
				edgeVert0 = 1;
				edgeVert1 = 3;

				break;
			case 3:
				edgeVert0 = 2;
				edgeVert1 = 3;
				break;
			case 4:
				edgeVert0 = 0;
				edgeVert1 = 4;
				break;
			case 5:
				edgeVert0 = 1;
				edgeVert1 = 5;

				break;
			case 6:
				edgeVert0 = 2;
				edgeVert1 = 6;
				break;
			case 7:
				edgeVert0 = 3;
				edgeVert1 = 7;
				break;
			case 8:
				edgeVert0 = 4;
				edgeVert1 = 5;
				break;
			case 9:
				edgeVert0 = 4;
				edgeVert1 = 6;
				break;
			case 10:
				edgeVert0 = 5;
				edgeVert1 = 7;
				break;
			case 11:
				edgeVert0 = 6;
				edgeVert1 = 7;
				break;
			default:
				assert (false);
		}

		getVertex(edgeVert0, pa);
		getVertex(edgeVert1, pb);
	}

	@Override
	public boolean isInside(Vector3 pt, float tolerance) {
	    Stack stack = Stack.enter();
		Vector3 halfExtents = getHalfExtentsWithoutMargin(stack.allocVector3());

		//btScalar minDist = 2*tolerance;

		boolean result =
				(pt.x <= (halfExtents.x + tolerance)) &&
				(pt.x >= (-halfExtents.x - tolerance)) &&
				(pt.y <= (halfExtents.y + tolerance)) &&
				(pt.y >= (-halfExtents.y - tolerance)) &&
				(pt.z <= (halfExtents.z + tolerance)) &&
				(pt.z >= (-halfExtents.z - tolerance));
		stack.leave();
		return result;
	}

	@Override
	public String getName() {
		return "Box";
	}

	@Override
	public int getNumPreferredPenetrationDirections() {
		return 6;
	}

	@Override
	public void getPreferredPenetrationDirection(int index, Vector3 penetrationVector) {
		switch (index) {
			case 0:
				penetrationVector.set(1f, 0f, 0f);
				break;
			case 1:
				penetrationVector.set(-1f, 0f, 0f);
				break;
			case 2:
				penetrationVector.set(0f, 1f, 0f);
				break;
			case 3:
				penetrationVector.set(0f, -1f, 0f);
				break;
			case 4:
				penetrationVector.set(0f, 0f, 1f);
				break;
			case 5:
				penetrationVector.set(0f, 0f, -1f);
				break;
			default:
				assert (false);
		}
	}

}

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

import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.Stack;

/** Single triangle shape.
 * 
 * @author jezek2 */
public class TriangleShape extends PolyhedralConvexShape {

	public final Vector3[] vertices1/* [3] */= new Vector3[] {new Vector3(), new Vector3(), new Vector3()};

	// JAVA NOTE: added
	public TriangleShape () {
	}

	public TriangleShape (Vector3 p0, Vector3 p1, Vector3 p2) {
		vertices1[0].set(p0);
		vertices1[1].set(p1);
		vertices1[2].set(p2);
	}

	// JAVA NOTE: added
	public void init (Vector3 p0, Vector3 p1, Vector3 p2) {
		vertices1[0].set(p0);
		vertices1[1].set(p1);
		vertices1[2].set(p2);
	}

	@Override
	public int getNumVertices () {
		return 3;
	}

	public Vector3 getVertexPtr (int index) {
		return vertices1[index];
	}

	@Override
	public void getVertex (int index, Vector3 vert) {
		vert.set(vertices1[index]);
	}

	@Override
	public BroadphaseNativeType getShapeType () {
		return BroadphaseNativeType.TRIANGLE_SHAPE_PROXYTYPE;
	}

	@Override
	public int getNumEdges () {
		return 3;
	}

	@Override
	public void getEdge (int i, Vector3 pa, Vector3 pb) {
		getVertex(i, pa);
		getVertex((i + 1) % 3, pb);
	}

	@Override
	public void getAabb (Transform t, Vector3 aabbMin, Vector3 aabbMax) {
// btAssert(0);
		getAabbSlow(t, aabbMin, aabbMax);
	}

	@Override
	public Vector3 localGetSupportingVertexWithoutMargin (Vector3 dir, Vector3 out) {
		Stack stack = Stack.enter();
		Vector3 dots = stack.allocVector3();
		dots.set(dir.dot(vertices1[0]), dir.dot(vertices1[1]), dir.dot(vertices1[2]));
		out.set(vertices1[VectorUtil.maxAxis(dots)]);
		stack.leave();
		return out;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin (Vector3[] vectors, Vector3[] supportVerticesOut, int numVectors) {
		Stack stack = Stack.enter();
		Vector3 dots = stack.allocVector3();

		for (int i = 0; i < numVectors; i++) {
			Vector3 dir = vectors[i];
			dots.set(dir.dot(vertices1[0]), dir.dot(vertices1[1]), dir.dot(vertices1[2]));
			supportVerticesOut[i].set(vertices1[VectorUtil.maxAxis(dots)]);
		}
		stack.leave();
	}

	@Override
	public void getPlane (Vector3 planeNormal, Vector3 planeSupport, int i) {
		getPlaneEquation(i, planeNormal, planeSupport);
	}

	@Override
	public int getNumPlanes () {
		return 1;
	}

	public void calcNormal (Vector3 normal) {
		Stack stack = Stack.enter();
		Vector3 tmp1 = stack.allocVector3();
		Vector3 tmp2 = stack.allocVector3();

		tmp1.set(vertices1[1]).sub(vertices1[0]);
		tmp2.set(vertices1[2]).sub(vertices1[0]);

		normal.set(tmp1).crs(tmp2);
		normal.nor();
		stack.leave();
	}

	public void getPlaneEquation (int i, Vector3 planeNormal, Vector3 planeSupport) {
		calcNormal(planeNormal);
		planeSupport.set(vertices1[0]);
	}

	@Override
	public void calculateLocalInertia (float mass, Vector3 inertia) {
		assert (false);
		inertia.set(0f, 0f, 0f);
	}

	@Override
	public boolean isInside (Vector3 pt, float tolerance) {
		Stack stack = Stack.enter();
		Vector3 normal = stack.allocVector3();
		calcNormal(normal);
		// distance to plane
		float dist = pt.dot(normal);
		float planeconst = vertices1[0].dot(normal);
		dist -= planeconst;
		if (dist >= -tolerance && dist <= tolerance) {
			// inside check on edge-planes
			int i;
			for (i = 0; i < 3; i++) {
				Vector3 pa = stack.allocVector3(), pb = stack.allocVector3();
				getEdge(i, pa, pb);
				Vector3 edge = stack.allocVector3();
				edge.set(pb).sub(pa);
				Vector3 edgeNormal = stack.allocVector3();
				edgeNormal.set(edge).crs(normal);
				edgeNormal.nor();
				/* float */dist = pt.dot(edgeNormal);
				float edgeConst = pa.dot(edgeNormal);
				dist -= edgeConst;
				if (dist < -tolerance) {
					stack.leave();
					return false;
				}
			}
			stack.leave();
			return true;
		}
		stack.leave();
		return false;
	}

	@Override
	public String getName () {
		return "Triangle";
	}

	@Override
	public int getNumPreferredPenetrationDirections () {
		return 2;
	}

	@Override
	public void getPreferredPenetrationDirection (int index, Vector3 penetrationVector) {
		calcNormal(penetrationVector);
		if (index != 0) {
			penetrationVector.scl(-1f);
		}
	}

}

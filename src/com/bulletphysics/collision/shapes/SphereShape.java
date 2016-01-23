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
import com.bulletphysics.util.Stack;

/**
 * SphereShape implements an implicit sphere, centered around a local origin with radius.
 * 
 * @author jezek2
 */
public class SphereShape extends ConvexInternalShape {
	
	public SphereShape(float radius) {
		implicitShapeDimensions.x = radius;
		collisionMargin = radius;
	}

	@Override
	public Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
		out.set(0f, 0f, 0f);
		return out;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut, int numVectors) {
		for (int i = 0; i < numVectors; i++) {
			supportVerticesOut[i].set(0f, 0f, 0f);
		}
	}

	@Override
	public void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
	    Stack stack = Stack.enter();
		Vector3 center = t.origin;
		Vector3 extent = stack.allocVector3();
		extent.set(getMargin(), getMargin(), getMargin());
		aabbMin.set(center).sub(extent);
		aabbMax.set(center).add(extent);
		stack.leave();
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.SPHERE_SHAPE_PROXYTYPE;
	}

	@Override
	public void calculateLocalInertia(float mass, Vector3 inertia) {
		float elem = 0.4f * mass * getMargin() * getMargin();
		inertia.set(elem, elem, elem);
	}

	@Override
	public String getName() {
		return "SPHERE";
	}
	
	public float getRadius() {
		return implicitShapeDimensions.x * localScaling.x;
	}

	@Override
	public void setMargin(float margin) {
		super.setMargin(margin);
	}

	@Override
	public float getMargin() {
		// to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
		// this means, non-uniform scaling is not supported anymore
		return getRadius();
	}
	
}

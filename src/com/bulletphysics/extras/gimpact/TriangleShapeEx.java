/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * This source file is part of GIMPACT Library.
 *
 * For the latest info, see http://gimpact.sourceforge.net/
 *
 * Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
 * email: projectileman@yahoo.com
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

package com.bulletphysics.extras.gimpact;

import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.collision.shapes.TriangleShape;
import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.Stack;

/** @author jezek2 */
public class TriangleShapeEx extends TriangleShape {

	public TriangleShapeEx () {
		super();
	}

	public TriangleShapeEx (Vector3 p0, Vector3 p1, Vector3 p2) {
		super(p0, p1, p2);
	}

	@Override
	public void getAabb (Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		Stack stack = Stack.enter();
		Vector3 tv0 = stack.alloc(vertices1[0]);
		t.transform(tv0);
		Vector3 tv1 = stack.alloc(vertices1[1]);
		t.transform(tv1);
		Vector3 tv2 = stack.alloc(vertices1[2]);
		t.transform(tv2);

		AABB trianglebox = stack.allocAABB();
		trianglebox.init(tv0, tv1, tv2, collisionMargin);

		aabbMin.set(trianglebox.min);
		aabbMax.set(trianglebox.max);
		stack.leave();
	}

	public void applyTransform (Transform t) {
		t.transform(vertices1[0]);
		t.transform(vertices1[1]);
		t.transform(vertices1[2]);
	}

	public void buildTriPlane (Quaternion plane) {
		Stack stack = Stack.enter();
		Vector3 tmp1 = stack.allocVector3();
		Vector3 tmp2 = stack.allocVector3();

		Vector3 normal = stack.allocVector3();
		tmp1.set(vertices1[1]).sub(vertices1[0]);
		tmp2.set(vertices1[2]).sub(vertices1[0]);
		normal.set(tmp1).crs(tmp2);
		normal.nor();

		plane.set(normal.x, normal.y, normal.z, vertices1[0].dot(normal));
		stack.leave();
	}

	public boolean overlap_test_conservative (TriangleShapeEx other) {
		float total_margin = getMargin() + other.getMargin();

		Stack stack = Stack.enter();
		Quaternion plane0 = stack.allocQuaternion();
		buildTriPlane(plane0);
		Quaternion plane1 = stack.allocQuaternion();
		other.buildTriPlane(plane1);

		// classify points on other triangle
		float dis0 = ClipPolygon.distance_point_plane(plane0, other.vertices1[0]) - total_margin;

		float dis1 = ClipPolygon.distance_point_plane(plane0, other.vertices1[1]) - total_margin;

		float dis2 = ClipPolygon.distance_point_plane(plane0, other.vertices1[2]) - total_margin;

		if (dis0 > 0.0f && dis1 > 0.0f && dis2 > 0.0f) {
			stack.leave();
			return false; // classify points on this triangle
		}
		dis0 = ClipPolygon.distance_point_plane(plane1, vertices1[0]) - total_margin;

		dis1 = ClipPolygon.distance_point_plane(plane1, vertices1[1]) - total_margin;

		dis2 = ClipPolygon.distance_point_plane(plane1, vertices1[2]) - total_margin;

		stack.leave();
		if (dis0 > 0.0f && dis1 > 0.0f && dis2 > 0.0f) {
			return false;
		}
		return true;
	}

}

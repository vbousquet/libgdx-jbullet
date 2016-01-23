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
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.Stack;

/** @author jezek2 */
class PlaneShape {

	public static void get_plane_equation (StaticPlaneShape shape, Quaternion equation) {
		Stack stack = Stack.enter();
		Vector3 tmp = stack.allocVector3();
		equation.set(shape.getPlaneNormal(tmp), 0);
		equation.w = shape.getPlaneConstant();
		stack.leave();
	}

	public static void get_plane_equation_transformed (StaticPlaneShape shape, Transform trans, Quaternion equation) {
		Stack stack = Stack.enter();
		get_plane_equation(shape, equation);

		Vector3 tmp = stack.allocVector3();

		MatrixUtil.getRow(trans.basis, 0, tmp);
		float x = Vector3.dot(tmp.x, tmp.y, tmp.z, equation.x, equation.y, equation.z);
		MatrixUtil.getRow(trans.basis, 1, tmp);
		float y = Vector3.dot(tmp.x, tmp.y, tmp.z, equation.x, equation.y, equation.z);
		MatrixUtil.getRow(trans.basis, 2, tmp);
		float z = Vector3.dot(tmp.x, tmp.y, tmp.z, equation.x, equation.y, equation.z);

		float w = Vector3.dot(trans.origin.x, trans.origin.y, trans.origin.z, equation.x, equation.y, equation.z) + equation.w;

		equation.set(x, y, z, w);
		stack.leave();
	}

}

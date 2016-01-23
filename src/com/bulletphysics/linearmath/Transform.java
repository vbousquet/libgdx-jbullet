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

package com.bulletphysics.linearmath;

import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.collision.shapes.UniformScalingShape;
import com.bulletphysics.util.Stack;
import com.bulletphysics.util.StaticAlloc;

/**
 * Transform represents translation and rotation (rigid transform). Scaling and
 * shearing is not supported.<p>
 * 
 * You can use local shape scaling or {@link UniformScalingShape} for static rescaling
 * of collision objects.
 * 
 * @author jezek2
 */
public class Transform {
	
	//protected BulletStack stack;

	/** Rotation matrix of this Transform. */
	public final Matrix3 basis = new Matrix3();
	
	/** Translation vector of this Transform. */
	public final Vector3 origin = new Vector3();

	public Transform() {
	}

	public Transform(Matrix3 mat) {
		basis.set(mat);
	}

	public Transform(Matrix4 mat) {
		set(mat);
	}

	public Transform(Transform tr) {
		set(tr);
	}
	
	public void set(Transform tr) {
		basis.set(tr.basis);
		origin.set(tr.origin);
	}
	
	public void set(Matrix3 mat) {
		basis.set(mat);
		origin.set(0f, 0f, 0f);
	}

	public void set(Matrix4 mat) {
		basis.val[Matrix3.M00] = mat.val[Matrix4.M00];
		basis.val[Matrix3.M01] = mat.val[Matrix4.M01];
		basis.val[Matrix3.M02] = mat.val[Matrix4.M02];
		basis.val[Matrix3.M10] = mat.val[Matrix4.M10];
		basis.val[Matrix3.M11] = mat.val[Matrix4.M11];
		basis.val[Matrix3.M12] = mat.val[Matrix4.M12];
		basis.val[Matrix3.M20] = mat.val[Matrix4.M20];
		basis.val[Matrix3.M21] = mat.val[Matrix4.M21];
		basis.val[Matrix3.M22] = mat.val[Matrix4.M22];
		origin.set(mat.val[Matrix4.M03], mat.val[Matrix4.M13], mat.val[Matrix4.M23]);
	}
	
	public void transform(Vector3 v) {
		v.mul(basis);
		v.add(origin);
	}

	public void setIdentity() {
		basis.idt();
		origin.set(0f, 0f, 0f);
	}
	
	public void inverse() {
		basis.transpose();
		origin.scl(-1f);
		origin.mul(basis);
	}

	public void inverse(Transform tr) {
		set(tr);
		inverse();
	}
	
	public void mul(Transform tr) {
	    Stack stack = Stack.enter();
		Vector3 vec = stack.alloc(tr.origin);
		transform(vec);

		basis.mul(tr.basis);
		origin.set(vec);
		stack.leave();
	}

	@StaticAlloc
	public void mul(Transform tr1, Transform tr2) {
	    Stack stack = Stack.enter();
		Vector3 vec = stack.alloc(tr2.origin);
		tr1.transform(vec);

		basis.set(tr1.basis).mul(tr2.basis);
		origin.set(vec);
		stack.leave();
	}
	
	public void invXform(Vector3 inVec, Vector3 out) {
		out.set(inVec).sub(origin);
		Stack stack = Stack.enter();
		Matrix3 mat = stack.alloc(basis);
		mat.transpose();
		out.mul(mat);
		stack.leave();
	}
	
	public Quaternion getRotation(Quaternion out) {
		MatrixUtil.getRotation(basis, out);
		return out;
	}
	
	public void setRotation(Quaternion q) {
		MatrixUtil.setRotation(basis, q);
	}
	
	public void setFromOpenGLMatrix(float[] m) {
		MatrixUtil.setFromOpenGLSubMatrix(basis, m);
		origin.set(m[12], m[13], m[14]);
	}

	public void getOpenGLMatrix(float[] m) {
		MatrixUtil.getOpenGLSubMatrix(basis, m);
		m[12] = origin.x;
		m[13] = origin.y;
		m[14] = origin.z;
		m[15] = 1f;
	}

	public Matrix4 getMatrix(Matrix4 out) {
		out.val[Matrix4.M00] = basis.val[Matrix3.M00];
		out.val[Matrix4.M01] = basis.val[Matrix3.M01];
		out.val[Matrix4.M02] = basis.val[Matrix3.M02];
		out.val[Matrix4.M03] = origin.x;
		out.val[Matrix4.M10] = basis.val[Matrix3.M10];
		out.val[Matrix4.M11] = basis.val[Matrix3.M11];
		out.val[Matrix4.M12] = basis.val[Matrix3.M12];
		out.val[Matrix4.M13] = origin.y;
		out.val[Matrix4.M20] = basis.val[Matrix3.M20];
		out.val[Matrix4.M21] = basis.val[Matrix3.M21];
		out.val[Matrix4.M22] = basis.val[Matrix3.M22];
		out.val[Matrix4.M23] = origin.z;
		out.val[Matrix4.M30] = 0;
		out.val[Matrix4.M31] = 0;
		out.val[Matrix4.M32] = 0;
		out.val[Matrix4.M33] = 1;
		return out;
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == null || !(obj instanceof Transform)) return false;
		Transform tr = (Transform)obj;
		return basis.equals(tr.basis) && origin.equals(tr.origin);
	}

	@Override
	public int hashCode() {
		int hash = 3;
		hash = 41 * hash + basis.hashCode();
		hash = 41 * hash + origin.hashCode();
		return hash;
	}
	
}

package com.bulletphysics.util;

import com.badlogic.gdx.math.Vector3;
import com.bulletphysics.linearmath.Transform;


public class Suppliers {

	public static final Supplier<Vector3> NEW_VECTOR3_SUPPLIER = new Supplier<Vector3>() {
		@Override
		public Vector3 get() {
			return new Vector3();
		}
	};

	public static final Supplier<Transform> NEW_TRANSFORM_SUPPLIER = new Supplier<Transform>() {
		@Override
		public Transform get() {
			return new Transform();
		}
	};

}

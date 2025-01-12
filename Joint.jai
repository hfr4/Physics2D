Joint :: struct {
	A, B:           *Body;
	bias_factor:    float;
	softness:       float;
	A_local_anchor: Vector2;
    B_local_anchor: Vector2;

    bias:                Vector2;
	combined_mass:       Matrix2;
	accumulated_impulse: Vector2;
}

InitJoint :: (using joint: *Joint, a: *Body, world_point_on_A: Vector2, b: *Body, world_point_on_B: Vector2, bf := 0.2, s := 0.0) {
	A              = a;
	B              = b;
	bias_factor    = bf;
	softness       = s;
	A_local_anchor = WorldToLocal(A, world_point_on_A);
	B_local_anchor = WorldToLocal(B, world_point_on_B);
}

InitJoint :: (using joint: *Joint, a: *Body, b: *Body, world_point: Vector2, bf := 0.2, s := 0.0) {
	InitJoint(joint, a, world_point, b, world_point, bf, s);
}

PreStep :: (using joint: *Joint, dt: float) {
	A_world_anchor := LocalToWorld(A, A_local_anchor);
	B_world_anchor := LocalToWorld(B, B_local_anchor);

	pA := A_world_anchor - A.position;
	pB := B_world_anchor - B.position;

	inv_mass_matrix := (A.inv_mass + B.inv_mass + softness) * Matrix2_Identity;

	inv_inertia_matrix :=
		A.inv_inertia * Matrix2.{ pA.y * pA.y, -pA.x * pA.y, -pA.x * pA.y, pA.x * pA.x } +
		B.inv_inertia * Matrix2.{ pB.y * pB.y, -pB.x * pB.y, -pB.x * pB.y, pB.x * pB.x };

	combined_mass = Inverse(inv_mass_matrix + inv_inertia_matrix);

	separation := B_world_anchor - A_world_anchor;
	bias = -bias_factor * separation * (ifx dt > 0.0 then 1.0 / dt else 0.0);

	ApplyImpulseAt(A, -accumulated_impulse, pA);
	ApplyImpulseAt(B,  accumulated_impulse, pB);
}

Step :: (using joint: *Joint) {
	pA := LocalToWorld(A, A_local_anchor) - A.position;
	pB := LocalToWorld(B, B_local_anchor) - B.position;

    velocity :=
		(B.linear_velocity + B.angular_velocity * Perpendicular(pB)) -
		(A.linear_velocity + A.angular_velocity * Perpendicular(pA));

	impulse := combined_mass * (bias - velocity - accumulated_impulse * softness);

	ApplyImpulseAt(A, -impulse, pA);
	ApplyImpulseAt(B,  impulse, pB);

	accumulated_impulse += impulse;
}

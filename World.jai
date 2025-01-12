World :: struct {
	bodies:     [..] *Body;
	joints:     [..] *Joint;
	collisions: Table(Collision_Key, Collision, CollisionKeyHash, ColisionKeyCompare);
}

Add :: (using world: *World, body: *Body) {
	array_add(*bodies, body);
}

Add :: (using world: *World, joint: *Joint) {
	array_add(*joints, joint);
}

Reset :: (using world: *World) {
	array_reset(*bodies);
	array_reset(*joints);
	table_reset(*collisions);
}

Update :: (using world: *World, dt: float) {
	// Broadphase
	for i: 0..bodies.count - 2 {
		for j: i + 1..bodies.count - 1 {
			A := bodies[i];
			B := bodies[j];

			if !IsStatic(A) || !IsStatic(B) {
				// Narrowphase
				new_collision := Collide(it.A, it.B);
				if new_collision.contacts_count > 0 {
					old_collision := table_find_pointer(*collisions, it);
					if old_collision {
						Merge(old_collision, new_collision);
					} else {
						table_add(*collisions, it, new_collision);
					}
				} else {
					table_remove(*collisions, it);
				}
			}
		}
	}

	// Integrate forces
	for bodies {
		if IsStatic(it) continue;
		it.linear_velocity  += it.inv_mass    * it.force_accumulator  * dt;
		it.angular_velocity += it.inv_inertia * it.torque_accumulator * dt;
	}

	// Apply impulses
	for * collisions PreStep(it, dt);
	for   joints     PreStep(it, dt);

	ITERATIONS :: 10;
	for 1..ITERATIONS {
    	for * collisions Step(it);
		for   joints     Step(it);
	}

	// Integrate velocities
	for bodies {
		it.position += it.linear_velocity  * dt;
		it.rotation += it.angular_velocity * dt;

		it.force_accumulator  = .{};
		it.torque_accumulator = 0.0;
	}
}

ApplyGravity :: (using world: *World, gravity: Vector2) {
	for bodies {
		if IsStatic(it) continue;
		weight := it.mass * gravity;
        it.force_accumulator += weight;
	}
}

ApplyFriction :: (using world: *World, linear_friction: float, angular_friction: float) {
	for bodies {
		if IsStatic(it) continue;
        force  := -it.linear_velocity  * it.mass    * linear_friction;
        torque := -it.angular_velocity * it.inertia * angular_friction;
        AddForce(it, force);
        AddTorque(it, torque);
    }
}
